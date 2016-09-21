package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

import javax.vecmath.Vector3d;

public abstract class LinearMomentumRateOfChangeControlModule
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final YoVariableRegistry registry;

   protected final YoFrameVector defaultLinearMomentumRateWeight;
   protected final YoFrameVector defaultAngularMomentumRateWeight;
   protected final YoFrameVector highLinearMomentumRateWeight;
   protected final YoFrameVector angularMomentumRateWeight;
   protected final YoFrameVector linearMomentumRateWeight;

   protected final EnumYoVariable<RobotSide> supportLegPreviousTick;
   protected final BooleanYoVariable minimizeAngularMomentumRateZ;

   protected final YoFrameVector controlledCoMAcceleration;

   protected final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
   protected final SpatialForceVector desiredMomentumRate = new SpatialForceVector();
   protected final DenseMatrix64F linearAndAngularZSelectionMatrix = CommonOps.identity(6);

   protected double omega0 = 0.0;
   protected double totalMass;
   protected double gravityZ;

   protected final ReferenceFrame centerOfMassFrame;
   protected final FramePoint centerOfMass;
   protected final FramePoint2d centerOfMass2d = new FramePoint2d();

   protected final FramePoint2d capturePoint = new FramePoint2d();
   protected final FramePoint2d desiredCapturePoint = new FramePoint2d();
   protected final FrameVector2d desiredCapturePointVelocity = new FrameVector2d();
   protected final FramePoint2d finalDesiredCapturePoint = new FramePoint2d();

   protected final FramePoint2d perfectCMP = new FramePoint2d();
   protected final FramePoint2d desiredCMP = new FramePoint2d();

   protected final BipedSupportPolygons bipedSupportPolygons;

   protected final FrameVector2d achievedCoMAcceleration2d = new FrameVector2d();
   protected double desiredCoMHeightAcceleration = 0.0;

   protected RobotSide supportSide = null;
   protected RobotSide transferToSide = null;

   public LinearMomentumRateOfChangeControlModule(String namePrefix, CommonHumanoidReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
         double gravityZ, double totalMass, YoVariableRegistry parentRegistry)
   {
      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.bipedSupportPolygons = bipedSupportPolygons;
      this.totalMass = totalMass;
      this.gravityZ = gravityZ;

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      centerOfMass = new FramePoint(centerOfMassFrame);

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      controlledCoMAcceleration = new YoFrameVector(namePrefix + "ControlledCoMAcceleration", "", centerOfMassFrame, registry);

      defaultLinearMomentumRateWeight = new YoFrameVector(namePrefix + "DefaultLinearMomentumRateWeight", worldFrame, registry);
      defaultAngularMomentumRateWeight = new YoFrameVector(namePrefix + "DefaultAngularMomentumRateWeight", worldFrame, registry);
      highLinearMomentumRateWeight = new YoFrameVector(namePrefix + "HighLinearMomentumRateWeight", worldFrame, registry);
      angularMomentumRateWeight = new YoFrameVector(namePrefix + "AngularMomentumRateWeight", worldFrame, registry);
      linearMomentumRateWeight = new YoFrameVector(namePrefix + "LinearMomentumRateWeight", worldFrame, registry);

      supportLegPreviousTick = EnumYoVariable.create(namePrefix + "SupportLegPreviousTick", "", RobotSide.class, registry, true);
      minimizeAngularMomentumRateZ = new BooleanYoVariable(namePrefix + "MinimizeAngularMomentumRateZ", registry);

      MatrixTools.removeRow(linearAndAngularZSelectionMatrix, 0);
      MatrixTools.removeRow(linearAndAngularZSelectionMatrix, 0);

      angularMomentumRateWeight.set(defaultAngularMomentumRateWeight);
      linearMomentumRateWeight.set(defaultLinearMomentumRateWeight);

      momentumRateCommand.setWeights(0.0, 0.0, 0.0, linearMomentumRateWeight.getX(), linearMomentumRateWeight.getY(), linearMomentumRateWeight.getZ());


      parentRegistry.addChild(registry);
   }


   public void setMomentumWeight(Vector3d angularWeight, Vector3d linearWeight)
   {
      defaultLinearMomentumRateWeight.set(linearWeight);
      defaultAngularMomentumRateWeight.set(angularWeight);
   }

   public void setMomentumWeight(Vector3d linearWeight)
   {
      defaultLinearMomentumRateWeight.set(linearWeight);
   }

   public void setAngularMomentumWeight(Vector3d angularWeight)
   {
      defaultAngularMomentumRateWeight.set(angularWeight);
   }

   public void setHighMomentumWeightForRecovery(Vector3d highLinearWeight)
   {
      highLinearMomentumRateWeight.set(highLinearWeight);
   }

   public void setSupportLeg(RobotSide newSupportSide)
   {
      supportSide = newSupportSide;
   }

   public void setTransferToSide(RobotSide transferToSide)
   {
      this.transferToSide = transferToSide;
   }

   public void setTransferFromSide(RobotSide robotSide)
   {
      if (robotSide != null)
         this.transferToSide = robotSide.getOppositeSide();
   }


   public void setOmega0(double omega0)
   {
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN");
      this.omega0 = omega0;
   }

   public void setCapturePoint(FramePoint2d capturePoint)
   {
      this.capturePoint.setIncludingFrame(capturePoint);
   }

   public void setDesiredCapturePoint(FramePoint2d desiredCapturePoint)
   {
      this.desiredCapturePoint.setIncludingFrame(desiredCapturePoint);
   }

   public void setDesiredCapturePointVelocity(FrameVector2d desiredCapturePointVelocity)
   {
      this.desiredCapturePointVelocity.setIncludingFrame(desiredCapturePointVelocity);
   }

   public void setHighMomentumWeight()
   {
      linearMomentumRateWeight.set(highLinearMomentumRateWeight);
      angularMomentumRateWeight.set(defaultAngularMomentumRateWeight);
   }

   public void setDefaultMomentumWeight()
   {
      linearMomentumRateWeight.set(defaultLinearMomentumRateWeight);
      angularMomentumRateWeight.set(defaultAngularMomentumRateWeight);
   }

   public void setDesiredCenterOfMassHeightAcceleration(double desiredCenterOfMassHeightAcceleration)
   {
      desiredCoMHeightAcceleration = desiredCenterOfMassHeightAcceleration;
   }

   public MomentumRateCommand getMomentumRateCommand()
   {
      return momentumRateCommand;
   }

   public void computeAchievedCMP(FrameVector achievedLinearMomentumRate, FramePoint2d achievedCMPToPack)
   {
      if (achievedLinearMomentumRate.containsNaN())
         return;

      centerOfMass2d.setToZero(centerOfMassFrame);
      centerOfMass2d.changeFrame(worldFrame);

      achievedCoMAcceleration2d.setByProjectionOntoXYPlaneIncludingFrame(achievedLinearMomentumRate);
      achievedCoMAcceleration2d.scale(1.0 / totalMass);
      achievedCoMAcceleration2d.changeFrame(worldFrame);

      achievedCMPToPack.set(achievedCoMAcceleration2d);
      achievedCMPToPack.scale(-1.0 / (omega0 * omega0));
      achievedCMPToPack.add(centerOfMass2d);
   }

   private final FramePoint cmp3d = new FramePoint();
   private final FrameVector groundReactionForce = new FrameVector();

   protected FrameVector computeGroundReactionForce(FramePoint2d cmp2d, double fZ)
   {
      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computePseudoCMP3d(cmp3d, centerOfMass, cmp2d, fZ, totalMass, omega0);

      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computeForce(groundReactionForce, centerOfMass, cmp3d, fZ);
      groundReactionForce.changeFrame(centerOfMassFrame);

      return groundReactionForce;
   }

   public void compute(FramePoint2d desiredCMPPreviousValue, FramePoint2d desiredCMPToPack)
   {
      computeCMPInternal(desiredCMPPreviousValue);

      capturePoint.changeFrame(worldFrame);
      desiredCMP.changeFrame(worldFrame);
      if (desiredCMP.containsNaN())
      {
         desiredCMP.set(capturePoint);
         System.err.println(getClass().getSimpleName() + ": desiredCMP contained NaN. Set it to capturePoint.");
      }

      desiredCMPToPack.setIncludingFrame(desiredCMP);
      desiredCMPToPack.changeFrame(worldFrame);

      supportLegPreviousTick.set(supportSide);

      double fZ = WrenchDistributorTools.computeFz(totalMass, gravityZ, desiredCoMHeightAcceleration);
      FrameVector linearMomentumRateOfChange = computeGroundReactionForce(desiredCMP, fZ);
      linearMomentumRateOfChange.changeFrame(centerOfMassFrame);
      linearMomentumRateOfChange.setZ(linearMomentumRateOfChange.getZ() - totalMass * gravityZ);

      if (linearMomentumRateOfChange.containsNaN())
         throw new RuntimeException("linearMomentumRateOfChange = " + linearMomentumRateOfChange);

      controlledCoMAcceleration.set(linearMomentumRateOfChange);
      controlledCoMAcceleration.scale(1.0 / totalMass);

      if (minimizeAngularMomentumRateZ.getBooleanValue())
      {
         desiredMomentumRate.setToZero(centerOfMassFrame);
         desiredMomentumRate.setLinearPart(linearMomentumRateOfChange);
         momentumRateCommand.set(desiredMomentumRate);
         momentumRateCommand.setSelectionMatrix(linearAndAngularZSelectionMatrix);
      }
      else
      {
         momentumRateCommand.setLinearMomentumRateOfChange(linearMomentumRateOfChange);
      }

      momentumRateCommand.setWeights(angularMomentumRateWeight.getX(), angularMomentumRateWeight.getY(), angularMomentumRateWeight.getZ(),
            linearMomentumRateWeight.getX(), linearMomentumRateWeight.getY(), linearMomentumRateWeight.getZ());
   }

   public abstract void computeCMPInternal(FramePoint2d desiredCMPPreviousValue);

   public void minimizeAngularMomentumRateZ(boolean enable)
   {
      minimizeAngularMomentumRateZ.set(enable);
   }

   public void setFinalDesiredCapturePoint(FramePoint2d finalDesiredCapturePoint)
   {
      this.finalDesiredCapturePoint.setIncludingFrame(finalDesiredCapturePoint);
   }

   public void setPerfectCMP(FramePoint2d perfectCMP)
   {
      this.perfectCMP.setIncludingFrame(perfectCMP);
   }

   public abstract void setDoubleSupportDuration(double doubleSupportDuration);

   public abstract void setSingleSupportDuration(double singleSupportDuration);

   public abstract void clearPlan();

   public abstract void addFootstepToPlan(Footstep footstep);

   public abstract void initializeForStanding();

   public abstract void initializeForSingleSupport();

   public abstract void initializeForTransfer();

   public abstract boolean getUpcomingFootstepSolution(Footstep footstepToPack);

   public abstract void setCMPProjectionArea(FrameConvexPolygon2d areaToProjectInto, FrameConvexPolygon2d safeArea);
}