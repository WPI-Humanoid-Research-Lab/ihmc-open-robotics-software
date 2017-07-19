package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.AbstractICPPlanner;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class SmoothCMPBasedICPPlanner extends AbstractICPPlanner
{
   private static final boolean VISUALIZE = true;

   private final ReferenceCoPTrajectoryGenerator referenceCoPGenerator;
   private final ReferenceCMPTrajectoryGenerator referenceCMPGenerator;
   private final ReferenceICPTrajectoryGenerator referenceICPGenerator;

   private final List<YoDouble> swingDurationShiftFractions = new ArrayList<>();

   private final YoInteger numberOfFootstepsToConsider;
   private final CoPPointName exitCoPName;
   private final CoPPointName entryCoPName;
   private final CoPPointName endCoPName;

   public SmoothCMPBasedICPPlanner(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                   CapturePointPlannerParameters icpPlannerParameters, SmoothCMPPlannerParameters plannerParameters,
                                   YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(bipedSupportPolygons, icpPlannerParameters);

      int numberOfFootstepsToConsider = plannerParameters.getNumberOfFootstepsToConsider();
      this.numberOfFootstepsToConsider = new YoInteger(namePrefix + "NumberOfFootstepsToConsider", registry);
      this.numberOfFootstepsToConsider.set(numberOfFootstepsToConsider);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         YoDouble swingDurationShiftFraction = new YoDouble("swingDurationShiftFraction" + i, registry);
         swingDurationShiftFraction.set(plannerParameters.getSwingDurationShiftFraction());
         swingDurationShiftFractions.add(swingDurationShiftFraction);
      }

      referenceCoPGenerator = new ReferenceCoPTrajectoryGenerator(namePrefix, plannerParameters, bipedSupportPolygons, contactableFeet,
                                                                    this.numberOfFootstepsToConsider, swingDurations, transferDurations,
                                                                    swingDurationAlphas, swingDurationShiftFractions, transferDurationAlphas,
                                                                    registry);

      referenceCMPGenerator = new ReferenceCMPTrajectoryGenerator(namePrefix, this.numberOfFootstepsToConsider, swingDurations, transferDurations,
                                                                  swingDurationAlphas, transferDurationAlphas, registry);

      referenceICPGenerator = new ReferenceICPTrajectoryGenerator(namePrefix, omega0, this.numberOfFootstepsToConsider, isStanding, useDecoupled, worldFrame,
                                                                                   registry);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry != null)
      {
         setupVisualizers(yoGraphicsListRegistry);
      }
      this.exitCoPName = plannerParameters.getExitCoPName();
      this.entryCoPName = plannerParameters.getEntryCoPName();
      this.endCoPName = plannerParameters.getEndCoPName();
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      referenceCoPGenerator.createVisualizerForConstantCoPs(yoGraphicsList, artifactList);

      artifactList.setVisible(VISUALIZE);
      yoGraphicsList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   @Override
   /** {@inheritDoc} */
   public void clearPlan()
   {
      referenceCoPGenerator.clear();
      referenceCMPGenerator.reset();
      referenceICPGenerator.reset();

      for (int i = 0; i < swingDurations.size(); i++)
      {
         swingDurations.get(i).setToNaN();
         transferDurations.get(i).setToNaN();
         swingDurationAlphas.get(i).setToNaN();
         transferDurationAlphas.get(i).setToNaN();
      }
   }

   @Override
   /** {@inheritDoc} */
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      referenceCoPGenerator.addFootstepToPlan(footstep, timing);
   }

   @Override
   /** {@inheritDoc} */
   public void initializeForStanding(double initialTime)
   {
      clearPlan();
      isStanding.set(true);
      isDoubleSupport.set(true);
      this.initialTime.set(initialTime);
      transferDurations.get(0).set(finalTransferDuration.getDoubleValue());
      transferDurationAlphas.get(0).set(finalTransferDurationAlpha.getDoubleValue());
      updateTransferPlan();
      
      throw new RuntimeException("to implement"); //TODO
   }

   @Override
   /** {@inheritDoc} */
   public void initializeForTransfer(double initialTime)
   {
      this.initialTime.set(initialTime);
      
      RobotSide transferToSide = this.transferToSide.getEnumValue();

      if (isStanding.getBooleanValue())
         referenceCoPGenerator.computeReferenceCoPsStartingFromDoubleSupport(true, transferToSide);
      else
         referenceCoPGenerator.computeReferenceCoPsStartingFromDoubleSupport(false, transferToSide);
      referenceCMPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      referenceICPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      
      referenceCoPGenerator.initializeForTransfer(this.initialTime.getDoubleValue());
      referenceCMPGenerator.initializeForTransfer(this.initialTime.getDoubleValue(), referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories());
      referenceICPGenerator.initializeForTransfer(this.initialTime.getDoubleValue(), referenceCMPGenerator.getTransferCMPTrajectories(), referenceCMPGenerator.getSwingCMPTrajectories());
      
      isStanding.set(false);
      isDoubleSupport.set(false);      
   }

   @Override
   /** {@inheritDoc} */
   public void computeFinalCoMPositionInTransfer()
   {
      throw new RuntimeException("to implement"); //TODOLater
   }

   @Override
   /** {@inheritDoc} */
   public void initializeForSingleSupport(double initialTime)
   {
      this.initialTime.set(initialTime);
      
      RobotSide supportSide = this.supportSide.getEnumValue();
            
      referenceCoPGenerator.computeReferenceCoPsStartingFromSingleSupport(supportSide);
      referenceCMPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      referenceICPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      
      referenceCoPGenerator.initializeForSwing(this.initialTime.getDoubleValue());
      referenceCMPGenerator.initializeForSwing(this.initialTime.getDoubleValue(), referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories());
      referenceICPGenerator.initializeForSwing(this.initialTime.getDoubleValue(), referenceCMPGenerator.getTransferCMPTrajectories(), referenceCMPGenerator.getSwingCMPTrajectories());
      
      isStanding.set(false);
      isDoubleSupport.set(true);
   }

   @Override
   /** {@inheritDoc} */
   public void computeFinalCoMPositionInSwing()
   {
      throw new RuntimeException("to implement"); //TODOLater
   }

   @Override
   /** {@inheritDoc} */
   protected void updateTransferPlan()
   {
      throw new RuntimeException("to implement"); //TODO
   }

   @Override
   /** {@inheritDoc} */
   protected void updateSingleSupportPlan()
   {
      throw new RuntimeException("to implement"); //TODO
   }

   @Override
   /** {@inheritDoc} */
   public void compute(double time)
   {
      referenceCoPGenerator.update(time);
      referenceCMPGenerator.update(time);
      referenceICPGenerator.compute(time);

      referenceCoPGenerator.getDesiredCenterOfPressure(desiredCoP, desiredCoPVelocity);
      referenceCMPGenerator.getDesiredCMP(desiredCMP);
      referenceICPGenerator.getLinearData(desiredICP);
      
      throw new RuntimeException("to implement"); //TODO
   }


   @Override
   /** {@inheritDoc} */
   public void getFinalDesiredCapturePointPosition(FramePoint finalDesiredCapturePointPositionToPack)
   {
      throw new RuntimeException("to implement"); //TODO
   }

   @Override
   /** {@inheritDoc} */
   public void getFinalDesiredCapturePointPosition(YoFramePoint2d finalDesiredCapturePointPositionToPack)
   {
      throw new RuntimeException("to implement"); //TODO
   }

   @Override
   /** {@inheritDoc} */
   public void getFinalDesiredCenterOfMassPosition(FramePoint2d finalDesiredCenterOfMassPositionToPack)
   {
      throw new RuntimeException("to implement"); //TODOLater
   }

   @Override
   /** {@inheritDoc} */
   public void getNextExitCMP(FramePoint entryCMPToPack)
   {
      List<CoPPointsInFoot> plannedCoPWaypoints = referenceCoPGenerator.getWaypoints();
      plannedCoPWaypoints.get(1).get(this.exitCoPName).getPosition(entryCMPToPack);
   }

   @Override
   /** {@inheritDoc} */
   public boolean isOnExitCMP()
   {
      throw new RuntimeException("to implement"); //TODO
   }

   @Override
   /** {@inheritDoc} */
   public int getNumberOfFootstepsToConsider()
   {
      return numberOfFootstepsToConsider.getIntegerValue();
   }

   @Override
   /** {@inheritDoc} */
   public int getNumberOfFootstepsRegistered()
   {
      return referenceCoPGenerator.getNumberOfFootstepsRegistered();
   }
}