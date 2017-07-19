package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class SwingCoPTrajectory extends CoPTrajectory
{
   private final static WalkingTrajectoryType type = WalkingTrajectoryType.SWING;   
   public SwingCoPTrajectory(String namePrefix, int stepNumber, CoPSplineType splineType, int maxNumberOfSegments, YoVariableRegistry registry)
   {
      super(namePrefix, stepNumber, splineType, maxNumberOfSegments, type, registry);
   }

}