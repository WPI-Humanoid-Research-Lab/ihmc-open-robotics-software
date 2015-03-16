package us.ihmc.atlas.ObstacleCourseTests;

import java.util.List;

import javax.vecmath.Vector3d;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseWobblyFootTest;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.utilities.code.agileTesting.BambooPlanType;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

@BambooPlan(planType = {BambooPlanType.Slow, BambooPlanType.VideoA})
public class AtlasObstacleCourseWobblyFootTest extends DRCObstacleCourseWobblyFootTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      final AtlasRobotVersion atlasVersion = AtlasRobotVersion.DRC_NO_HANDS_UNPLUGGED_V4;

      DRCRobotModel robotModel = new AtlasRobotModel(atlasVersion, AtlasRobotModel.AtlasTarget.SIM, false)
      {
         @Override
         public AtlasJointMap getJointMap()
         {
            return createJointMapWithWobblyFeet(getAtlasVersion());
         }
      };

      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   protected DoubleYoVariable getPelvisOrientationErrorVariableName(SimulationConstructionSet scs)
   {
      return (DoubleYoVariable) scs.getVariable(
          "MomentumBasedControllerFactory.PelvisOrientationManager.RootJointAngularAccelerationControlModule.pelvisAxisAngleOrientationController",
          "pelvisOrientationErrorMagnitude");
   }

   private AtlasJointMap createJointMapWithWobblyFeet(final AtlasRobotVersion atlasVersion)
   {
      AtlasJointMap atlasJointMap = new AtlasJointMap(atlasVersion)
      {
         @Override
         public List<Pair<String, Vector3d>> getJointNameGroundContactPointMap()
         {
            return createWobblyContactPoints(this, atlasVersion).getJointNameGroundContactPointMap();
         }
      };

      return atlasJointMap;
   }

   private AtlasContactPointParameters createWobblyContactPoints(DRCRobotJointMap jointMap, AtlasRobotVersion atlasVersion)
   {
      AtlasContactPointParameters contactPointParameters = new AtlasContactPointParameters(jointMap, atlasVersion, false);
      double footZWobbleForTests = 0.01;
      contactPointParameters.createWobblyFootContactPoints(footZWobbleForTests);
      return contactPointParameters;
   }
}
