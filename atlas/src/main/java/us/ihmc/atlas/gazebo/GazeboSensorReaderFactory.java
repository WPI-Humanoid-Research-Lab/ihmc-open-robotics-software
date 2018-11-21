package us.ihmc.atlas.gazebo;

import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class GazeboSensorReaderFactory implements SensorReaderFactory
{
   private StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;
   private DRCRobotSensorInformation sensorInformation;
   private GazeboSensorReader atlasSensorReader;
   private final StateEstimatorParameters stateEstimatorParameters;

   public GazeboSensorReaderFactory(DRCRobotSensorInformation sensorInformation, StateEstimatorParameters stateEstimatorParameters)
   {
      this.sensorInformation = sensorInformation;     
      this.stateEstimatorParameters = stateEstimatorParameters;
   }

   @Override
   public void build(FloatingInverseDynamicsJoint rootJoint, IMUDefinition[] imuDefinitions, ForceSensorDefinition[] forceSensorDefinitions,
         ContactSensorHolder contactSensorDataHolder, RawJointSensorDataHolderMap rawJointSensorDataHolderMap, JointDesiredOutputList estimatorDesiredJointDataHolder, YoVariableRegistry registry)
   {
      stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();

      for (InverseDynamicsJoint joint : ScrewTools.computeSubtreeJoints(rootJoint.getSuccessor()))
      {
         if (joint instanceof OneDoFJoint)
         {
            OneDoFJoint oneDoFJoint = (OneDoFJoint) joint;
            stateEstimatorSensorDefinitions.addJointSensorDefinition(oneDoFJoint);
         }
      }

      for (IMUDefinition imuDefinition : imuDefinitions)
      {
         stateEstimatorSensorDefinitions.addIMUSensorDefinition(imuDefinition);
      }

      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
      {
         stateEstimatorSensorDefinitions.addForceSensorDefinition(forceSensorDefinition);
      }

      atlasSensorReader = new GazeboSensorReader(stateEstimatorSensorDefinitions, sensorInformation, stateEstimatorParameters,
            rawJointSensorDataHolderMap, registry);
   }

   public GazeboSensorReader getSensorReader()
   {
      return atlasSensorReader;
   }

   public void connectSensorReader() {
	   if(!atlasSensorReader.isConnected())
	   {
		   atlasSensorReader.connect();
	   }
   }
   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   public boolean useStateEstimator()
   {
      return true;
   }

}