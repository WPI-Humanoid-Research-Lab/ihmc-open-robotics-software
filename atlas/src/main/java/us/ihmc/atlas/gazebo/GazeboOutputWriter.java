package us.ihmc.atlas.gazebo;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.SocketChannel;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;

import us.ihmc.avatar.SimulatedLowLevelOutputWriter;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class GazeboOutputWriter implements DRCOutputProcessor
{
   private final SocketAddress address = new InetSocketAddress("127.0.0.1", 1235);

   private SocketChannel channel;

   private final YoVariableRegistry registry = new YoVariableRegistry(GazeboSensorReaderFactory.class.getSimpleName());

   private final int estimatorTicksPerControlTick;
   private final int estimatorFrequencyInHz;
   private final ArrayList<OneDoFJoint> joints = new ArrayList<>();
   private JointDesiredOutputList outputList;
   private RawJointSensorDataHolderMap rawJointMap;
   private ByteBuffer jointCommand;
   

// Since the finger joint controller doesn't set the OneDoFJoints used in this writer, this acts as an object communicator for finger joint angles
   private HashMap<String, OneDegreeOfFreedomJoint> fingerJointMap = null;

   public GazeboOutputWriter(DRCRobotModel robotModel)
   {
      estimatorTicksPerControlTick = (int) Math.round(robotModel.getControllerDT() / robotModel.getEstimatorDT());
      estimatorFrequencyInHz = (int) (1.0 / robotModel.getEstimatorDT());
      
   }

   @Override
   public void initialize()
   {
//	   connect();
	   System.out.println("GazeboOutputWriter initialized");
   }

   @Override
   public void processAfterController(long timestamp)
   {
      jointCommand.clear();

      jointCommand.putLong(estimatorTicksPerControlTick);
      jointCommand.putLong(timestamp);
      jointCommand.putLong(estimatorFrequencyInHz);
      
      for (int i = 0; i < joints.size(); i++)
      {
         OneDoFJoint joint = joints.get(i);

         if (fingerJointMap == null || !fingerJointMap.containsKey(joint.getName()))
         {
            if (joint.isUnderPositionControl())
            {
            	jointCommand.putDouble(outputList.getJointDesiredOutput(joint).getDesiredPosition());
            }
            else
            {
            	jointCommand.putDouble(outputList.getJointDesiredOutput(joint).getDesiredTorque());
            }
         }
         else
            jointCommand.putDouble(fingerJointMap.get(joint.getName()).getqDesired()); // fingers are always position controlled
      }

      jointCommand.flip();

      try
      {
         while (jointCommand.hasRemaining())
         {
            channel.write(jointCommand);
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
   
   public void setFingerJointsProvider(SideDependentList<List<OneDegreeOfFreedomJoint>> allFingerJoints)
   {
      fingerJointMap = new HashMap<String, OneDegreeOfFreedomJoint>();

      for (RobotSide robotSide : RobotSide.values)
      {
         for (OneDegreeOfFreedomJoint joint : allFingerJoints.get(robotSide))
         {
            fingerJointMap.put(joint.getName(), joint);
         }
      }
   }

   @Override
   public void setLowLevelControllerCoreOutput(FullHumanoidRobotModel controllerRobotModel, JointDesiredOutputList lowLevelControllerCoreOutput, RawJointSensorDataHolderMap rawJointSensorDataHolderMap)
   {
      outputList = lowLevelControllerCoreOutput;
      rawJointMap = rawJointSensorDataHolderMap;
//      controllerRobotModel.getOneDoFJoints(joints);
      joints.clear();
      joints.addAll(Arrays.asList(controllerRobotModel.getOneDoFJoints()));
      Collections.sort(joints, new Comparator<OneDoFJoint>()
      {

         @Override public int compare(OneDoFJoint o1, OneDoFJoint o2)
         {
            return o1.getName().compareTo(o2.getName());
         }
      });

      jointCommand = ByteBuffer.allocate(joints.size() * 8 + 24);
      jointCommand.order(ByteOrder.nativeOrder());

      System.out.println("num of joints = " + joints.size());
   }

   private void sendInitialState() throws IOException
   {
      jointCommand.clear();
      jointCommand.putLong(estimatorTicksPerControlTick * 3);
      jointCommand.putLong(0);
      jointCommand.putLong(1000);
      for (int i = 0; i < joints.size(); i++)
      {
         jointCommand.putDouble(0.0);
      }
      jointCommand.flip();

      while (jointCommand.hasRemaining())
      {
         channel.write(jointCommand);
      }
   }

   public void connect()
   {
//      try
//      {
//
//      }
//      catch (IOException e)
//      {
//         throw new RuntimeException(e);
//      }

      boolean isConnected = false;
      System.out.println("[GazeboOutputWriter] Connecting to " + address);
      while(!isConnected)
      {
         try
         {
            channel = SocketChannel.open();
            channel.configureBlocking(true);
            channel.socket().setKeepAlive(true);
            channel.socket().setReuseAddress(true);
            channel.socket().setSoLinger(false, 0);
            channel.socket().setTcpNoDelay(true);

            channel.connect(address);
            isConnected = true;
            sendInitialState();
         }
         catch (IOException e)
         {
            System.out.println("Connect failed.");
            try
            {
               channel.close();
            }
            catch (IOException e1)
            {
               e1.printStackTrace();
            }
            ThreadTools.sleep(3000);
            isConnected = false;
         }
      }

      System.out.println("[GazeboOutputWriter] Connected");
      System.out.println("num of joints = " + joints.size());
   }

   @Override
   public void setForceSensorDataHolderForController(ForceSensorDataHolderReadOnly forceSensorDataHolderForController)
   {

   }

   @Override
   public YoVariableRegistry getControllerYoVariableRegistry()
   {
      return registry;
   }


}
