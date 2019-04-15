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
import java.util.Iterator;
import java.util.List;

import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.SimulatedLowLevelOutputWriter;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotiq.model.RobotiqHandModel;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class GazeboOutputWriter implements DRCOutputProcessor
{
	private final SocketAddress address = new InetSocketAddress("127.0.0.1", 1235);

	private SocketChannel channel;

	private final YoVariableRegistry registry = new YoVariableRegistry(GazeboSensorReaderFactory.class.getSimpleName());
	private final HumanoidFloatingRootJointRobot simulatedRobot;
	private final SideDependentList<List<OneDegreeOfFreedomJoint>> allFingerJoints = new SideDependentList<>();
	private final int estimatorTicksPerControlTick;
	private final int estimatorFrequencyInHz;
	private final ArrayList<OneDoFJoint> joints = new ArrayList<>();
	private JointDesiredOutputList outputList;
	private RawJointSensorDataHolderMap rawJointMap;
	private ByteBuffer jointCommand;


	// Since the finger joint controller doesn't set the OneDoFJoints used in this writer, this acts as an object communicator for finger joint angles
	private HashMap<String, OneDegreeOfFreedomJoint> fingerJointMap = null;

	public GazeboOutputWriter(DRCRobotModel robotModel, AtlasRobotVersion atlasVersion)
	{
		estimatorTicksPerControlTick = (int) Math.round(robotModel.getControllerDT() / robotModel.getEstimatorDT());
		estimatorFrequencyInHz = (int) (1.0 / robotModel.getEstimatorDT());
		simulatedRobot = robotModel.createHumanoidFloatingRootJointRobot(false);
		if(atlasVersion == AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ) 
		{
			populateFingerMap();
		}
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
			//			System.out.println(joint.getName());
			if(joint.getName().equals("hokuyo_joint")) {
				jointCommand.putDouble(Double.NaN);
				continue;
			}
			if(joint.getName().endsWith("palm_finger_middle_joint")) {
				jointCommand.putDouble(0.0); // palm_finger_middle_joint is a fixed joint
				continue;
			}
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
				jointCommand.putDouble(fingerJointMap.get(joint.getName()).getqDesired()); // Just set the position that user requested
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

	private void populateFingerMap() {

		RobotiqHandModel handModel = new RobotiqHandModel();
		for (RobotSide robotSide :RobotSide.values )
		{	   
			allFingerJoints.put(robotSide, new ArrayList<OneDegreeOfFreedomJoint>());

			for (HandJointName jointEnum : handModel.getHandJointNames())
			{
				OneDegreeOfFreedomJoint fingerJoint = simulatedRobot.getOneDegreeOfFreedomJoint(jointEnum.getJointName(robotSide));
				allFingerJoints.get(robotSide).add(fingerJoint);
			}
		}

		setFingerJointsProvider(allFingerJoints);
		return;
		//		fingerJointMap = new HashMap<String, OneDegreeOfFreedomJoint>();
		//		fingerJointMap.put("l_finger_1_joint_1", null);
		//		fingerJointMap.put("l_finger_1_joint_2", null);
		//		fingerJointMap.put("l_finger_1_joint_3", null);
		//		fingerJointMap.put("l_finger_2_joint_1", null);
		//		fingerJointMap.put("l_finger_2_joint_2", null);
		//		fingerJointMap.put("l_finger_2_joint_3", null);
		//		fingerJointMap.put("l_finger_middle_joint_1", null);
		//		fingerJointMap.put("l_finger_middle_joint_2", null);
		//		fingerJointMap.put("l_finger_middle_joint_3", null);
		//		fingerJointMap.put("l_palm_finger_1_joint", null);
		//		fingerJointMap.put("l_palm_finger_2_joint", null);
		//		fingerJointMap.put("l_palm_finger_middle_joint", null);
		//		fingerJointMap.put("r_finger_1_joint_1", null);
		//		fingerJointMap.put("r_finger_1_joint_2", null);
		//		fingerJointMap.put("r_finger_1_joint_3", null);
		//		fingerJointMap.put("r_finger_2_joint_1", null);
		//		fingerJointMap.put("r_finger_2_joint_2", null);
		//		fingerJointMap.put("r_finger_2_joint_3", null);
		//		fingerJointMap.put("r_finger_middle_joint_1", null);
		//		fingerJointMap.put("r_finger_middle_joint_2", null);
		//		fingerJointMap.put("r_finger_middle_joint_3", null);
		//		fingerJointMap.put("r_palm_finger_1_joint", null);
		//		fingerJointMap.put("r_palm_finger_2_joint", null);
		//		fingerJointMap.put("r_palm_finger_middle_joint", null);

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
		//      for (Iterator iterator = joints.iterator(); iterator.hasNext();) {
		//		OneDoFJoint oneDoFJoint = (OneDoFJoint) iterator.next();
		//		System.out.println(oneDoFJoint.getName());
		//		
		//	}
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
