package us.ihmc.atlas.gazebo;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Arrays;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotModelFactory;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.atlas.ros.RosAtlasAuxiliaryRobotDataPublisher;
import us.ihmc.avatar.DRCEstimatorThread;
import us.ihmc.avatar.SimulatedLowLevelOutputWriter;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WalkingProvider;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicator;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.wholeBodyController.DRCOutputProcessorWithStateChangeSmoother;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizer;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.networkProcessor.DRCNetworkProcessor;
import us.ihmc.avatar.networkProcessor.modules.uiConnector.UiPacketToRosMsgRedirector;
import us.ihmc.avatar.networkProcessor.time.SimulationRosClockPPSTimestampOffsetProvider;
import us.ihmc.avatar.rosAPI.ThePeoplesGloriousNetworkProcessor;

public class GazeboControllerFactory
{

	private static final WalkingProvider walkingProvider = WalkingProvider.DATA_PRODUCER;

	private static String defaultRosNameSpace = "/ihmc_ros/atlas";
	private static String nodeName = "/robot_data";
	private static final double gravity = -9.81;
	private final PacketCommunicator controllerPacketCommunicationServer;
	private final PacketCommunicator rosCommunicationServer;
	private final YoVariableServer yoVariableServer;
	private final Thread simulationThread;
	private GazeboSensorReaderFactory sensorReaderFactory;
	private final HumanoidGlobalDataProducer dataProducer;

	public Thread getSimulationThread() {
		return simulationThread;
	}



	public GazeboControllerFactory(GazeboAtlasRobotModel robotModel, String nameSpace, String robotName, String tfPrefix) throws IOException
	{
		HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot = robotModel.getHumanoidFloatingRootJointRobot();

		controllerPacketCommunicationServer = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, 
				new IHMCCommunicationKryoNetClassList());
		rosCommunicationServer = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ROS_API_COMMUNICATOR, new IHMCCommunicationKryoNetClassList());

		yoVariableServer = new YoVariableServer(getClass(), new PeriodicNonRealtimeThreadSchedulerFactory(), 
				robotModel.getLogModelProvider(),robotModel.getLogSettings(), robotModel.getEstimatorDT());

		dataProducer = new HumanoidGlobalDataProducer(controllerPacketCommunicationServer);

		/*
		 * Setup threads
		 */
		ThreadDataSynchronizer threadDataSynchronizer = new ThreadDataSynchronizer(robotModel);
		DRCEstimatorThread estimatorThread = createEstimatorThread(robotModel, threadDataSynchronizer);
		DRCControllerThread controllerThread = createControllerThread(robotModel, threadDataSynchronizer);

		GazeboThreadedRobotController robotController = new GazeboThreadedRobotController(humanoidFloatingRootJointRobot);
		int estimatorTicksPerSimulationTick = (int) Math.round(robotModel.getEstimatorDT() / robotModel.getEstimatorDT());
		int controllerTicksPerSimulationTick = (int) Math.round(robotModel.getControllerDT() / robotModel.getEstimatorDT());

		robotController.addController(controllerThread, controllerTicksPerSimulationTick, true);
		robotController.addController(estimatorThread, estimatorTicksPerSimulationTick, false);

		humanoidFloatingRootJointRobot.setController(robotController);

		createNetworkProcessor(robotModel, tfPrefix);

		simulationThread = new Thread(robotController);

		connectAll(robotModel);

	}

	private DRCEstimatorThread createEstimatorThread(GazeboAtlasRobotModel robotModel, ThreadDataSynchronizer threadDataSynchronizer)
	{
		AtlasSensorInformation sensorInformation =(AtlasSensorInformation) robotModel.getSensorInformation();
		HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot = robotModel.getHumanoidFloatingRootJointRobot();

		JointDesiredOutputWriter outputWriter = robotModel.getCustomSimulationOutputWriter(humanoidFloatingRootJointRobot);

		StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();
		sensorReaderFactory = new GazeboSensorReaderFactory(sensorInformation, stateEstimatorParameters);

		PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber = new PelvisPoseCorrectionCommunicator(dataProducer);
		dataProducer.attachListener(StampedPosePacket.class, externalPelvisPoseSubscriber);

		DRCEstimatorThread estimatorThread = new DRCEstimatorThread(sensorInformation, robotModel.getContactPointParameters(),robotModel,
				robotModel.getStateEstimatorParameters(), sensorReaderFactory, threadDataSynchronizer, new PeriodicNonRealtimeThreadScheduler( "DRCSimGazeboYoVariableServer"), 
				dataProducer, outputWriter , yoVariableServer, gravity);
		estimatorThread.setExternalPelvisCorrectorSubscriber(externalPelvisPoseSubscriber);

		return estimatorThread;

	}

	private DRCControllerThread createControllerThread(GazeboAtlasRobotModel robotModel, ThreadDataSynchronizer threadDataSynchronizer)
	{
		AtlasSensorInformation sensorInformation =(AtlasSensorInformation) robotModel.getSensorInformation();
		HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot = robotModel.getHumanoidFloatingRootJointRobot();

		DRCOutputProcessor outputProcessor = robotModel.getCustomSimulationOutputProcessor(humanoidFloatingRootJointRobot);

		HighLevelHumanoidControllerFactory controllerFactory = createDRCControllerFactory(sensorInformation, robotModel, controllerPacketCommunicationServer);

		DRCControllerThread controllerThread = new DRCControllerThread(robotModel, robotModel.getSensorInformation(), controllerFactory, 
				threadDataSynchronizer, outputProcessor, 
				dataProducer, yoVariableServer, gravity, robotModel.getEstimatorDT());

		return controllerThread;

	}

	private void connectAll(GazeboAtlasRobotModel robotModel)  throws IOException
	{
		controllerPacketCommunicationServer.connect();
		yoVariableServer.start();
		robotModel.connectOutputProcessor();
		sensorReaderFactory.connectSensorReader();
		simulationThread.start();
	}

	public void disconnect() {
		controllerPacketCommunicationServer.disconnect();
		rosCommunicationServer.disconnect();
		yoVariableServer.close();

	}
	private void createNetworkProcessor(AtlasRobotModel robotModel, String tfPrefix) throws IOException
	{
		DRCNetworkModuleParameters networkModuleParams = new DRCNetworkModuleParameters();
		URI rosURI = NetworkParameters.getROSURI();
		networkModuleParams.setRosUri(rosURI);
		networkModuleParams.enableRosModule(true);
		networkModuleParams.enableBehaviorModule(true);
		networkModuleParams.enableBehaviorVisualizer(true);
		networkModuleParams.enableROSAPICommunicator(true);
		networkModuleParams.enableLocalControllerCommunicator(true);
		networkModuleParams.enableControllerCommunicator(true);

		DRCNetworkProcessor networkProcessor = new DRCNetworkProcessor(robotModel, networkModuleParams);

		new UiPacketToRosMsgRedirector(robotModel, rosURI, rosCommunicationServer, networkProcessor.getPacketRouter(), defaultRosNameSpace);

		RosMainNode rosMainNode = new RosMainNode(rosURI, defaultRosNameSpace + nodeName);
		RosAtlasAuxiliaryRobotDataPublisher auxiliaryRobotDataPublisher = new RosAtlasAuxiliaryRobotDataPublisher(rosMainNode, defaultRosNameSpace);
		rosMainNode.execute();

		rosCommunicationServer.attachListener(RobotConfigurationData.class, auxiliaryRobotDataPublisher);

		/*
		 * ThePeoplesGloriousNetworkProcessor connects rosCommunicator. So it is not required to connect it outside.
		 * namespace parameter in this constructor takes the namespace+robotName
		 */
		new ThePeoplesGloriousNetworkProcessor(rosURI, rosCommunicationServer, robotModel, defaultRosNameSpace, tfPrefix);

	}


	private static HighLevelHumanoidControllerFactory createDRCControllerFactory(AtlasSensorInformation sensorInformation, DRCRobotModel robotModel, PacketCommunicator packetCommunicator)
	{
		ContactableBodiesFactory contactableBodiesFactory = robotModel.getContactPointParameters().getContactableBodiesFactory();

		HighLevelControllerParameters highLevelControllerParameters = robotModel.getHighLevelControllerParameters();
		WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
		ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();
		SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
		SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
		SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();

		HighLevelHumanoidControllerFactory controllerFactory = new HighLevelHumanoidControllerFactory(contactableBodiesFactory, feetForceSensorNames,
				feetContactSensorNames, wristForceSensorNames, highLevelControllerParameters, walkingControllerParameters, capturePointPlannerParameters);
		controllerFactory.createControllerNetworkSubscriber(new PeriodicNonRealtimeThreadScheduler("controllerNetworkSubscriber"), packetCommunicator);

		controllerFactory.useDefaultDoNothingControlState();
		controllerFactory.useDefaultWalkingControlState();

		controllerFactory.addRequestableTransition(HighLevelControllerName.DO_NOTHING_BEHAVIOR, HighLevelControllerName.WALKING);
		controllerFactory.addRequestableTransition(HighLevelControllerName.WALKING, HighLevelControllerName.DO_NOTHING_BEHAVIOR);
		controllerFactory.setInitialState(HighLevelControllerName.WALKING);

		if (walkingProvider == WalkingProvider.VELOCITY_HEADING_COMPONENT)
		{
			boolean useHeadingAndVelocityScript = false;
			controllerFactory.createComponentBasedFootstepDataMessageGenerator(useHeadingAndVelocityScript, null);
		}

		return controllerFactory;
	}

	public static void main(String args[]) throws IOException, JSAPException
	{
		JSAP jsap = new JSAP();

		FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);
		robotModel.setHelp("Robot models: " + Arrays.toString(AtlasRobotModelFactory.getAvailableRobotModels()));
		jsap.registerParameter(robotModel);
		JSAPResult config = jsap.parse(args);
		AtlasRobotVersion atlasModel;
		if(!config.success())
		{
			atlasModel = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
		}
		atlasModel = AtlasRobotVersion.valueOf(config.getString("robotModel").toUpperCase().trim());
		GazeboAtlasRobotModel model = new GazeboAtlasRobotModel(atlasModel);
		GazeboControllerFactory controller = new GazeboControllerFactory(model, "ihmc_ros", "atlas", null);
		try
		{
			controller.getSimulationThread().join();
		}
		catch (InterruptedException e)
		{
			e.printStackTrace();
		}
		finally {
			controller.disconnect();
		}


	}
}
