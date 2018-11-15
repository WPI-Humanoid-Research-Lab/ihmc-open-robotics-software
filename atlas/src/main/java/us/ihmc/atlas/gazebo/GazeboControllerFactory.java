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
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.atlas.ros.RosAtlasAuxiliaryRobotDataPublisher;
import us.ihmc.avatar.DRCEstimatorThread;
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
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
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

	private static final boolean INTEGRATE_ACCELERATIONS_AND_CONTROL_VELOCITIES = true;

	private static final WalkingProvider walkingProvider = WalkingProvider.DATA_PRODUCER;

	//	private static final boolean USE_GUI = true;

	private final AtlasSensorInformation sensorInformation;
	private static String defaultRosNameSpace = "/ihmc_ros/atlas";

	private static String nodeName = "/robot_data";
	private static final double gravity = -9.81;
	private final PacketCommunicator controllerPacketCommunicator;
	
	public GazeboControllerFactory(DRCRobotModel robotModel, String nameSpace, String robotName, String tfPrefix) throws URISyntaxException, IOException
	{
		/*
		 * Create registries
		 */
		sensorInformation =(AtlasSensorInformation) robotModel.getSensorInformation();


		/*
		 * Create network servers/clients
		 */
		controllerPacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());
		YoVariableServer yoVariableServer = new YoVariableServer(getClass(), new PeriodicNonRealtimeThreadSchedulerFactory(), robotModel.getLogModelProvider(),
				robotModel.getLogSettings(), robotModel.getEstimatorDT());
		RobotVisualizer robotVisualizer = yoVariableServer;
		HumanoidGlobalDataProducer dataProducer = new HumanoidGlobalDataProducer(controllerPacketCommunicator);
//		      BehaviorStatusProducer atlasBehaviorStatusProducer = new BehaviorStatusProducer(dataProducer);

		/*
		 * Create controllers
		 */
		HighLevelHumanoidControllerFactory controllerFactory = createDRCControllerFactory(sensorInformation, robotModel, controllerPacketCommunicator);

		/*
		 * Create sensors
		 */
		StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();

		GazeboSensorReaderFactory sensorReaderFactory = new GazeboSensorReaderFactory(sensorInformation, stateEstimatorParameters);

		/*
		 * Create output writer
		 */
		GazeboOutputWriter gazeboOutputWriter = new GazeboOutputWriter(robotModel);

		PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber = new PelvisPoseCorrectionCommunicator(dataProducer);
		dataProducer.attachListener(StampedPosePacket.class, externalPelvisPoseSubscriber);

		/*
		 * Build controller
		 */
		ThreadDataSynchronizer threadDataSynchronizer = new ThreadDataSynchronizer(robotModel);
		DRCEstimatorThread estimatorThread = new DRCEstimatorThread(robotModel.getSensorInformation(), robotModel.getContactPointParameters(),robotModel,
				robotModel.getStateEstimatorParameters(), sensorReaderFactory, threadDataSynchronizer, new PeriodicNonRealtimeThreadScheduler( "DRCPoseCommunicator"), dataProducer,null, robotVisualizer, gravity);
		estimatorThread.setExternalPelvisCorrectorSubscriber(externalPelvisPoseSubscriber);
		DRCControllerThread controllerThread = new DRCControllerThread(robotModel, robotModel.getSensorInformation(), controllerFactory, threadDataSynchronizer,
				gazeboOutputWriter, dataProducer, robotVisualizer, gravity, robotModel.getEstimatorDT());


		/*
		 * Setup threads
		 */
		GazeboThreadedRobotController robotController = new GazeboThreadedRobotController();
		int estimatorTicksPerSimulationTick = (int) Math.round(robotModel.getEstimatorDT() / robotModel.getEstimatorDT());
		int controllerTicksPerSimulationTick = (int) Math.round(robotModel.getControllerDT() / robotModel.getEstimatorDT());

		robotController.addController(estimatorThread, estimatorTicksPerSimulationTick, false);
		robotController.addController(controllerThread, controllerTicksPerSimulationTick, true);

		try
		{
			controllerPacketCommunicator.connect();
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}


		DRCNetworkModuleParameters networkModuleParams = new DRCNetworkModuleParameters();
		URI rosURI = NetworkParameters.getROSURI();
		networkModuleParams.setRosUri(rosURI);
		networkModuleParams.enableRosModule(true);
		networkModuleParams.enableROSAPICommunicator(true);
		networkModuleParams.enableLocalControllerCommunicator(true);
		networkModuleParams.enableControllerCommunicator(true);
		networkModuleParams.enableBehaviorModule(true);
		networkModuleParams.enableBehaviorVisualizer(true);
		

		CloseableAndDisposableRegistry closeableAndDisposableRegistry = new CloseableAndDisposableRegistry();

		DRCNetworkProcessor networkProcessor = new DRCNetworkProcessor(robotModel, networkModuleParams);

		PacketCommunicator rosCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ROS_API_COMMUNICATOR, new IHMCCommunicationKryoNetClassList());
		
		new UiPacketToRosMsgRedirector(robotModel, rosURI, rosCommunicator, networkProcessor.getPacketRouter(), defaultRosNameSpace);
		
        
		RosMainNode rosMainNode = new RosMainNode(rosURI, nameSpace + nodeName);
		RosAtlasAuxiliaryRobotDataPublisher auxiliaryRobotDataPublisher = new RosAtlasAuxiliaryRobotDataPublisher(rosMainNode, nameSpace);
		rosMainNode.execute();

		rosCommunicator.attachListener(RobotConfigurationData.class, auxiliaryRobotDataPublisher);

		new ThePeoplesGloriousNetworkProcessor(rosURI, rosCommunicator, robotModel, nameSpace, tfPrefix);
		
		yoVariableServer.start();
		
		gazeboOutputWriter.connect();
		
		sensorReaderFactory.getSensorReader().connect();
		
		Thread simulationThread = new Thread(robotController);
		simulationThread.start();


		try
		{
			simulationThread.join();
		}
		catch (InterruptedException e)
		{
			e.printStackTrace();
		}

	}

	//   private MomentumBasedControllerFactory createDRCControllerFactory(DRCRobotModel robotModel, HumanoidGlobalDataProducer dataProducer)
	//   {
	//      ContactableBodiesFactory contactableBodiesFactory = robotModel.getContactPointParameters().getContactableBodiesFactory();
	//
	//      ArmControllerParameters armControllerParameters = robotModel.getArmControllerParameters();
	//      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
	//      final HighLevelState initialBehavior;
	//      CapturePointPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();
	//      initialBehavior = HighLevelState.WALKING; // HERE!!
	//
	//      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForSlowWalkingOnRobot(walkingControllerParameters);
	//      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
	//      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
	//      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
	//
	//      MomentumBasedControllerFactory controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory, feetForceSensorNames,
	//            feetContactSensorNames, wristForceSensorNames, walkingControllerParameters, armControllerParameters, capturePointPlannerParameters, initialBehavior);
	//
	////      controllerFactory.addHighLevelBehaviorFactory(new JointPositionControllerFactory(true));
	//
	//      if (USE_GUI)
	//      {
	//         VariousWalkingProviderFactory variousWalkingProviderFactory = new DataProducerVariousWalkingProviderFactory(dataProducer, footstepTimingParameters, new PeriodicNonRealtimeThreadScheduler("CapturabilityBasedStatusProducer"));
	//         controllerFactory.setVariousWalkingProviderFactory(variousWalkingProviderFactory);
	//
	//      }
	//      else
	//      {
	//         VariousWalkingProviderFactory variousWalkingProviderFactory = new ComponentBasedVariousWalkingProviderFactory(true, null, robotModel.getControllerDT());
	//         controllerFactory.setVariousWalkingProviderFactory(variousWalkingProviderFactory);
	//      }
	//
	//      return controllerFactory;
	//   }

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
		controllerFactory.setInitialState(highLevelControllerParameters.getDefaultInitialControllerState());

		if (walkingProvider == WalkingProvider.VELOCITY_HEADING_COMPONENT)
		{
			boolean useHeadingAndVelocityScript = false;
			controllerFactory.createComponentBasedFootstepDataMessageGenerator(useHeadingAndVelocityScript, null);
		}

		return controllerFactory;
	}

	public static void main(String args[]) throws JSAPException, IOException
	{
		JSAP jsap = new JSAP();

		FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);

		robotModel.setHelp("Robot models: " + Arrays.toString(AtlasRobotModelFactory.getAvailableRobotModels()));
		jsap.registerParameter(robotModel);

		JSAPResult config = jsap.parse(args);

		if (config.success())
		{
			try
			{

				AtlasRobotModel model = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), RobotTarget.GAZEBO, true);


				new GazeboControllerFactory(model, "ihmc_ros", "atlas", null);
			}
			catch (IllegalArgumentException e)
			{
				System.err.println("Incorrect robot model " + config.getString("robotModel"));
				System.out.println(jsap.getHelp());

			} catch (URISyntaxException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		else
		{
			for (java.util.Iterator<?> errs = config.getErrorMessageIterator(); errs.hasNext();)
			{
				System.err.println("Error: " + errs.next());
			}
			System.out.println(jsap.getHelp());
		}

	}
}
