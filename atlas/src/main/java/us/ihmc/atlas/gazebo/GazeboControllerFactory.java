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

	private static final boolean INTEGRATE_ACCELERATIONS_AND_CONTROL_VELOCITIES = true;

	private static final WalkingProvider walkingProvider = WalkingProvider.DATA_PRODUCER;

	//	private static final boolean USE_GUI = true;

	private final AtlasSensorInformation sensorInformation;
	private static String defaultRosNameSpace = "/ihmc_ros/atlas";

	private static String nodeName = "/robot_data";
	private static final double gravity = -9.81;
	private final PacketCommunicator controllerPacketCommunicator;
	private final HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot ;
	
	public GazeboControllerFactory(GazeboAtlasRobotModel robotModel, String nameSpace, String robotName, String tfPrefix) throws URISyntaxException, IOException
	{
		/*
		 * Create registries
		 */
		sensorInformation =(AtlasSensorInformation) robotModel.getSensorInformation();
		humanoidFloatingRootJointRobot = robotModel.getHumanoidFloatingRootJointRobot();

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
		DRCOutputProcessor outputProcessor = robotModel.getCustomSimulationOutputProcessor(humanoidFloatingRootJointRobot);
		JointDesiredOutputWriter outputWriter = robotModel.getCustomSimulationOutputWriter(humanoidFloatingRootJointRobot);

		PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber = new PelvisPoseCorrectionCommunicator(dataProducer);
		dataProducer.attachListener(StampedPosePacket.class, externalPelvisPoseSubscriber);

		/*
		 * Build controller
		 */
		ThreadDataSynchronizer threadDataSynchronizer = new ThreadDataSynchronizer(robotModel);
		
		DRCEstimatorThread estimatorThread = new DRCEstimatorThread(sensorInformation, robotModel.getContactPointParameters(),robotModel,
				robotModel.getStateEstimatorParameters(), sensorReaderFactory, threadDataSynchronizer, new PeriodicNonRealtimeThreadScheduler( "DRCSimGazeboYoVariableServer"), 
				dataProducer, outputWriter , robotVisualizer, gravity);
		estimatorThread.setExternalPelvisCorrectorSubscriber(externalPelvisPoseSubscriber);
		
		try
		{
			humanoidFloatingRootJointRobot.update();
			humanoidFloatingRootJointRobot.doDynamicsButDoNotIntegrate();
			humanoidFloatingRootJointRobot.update();
		}
		catch (UnreasonableAccelerationException e)
		{
			throw new RuntimeException("UnreasonableAccelerationException");
		}

		Point3D initialCoMPosition = new Point3D();
		humanoidFloatingRootJointRobot.computeCenterOfMass(initialCoMPosition);

		Quaternion initialEstimationLinkOrientation = new Quaternion();
		humanoidFloatingRootJointRobot.getRootJoint().getJointTransform3D().getRotation(initialEstimationLinkOrientation);

		estimatorThread.initializeEstimatorToActual(initialCoMPosition, initialEstimationLinkOrientation);

		DRCControllerThread controllerThread = new DRCControllerThread(robotModel, sensorInformation, controllerFactory, threadDataSynchronizer,
				outputProcessor, dataProducer, robotVisualizer, gravity, robotModel.getEstimatorDT());

		/*
		 * Setup threads
		 */
		GazeboThreadedRobotController robotController = new GazeboThreadedRobotController(humanoidFloatingRootJointRobot);
		int estimatorTicksPerSimulationTick = (int) Math.round(robotModel.getEstimatorDT() / robotModel.getEstimatorDT());
		int controllerTicksPerSimulationTick = (int) Math.round(robotModel.getControllerDT() / robotModel.getEstimatorDT());

		robotController.addController(controllerThread, controllerTicksPerSimulationTick, true);
		robotController.addController(estimatorThread, estimatorTicksPerSimulationTick, false);
		
		humanoidFloatingRootJointRobot.setController(robotController);
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

		DRCNetworkProcessor networkProcessor = new DRCNetworkProcessor(robotModel, networkModuleParams);

		PacketCommunicator rosCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ROS_API_COMMUNICATOR, new IHMCCommunicationKryoNetClassList());

		new UiPacketToRosMsgRedirector(robotModel, rosURI, rosCommunicator, networkProcessor.getPacketRouter(), defaultRosNameSpace);


		RosMainNode rosMainNode = new RosMainNode(rosURI, nameSpace + nodeName);
		RosAtlasAuxiliaryRobotDataPublisher auxiliaryRobotDataPublisher = new RosAtlasAuxiliaryRobotDataPublisher(rosMainNode, nameSpace);
		rosMainNode.execute();

		rosCommunicator.attachListener(RobotConfigurationData.class, auxiliaryRobotDataPublisher);

		new ThePeoplesGloriousNetworkProcessor(rosURI, rosCommunicator, robotModel, nameSpace, tfPrefix);

		yoVariableServer.start();

		robotModel.connectOutputProcessor();

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
		controllerFactory.setInitialState(HighLevelControllerName.WALKING);

		if (walkingProvider == WalkingProvider.VELOCITY_HEADING_COMPONENT)
		{
			boolean useHeadingAndVelocityScript = false;
			controllerFactory.createComponentBasedFootstepDataMessageGenerator(useHeadingAndVelocityScript, null);
		}

		return controllerFactory;
	}

	public static void main(String args[]) throws JSAPException, IOException
	{
		try
		{
			GazeboAtlasRobotModel model = new GazeboAtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);
			new GazeboControllerFactory(model, "ihmc_ros", "atlas", null);
		}
		catch (IllegalArgumentException e)
		{
			e.printStackTrace();

		} catch (URISyntaxException e) 
		{
			e.printStackTrace();
		}


	}
}
