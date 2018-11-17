package us.ihmc.atlas.gazebo;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotModelFactory;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.SimulatedLowLevelOutputWriter;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCOutputProcessor;

public class GazeboAtlasRobotModel extends AtlasRobotModel {
	
	private final GazeboOutputWriter gazeboOutputProcessor;
	private final JointDesiredOutputWriter jointDesiredOutputWriter;
	private final HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot;
	
	public GazeboAtlasRobotModel(AtlasRobotVersion atlasVersion) {
		super(atlasVersion, RobotTarget.GAZEBO, true);
		
		gazeboOutputProcessor = new GazeboOutputWriter(this);
		
		humanoidFloatingRootJointRobot = createHumanoidFloatingRootJointRobot(false);
		
		jointDesiredOutputWriter = new SimulatedLowLevelOutputWriter(humanoidFloatingRootJointRobot, true);
		
	}
	
	public HumanoidFloatingRootJointRobot getHumanoidFloatingRootJointRobot() {
		return humanoidFloatingRootJointRobot;
	}

	@Override
	public  DRCOutputProcessor getCustomSimulationOutputProcessor(HumanoidFloatingRootJointRobot floatingRootJointRobot) {
		System.out.println("Providing gazebo output processor");
		return gazeboOutputProcessor;
	}
	
	@Override
	public JointDesiredOutputWriter getCustomSimulationOutputWriter(HumanoidFloatingRootJointRobot floatingRootJointRobot) {
		System.out.println("Providing gazebo output writer");
		return jointDesiredOutputWriter;
	}
	
	public void connectOutputProcessor() {
		gazeboOutputProcessor.connect();
	}
}
