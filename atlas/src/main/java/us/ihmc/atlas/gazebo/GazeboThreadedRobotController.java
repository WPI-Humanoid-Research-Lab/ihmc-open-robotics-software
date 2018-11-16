package us.ihmc.atlas.gazebo;

import us.ihmc.simulationConstructionSetTools.robotController.AbstractThreadedRobotController;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControllerExecutor;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;

public class GazeboThreadedRobotController extends AbstractThreadedRobotController implements Runnable
{

   private volatile boolean running = true;
   private HumanoidFloatingRootJointRobot robotModel;
   public GazeboThreadedRobotController(HumanoidFloatingRootJointRobot simulatedRobotModel)
   {
      super(GazeboThreadedRobotController.class.getSimpleName());
      robotModel = simulatedRobotModel;
   }

   @Override
   public void addController(MultiThreadedRobotControlElement controller, int executionsPerControlTick, boolean skipFirstControlCycle)
   {
      controllers.add(new MultiThreadedRobotControllerExecutor(robotModel, controller, executionsPerControlTick, skipFirstControlCycle, registry));
   }

   @Override
   public void run()
   {
      while(running)
      {
         doControl();
      }
   }
   
   public void stop()
   {
      running = false;
   }

}
