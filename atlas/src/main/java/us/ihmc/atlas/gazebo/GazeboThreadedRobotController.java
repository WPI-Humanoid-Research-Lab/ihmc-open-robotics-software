package us.ihmc.atlas.gazebo;

import us.ihmc.simulationConstructionSetTools.robotController.AbstractThreadedRobotController;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControllerExecutor;

public class GazeboThreadedRobotController extends AbstractThreadedRobotController implements Runnable
{

   private volatile boolean running = true;
   
   public GazeboThreadedRobotController()
   {
      super(GazeboThreadedRobotController.class.getSimpleName());
   }

   @Override
   public void addController(MultiThreadedRobotControlElement controller, int executionsPerControlTick, boolean skipFirstControlCycle)
   {
      controllers.add(new MultiThreadedRobotControllerExecutor(null, controller, executionsPerControlTick, skipFirstControlCycle, registry));
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
