package us.ihmc.darpaRoboticsChallenge.posePlayback;

import java.io.IOException;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.environment.VRCTask;
import us.ihmc.darpaRoboticsChallenge.environment.VRCTaskName;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class PosePlaybackSCSBridge
{
   private static final String ipAddress = DRCConfigParameters.CLOUD_MINION5_IP;

   private final PosePlaybackAllJointsController posePlaybackController;
   private final PosePlaybackSender posePlaybackSender;
   private PosePlaybackRobotPoseSequence posePlaybackRobotPoseSequence;

   public PosePlaybackSCSBridge() throws IOException
   {
      YoVariableRegistry registry = new YoVariableRegistry("PlaybackPoseSCSBridge");
      posePlaybackController = new PosePlaybackAllJointsController(registry);
      posePlaybackSender = new PosePlaybackSender(posePlaybackController, ipAddress);
      posePlaybackRobotPoseSequence = new PosePlaybackRobotPoseSequence();

      VRCTask vrcTask = new VRCTask(VRCTaskName.ONLY_VEHICLE);
      SDFRobot sdfRobot = vrcTask.getRobot();

      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);
      scs.addYoVariableRegistry(registry);


      DRCRobotMidiSliderBoardPositionManipulation sliderBoard = new DRCRobotMidiSliderBoardPositionManipulation(scs);

      CaptureSnapshotListener captureSnapshotListener = new CaptureSnapshotListener(sdfRobot);
      sliderBoard.addIsCaptureSnapshotListener(captureSnapshotListener);
      
      SaveSequenceListener saveSequenceListener = new SaveSequenceListener();
      sliderBoard.addIsSaveSequenceRequestedListener(saveSequenceListener);

      scs.startOnAThread();

      try
      {
         posePlaybackSender.connect();
         posePlaybackSender.waitUntilConnected();
      }
      catch (Exception e)
      {
         System.err.println("Didn't connect to posePlaybackSender!");
      }

//    while (true)
//    {
//       PosePlaybackRobotPose pose = new PosePlaybackRobotPose(sdfRobot);
//       posePlaybackController.setPlaybackPose(pose);
//
//       posePlaybackSender.writeData();
//
//       ThreadTools.sleep(1000);
//    }
   }

   private class CaptureSnapshotListener implements VariableChangedListener
   {
      private final SDFRobot sdfRobot;

      public CaptureSnapshotListener(SDFRobot sdfRobot)
      {
         this.sdfRobot = sdfRobot;
      }

      public void variableChanged(YoVariable yoVariable)
      {
         BooleanYoVariable captureSnapshot = (BooleanYoVariable) yoVariable;
         if (captureSnapshot.getBooleanValue())
         {
            PosePlaybackRobotPose pose = new PosePlaybackRobotPose(sdfRobot);
            System.out.println(pose);
            posePlaybackRobotPoseSequence.addPose(pose);
            System.out.println("adding pose to sequence list");

            posePlaybackController.setPlaybackPose(pose);

            try
            {
               if (posePlaybackSender.isConnected())
                  posePlaybackSender.writeData();

            }
            catch (IOException e)
            {
            }
         }
      }
   }
   
   private class SaveSequenceListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable yoVariable)
      {         
         System.out.println("saving file");
         posePlaybackRobotPoseSequence.promptWriteToFile();
      }
   }


   public static void main(String[] args) throws IOException
   {
      new PosePlaybackSCSBridge();
   }

}
