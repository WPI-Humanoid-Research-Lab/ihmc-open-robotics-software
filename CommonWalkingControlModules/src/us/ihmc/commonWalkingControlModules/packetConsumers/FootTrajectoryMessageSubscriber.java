package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootTrajectoryMessageSubscriber implements PacketConsumer<FootTrajectoryMessage>
{
   private final HumanoidGlobalDataProducer globalDataProducer;

   private final SideDependentList<AtomicReference<FootTrajectoryMessage>> latestMessageReferences = new SideDependentList<>(
         new AtomicReference<FootTrajectoryMessage>(null), new AtomicReference<FootTrajectoryMessage>(null));

   public FootTrajectoryMessageSubscriber(HumanoidGlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;

      globalDataProducer.attachListener(FootTrajectoryMessage.class, this);
   }

   public boolean isNewTrajectoryMessageAvailable()
   {
      for (RobotSide robotSide : RobotSide.values)
         if (latestMessageReferences.get(robotSide).get() != null)
            return true;
      return false;
   }

   public boolean isNewTrajectoryMessageAvailable(RobotSide robotSide)
   {
      return latestMessageReferences.get(robotSide).get() != null;
   }

   public FootTrajectoryMessage pollMessage(RobotSide robotSide)
   {
      return latestMessageReferences.get(robotSide).getAndSet(null);
   }

   public void clearMessagesInQueue()
   {
      for (RobotSide robotSide : RobotSide.values)
         latestMessageReferences.get(robotSide).set(null);
   }

   @Override
   public void receivedPacket(FootTrajectoryMessage footTrajectoryMessage)
   {
      String errorMessage = PacketValidityChecker.validateFootTrajectoryMessage(footTrajectoryMessage);
      if (errorMessage != null)
      {
         if (globalDataProducer != null)
            globalDataProducer.notifyInvalidPacketReceived(footTrajectoryMessage.getClass(), errorMessage);
         return;
      }

      RobotSide robotSide = footTrajectoryMessage.getRobotSide();
      latestMessageReferences.get(robotSide).set(footTrajectoryMessage);
   }
}
