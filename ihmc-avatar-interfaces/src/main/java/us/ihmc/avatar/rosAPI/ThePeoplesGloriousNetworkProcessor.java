package us.ihmc.avatar.rosAPI;

import java.io.IOException;
import java.net.URI;
import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import ihmc_msgs.HandDesiredConfigurationRosMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.ros.IHMCPacketToMsgPublisher;
import us.ihmc.avatar.ros.IHMCROSTranslationRuntimeTools;
import us.ihmc.avatar.ros.PeriodicRosHighLevelStatePublisher;
import us.ihmc.avatar.ros.RosCapturabilityBasedStatusPublisher;
import us.ihmc.avatar.ros.RosRobotConfigurationDataPublisher;
import us.ihmc.avatar.ros.RosSCSCameraPublisher;
import us.ihmc.avatar.ros.RosSCSLidarPublisher;
import us.ihmc.avatar.ros.RosTfPublisher;
import us.ihmc.avatar.ros.subscriber.IHMCMsgToPacketSubscriber;
import us.ihmc.avatar.ros.subscriber.RequestControllerStopSubscriber;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.ControllerCrashNotificationPacket;
import us.ihmc.communication.packets.InvalidPacketNotificationPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateChangeStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.ihmcPerception.IHMCProntoRosLocalizationUpdateSubscriber;
import us.ihmc.ihmcPerception.RosLocalizationPoseCorrectionSubscriber;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.msgToPacket.converter.GenericROSTranslationTools;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.RosTopicSubscriberInterface;
import us.ihmc.wholeBodyController.DRCRobotJointMap;


import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;



public class ThePeoplesGloriousNetworkProcessor
{
   private static final String nodeName = "/controller";

   private final DRCROSPPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final RosMainNode rosMainNode;
   private final DRCRobotModel robotModel;
   private final PacketCommunicator controllerCommunicationBridge;
   private final ObjectCommunicator scsSensorCommunicationBridge;

   private final PacketCommunicator LeftHandCommunicator;
//   private final PacketCommunicator RightHandCommunicator;
   
   private final ArrayList<AbstractRosTopicSubscriber<?>> subscribers;
   private final ArrayList<RosTopicPublisher<?>> publishers;

   private final NodeConfiguration nodeConfiguration;
   private final MessageFactory messageFactory;
   private final FullHumanoidRobotModel fullRobotModel;

   public ThePeoplesGloriousNetworkProcessor(URI rosUri, PacketCommunicator controllerCommunicationBridge, DRCRobotModel robotModel, String namespace,
         String tfPrefix, String... additionalMessagePackages) throws IOException
   {
      this(rosUri, controllerCommunicationBridge, null, robotModel.getPPSTimestampOffsetProvider(), robotModel, namespace, tfPrefix, Collections.<Class>emptySet(), additionalMessagePackages);
   }

   public ThePeoplesGloriousNetworkProcessor(URI rosUri, PacketCommunicator controllerCommunicationBridge, DRCRobotModel robotModel, String namespace,
         String tfPrefix, Collection<Class> additionalPacketTypes, String... additionalMessagePackages) throws IOException
   {
      this(rosUri, controllerCommunicationBridge, null, robotModel.getPPSTimestampOffsetProvider(), robotModel, namespace, tfPrefix, additionalPacketTypes, additionalMessagePackages);
   }

   public ThePeoplesGloriousNetworkProcessor(URI rosUri, PacketCommunicator rosAPI_communicator, ObjectCommunicator sensorCommunicator,
         DRCROSPPSTimestampOffsetProvider ppsOffsetProvider, DRCRobotModel robotModel, String namespace, String tfPrefix, Collection<Class> additionalPacketTypes, String... additionalMessagePackages) throws IOException
   {
      this(rosUri, rosAPI_communicator, sensorCommunicator, robotModel.getPPSTimestampOffsetProvider(), robotModel, namespace, tfPrefix, additionalPacketTypes, null, null, additionalMessagePackages);
   }

   public ThePeoplesGloriousNetworkProcessor(URI rosUri, PacketCommunicator rosAPI_communicator, ObjectCommunicator sensorCommunicator,
         DRCROSPPSTimestampOffsetProvider ppsOffsetProvider, DRCRobotModel robotModel, String namespace, String tfPrefix, Collection<Class> additionalPacketTypes,
         List<Map.Entry<String, RosTopicSubscriberInterface<? extends Message>>> customSubscribers,
         List<Map.Entry<String, RosTopicPublisher<? extends Message>>> customPublishers, String... additionalMessagePackages) throws IOException
   {
      this.rosMainNode = new RosMainNode(rosUri, namespace + nodeName);
      this.robotModel = robotModel;
      this.controllerCommunicationBridge = rosAPI_communicator;
      this.scsSensorCommunicationBridge = sensorCommunicator;
      this.ppsTimestampOffsetProvider = ppsOffsetProvider;
      this.ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
      this.subscribers = new ArrayList<AbstractRosTopicSubscriber<?>>();
      this.publishers = new ArrayList<RosTopicPublisher<?>>();

      this.nodeConfiguration = NodeConfiguration.newPrivate();
      this.messageFactory = nodeConfiguration.getTopicMessageFactory();
      this.fullRobotModel = robotModel.createFullRobotModel();
      
      //---------------------------------------------------------------------------------------------------------------------//
      // create hand communicators
      this.LeftHandCommunicator = 
    		  PacketCommunicator.createIntraprocessPacketCommunicator(	NetworkPorts.CUSTOM_ROBOTIQ_HAND_COMMAND_DISPACHER_PORT,
    				  													new IHMCCommunicationKryoNetClassList());
//      this.RightHandCommunicator = 
//    		  PacketCommunicator.createIntraprocessPacketCommunicator(	NetworkPorts.RIGHT_HAND_MANAGER_PORT,
//    				  													new IHMCCommunicationKryoNetClassList());
      
      // connect hand communicators
      try 
      {    	  
    	  System.out.println("connecting CUSTOM_ROBOTIQ_HAND_COMMAND_DISPACHER_PORT hand ...");
    	  this.LeftHandCommunicator.connect();
    	  System.out.println("connected CUSTOM_ROBOTIQ_HAND_COMMAND_DISPACHER_PORT hand ...");
      }
      catch (IOException e1)
      {
    	  e1.printStackTrace();
      }
      
//      try 
//      {    	  
//    	  System.out.println("connecting right hand ...");
//    	  this.RightHandCommunicator.connect();
//      }
//      catch (IOException e1)
//      {
//    	  e1.printStackTrace();
//      }
      //---------------------------------------------------------------------------------------------------------------------//
      
      HumanoidRobotDataReceiver robotDataReceiver = new HumanoidRobotDataReceiver(fullRobotModel, null);
      rosAPI_communicator.attachListener(RobotConfigurationData.class, robotDataReceiver);
      rosAPI_communicator.attachListener(RobotConfigurationData.class, ppsOffsetProvider);
      rosAPI_communicator.attachListener(HighLevelStateChangeStatusMessage.class, new PeriodicRosHighLevelStatePublisher(rosMainNode, namespace));
      rosAPI_communicator.attachListener(CapturabilityBasedStatus.class, new RosCapturabilityBasedStatusPublisher(rosMainNode, namespace));

      setupInputs(namespace, robotDataReceiver, fullRobotModel, additionalMessagePackages);
      setupOutputs(namespace, tfPrefix, additionalMessagePackages);
      setupRosLocalization();
      setupErrorTopics();

      if (customSubscribers != null)
      {
         for (Map.Entry<String, RosTopicSubscriberInterface<? extends Message>> sub : customSubscribers)
         {
        	 System.out.println("Adding "+ sub.getKey().toString());
            this.subscribers.add((AbstractRosTopicSubscriber<?>) sub.getValue());
            rosMainNode.attachSubscriber(sub.getKey(), sub.getValue());
         }
      }

      if (customPublishers != null)
      {
         for (Map.Entry<String, RosTopicPublisher<? extends Message>> pub : customPublishers)
         {
            this.publishers.add(pub.getValue());
            rosMainNode.attachPublisher(pub.getKey(), pub.getValue());
         }
      }

      rosMainNode.execute();

      while (!rosMainNode.isStarted())
      {
         ThreadTools.sleep(100);
      }

      rosAPI_communicator.connect();

      System.out.println("IHMC ROS API node successfully started.");
   }



   @SuppressWarnings("unchecked")
   private void setupOutputs(String namespace, String tfPrefix, String... additionalPackages)
   {
      FullRobotModel fullRobotModel = robotModel.createFullRobotModel();
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      DRCRobotJointMap jointMap = robotModel.getJointMap();

      RosTfPublisher tfPublisher = new RosTfPublisher(rosMainNode, tfPrefix);

      RosRobotConfigurationDataPublisher robotConfigurationPublisher = new RosRobotConfigurationDataPublisher(robotModel, controllerCommunicationBridge,
            rosMainNode, ppsTimestampOffsetProvider, sensorInformation, jointMap, namespace, tfPublisher);
      if (scsSensorCommunicationBridge != null)
      {
         publishSimulatedCameraAndLidar(fullRobotModel, sensorInformation, robotConfigurationPublisher);
      }

      Set<Class<?>> outputTypes = GenericROSTranslationTools.getCoreOutputTopics();

      if(additionalPackages != null && additionalPackages.length > 0)
      {
         for (String additionalPackage : additionalPackages)
         {
            Set<Class<?>> additionalOutputs = GenericROSTranslationTools.getOutputTopicsForPackage(additionalPackage);
            outputTypes.addAll(additionalOutputs);
         }
      }

      for (Class outputType : outputTypes)
      {
         RosMessagePacket rosAnnotation = (RosMessagePacket) outputType.getAnnotation(RosMessagePacket.class);
         String rosMessageTypeString = IHMCROSTranslationRuntimeTools.getROSMessageTypeStringFromIHMCMessageClass(outputType);
         Message message = messageFactory.newFromType(rosMessageTypeString);

         IHMCPacketToMsgPublisher<Message, Packet> publisher = IHMCPacketToMsgPublisher.createIHMCPacketToMsgPublisher(message, false, controllerCommunicationBridge, outputType);
         publishers.add(publisher);
         rosMainNode.attachPublisher(namespace + rosAnnotation.topic(), publisher);
      }

//      PrintStreamToRosBridge printStreamBridge = new PrintStreamToRosBridge(rosMainNode, namespace);
//      printStreamBridge.start();
//      System.setErr(printStreamBridge);
   }

   private void publishSimulatedCameraAndLidar(FullRobotModel fullRobotModel, DRCRobotSensorInformation sensorInformation,
         RosRobotConfigurationDataPublisher robotConfigurationPublisher)
   {
      if (sensorInformation.getCameraParameters().length > 0)
      {
         new RosSCSCameraPublisher(scsSensorCommunicationBridge, rosMainNode, ppsTimestampOffsetProvider, sensorInformation.getCameraParameters());
      }

      DRCRobotLidarParameters[] lidarParameters = sensorInformation.getLidarParameters();
      if (lidarParameters.length > 0)
      {
         new RosSCSLidarPublisher(scsSensorCommunicationBridge, rosMainNode, ppsTimestampOffsetProvider, fullRobotModel, lidarParameters);

         DRCRobotLidarParameters primaryLidar = lidarParameters[0];
         robotConfigurationPublisher.setAdditionalJointStatePublishing(primaryLidar.getLidarSpindleJointTopic(), primaryLidar.getLidarSpindleJointName());
      }
   }

   private void setupInputs(String namespace, HumanoidRobotDataReceiver robotDataReceiver, FullHumanoidRobotModel fullRobotModel, String... additionalPackages)
   {
      Set<Class<?>> inputTypes = GenericROSTranslationTools.getCoreInputTopics();

      if(additionalPackages != null && additionalPackages.length > 0)
      {
         for (String additionalPackage : additionalPackages)
         {
            Set<Class<?>> additionalInputs = GenericROSTranslationTools.getInputTopicsForPackage(additionalPackage);
            inputTypes.addAll(additionalInputs);
         }
      }

      for (Class inputType : inputTypes)
      {
         RosMessagePacket rosAnnotation = (RosMessagePacket) inputType.getAnnotation(RosMessagePacket.class);
         String rosMessageTypeString = IHMCROSTranslationRuntimeTools.getROSMessageTypeStringFromIHMCMessageClass(inputType);
         Message message = messageFactory.newFromType(rosMessageTypeString);
         
         IHMCMsgToPacketSubscriber<Message> subscriber;
         
         /*
          * check if the message is of the type HandDesiredConfigurationRosMessage and bind appropriate Communicator
          * (i.e HandDesiredConfigurationRosMessage to LeftHandCommunicator or RightHandCommunicator 
          * and control messages to controllerCommunicationBridge).
          */
         if(message instanceof HandDesiredConfigurationRosMessage)
         {
        	 System.out.println("config type "+message.toString());

//        	 if (((HandDesiredConfigurationRosMessage) message).getRobotSide()==HandDesiredConfigurationRosMessage.LEFT) 
//        	 {        		 
//        		 subscriber = IHMCMsgToPacketSubscriber.createIHMCMsgToPacketSubscriber(message, LeftHandCommunicator, PacketDestination.LEFT_HAND.ordinal());
//        	 }
//        	 else 
//        	 {
//        		 subscriber = IHMCMsgToPacketSubscriber.createIHMCMsgToPacketSubscriber(message, RightHandCommunicator, PacketDestination.RIGHT_HAND.ordinal());	 
//        	 }
        	 

        	 IHMCMsgToPacketSubscriber<Message> subscriber_L = IHMCMsgToPacketSubscriber.createIHMCMsgToPacketSubscriber(message, LeftHandCommunicator, PacketDestination.LEFT_HAND.ordinal());

//        	 IHMCMsgToPacketSubscriber<Message> subscriber_R = IHMCMsgToPacketSubscriber.createIHMCMsgToPacketSubscriber(message, RightHandCommunicator, PacketDestination.RIGHT_HAND.ordinal());
        	 
        	 subscribers.add(subscriber_L);
//        	 subscribers.add(subscriber_R);

        	 rosMainNode.attachSubscriber(namespace + rosAnnotation.topic(), subscriber_L);
//        	 rosMainNode.attachSubscriber(namespace + rosAnnotation.topic(), subscriber_R);
        	 
         }
         else 
         {	 
        	 subscriber = IHMCMsgToPacketSubscriber.createIHMCMsgToPacketSubscriber(message, controllerCommunicationBridge, PacketDestination.CONTROLLER.ordinal());
        	 subscribers.add(subscriber);
        	 rosMainNode.attachSubscriber(namespace + rosAnnotation.topic(), subscriber);

         }
         
      }
      
      RequestControllerStopSubscriber requestStopSubscriber = new RequestControllerStopSubscriber(controllerCommunicationBridge);
      rosMainNode.attachSubscriber(namespace + "/control/request_stop", requestStopSubscriber);
   }

   private void setupRosLocalization()
   {
      new RosLocalizationPoseCorrectionSubscriber(rosMainNode, controllerCommunicationBridge, ppsTimestampOffsetProvider);
      new IHMCProntoRosLocalizationUpdateSubscriber(rosMainNode, controllerCommunicationBridge, ppsTimestampOffsetProvider);
   }

   private void setupErrorTopics()
   {
      controllerCommunicationBridge.attachListener(InvalidPacketNotificationPacket.class, new PacketConsumer<InvalidPacketNotificationPacket>()
      {
         @Override
         public void receivedPacket(InvalidPacketNotificationPacket packet)
         {
            System.err.println("Controller recieved invalid packet of type " + packet.packetClass);
            System.err.println("Message: " + packet.errorMessage);
         }
      });

      controllerCommunicationBridge.attachListener(ControllerCrashNotificationPacket.class, new PacketConsumer<ControllerCrashNotificationPacket>()
      {
         @Override
         public void receivedPacket(ControllerCrashNotificationPacket packet)
         {
            System.err.println("Controller crashed at " + packet.location);
            System.err.println("StackTrace: " + packet.stacktrace);
         }
      });
   }
}
