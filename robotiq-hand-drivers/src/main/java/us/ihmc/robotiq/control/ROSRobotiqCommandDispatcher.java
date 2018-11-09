package us.ihmc.robotiq.control;

import java.io.IOException;
import java.net.URI;

import java.net.URISyntaxException;

import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.subscribers.HandDesiredConfigurationMessageSubscriber;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.utilities.ros.RosTools;


public class ROSRobotiqCommandDispatcher implements Runnable
{

   private final RobotiqHandCommandManager rightRobotiqHandCommandManager;   
   private final RobotiqHandCommandManager leftRobotiqHandCommandManager;
   
   private final PacketCommunicator RightHandCommunicator; 
   private final PacketCommunicator LeftHandCommunicator;
   
   private final PacketCommunicator IHMCHandDesiredMessageCommunicator;
   
   public ROSRobotiqCommandDispatcher( )
   {
	 //---------------------------------------------------------------------------------------------------------------------// 
      IHMCHandDesiredMessageCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(
    		  NetworkPorts.CUSTOM_ROBOTIQ_HAND_COMMAND_DISPATCHER_PORT, new IHMCCommunicationKryoNetClassList());

      
      try
      {
         IHMCHandDesiredMessageCommunicator.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      
      
      
      
      rightRobotiqHandCommandManager = new RobotiqHandCommandManager(RobotSide.RIGHT);
      leftRobotiqHandCommandManager = new RobotiqHandCommandManager(RobotSide.LEFT);
      
      //---------------------------------------------------------------------------------------------------------------------//
      // create hand communicators
      this.LeftHandCommunicator = 
    		  PacketCommunicator.createIntraprocessPacketCommunicator(	NetworkPorts.LEFT_HAND_MANAGER_PORT,
    				  													new IHMCCommunicationKryoNetClassList());
      this.RightHandCommunicator = 
    		  PacketCommunicator.createIntraprocessPacketCommunicator(	NetworkPorts.RIGHT_HAND_MANAGER_PORT,
    				  													new IHMCCommunicationKryoNetClassList());
      
      // connect hand communicators
      try 
      {    	  
    	  System.out.println("connecting left hand ...");
    	  this.LeftHandCommunicator.connect();
      }
      catch (IOException e1)
      {
    	  e1.printStackTrace();
      }
      
      try 
      {    	  
    	  System.out.println("connecting right hand ...");
    	  this.RightHandCommunicator.connect();
      }
      catch (IOException e1)
      {
    	  e1.printStackTrace();
      }
      //---------------------------------------------------------------------------------------------------------------------//
      System.out.println("Hand packet router running");
      
      // Callback for /ihmc_ros/atlas/control/hand_desired_configuration
      IHMCHandDesiredMessageCommunicator.attachListener(
    	      HandDesiredConfigurationMessage.class, new PacketConsumer<HandDesiredConfigurationMessage>()
    	      {
    	         public void receivedPacket(HandDesiredConfigurationMessage ihmcMessage)
    	         {
    	        	 System.out.println(this.getClass().toString()+"->side"+ihmcMessage.robotSide.toString()+":"+ihmcMessage.getHandDesiredConfiguration().toString());
    	        	 if (ihmcMessage.getRobotSide()==RobotSide.LEFT) {
    	             	System.out.println("got LEFT HAND msg");
    	             	LeftHandCommunicator.send(ihmcMessage);
    	             }
    	             else 
    	             {
    	             	System.out.println("got RIGHT HAND msg");
    	             	RightHandCommunicator.send(ihmcMessage);
    	             }
    	         }
    	      });
    	      
   }

   @Override
   public void run()
   {
	   System.out.println("Running Hand Dispacher ...");
      while (true)
      {
         
      }
   }
}
