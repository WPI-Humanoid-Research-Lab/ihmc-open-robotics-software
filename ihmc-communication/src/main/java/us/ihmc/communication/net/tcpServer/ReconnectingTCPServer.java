package us.ihmc.communication.net.tcpServer;

import java.io.DataInputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

import us.ihmc.commons.thread.ThreadTools;

public class ReconnectingTCPServer extends ReconnectingTCPConnection
{
   private final ServerSocket singleConnectionServer;
   private Socket socket;

   public ReconnectingTCPServer(int port) throws IOException
   {
      super(2097152);
      singleConnectionServer = new ServerSocket(port);
//      singleConnectionServer.setSoTimeout(5000);

   }

   public void disconnect()
   {
      synchronized (connectionStatusSync)
      {
//         if (getStatus() == Status.CONNECTED)
         {
            try
            {
               inputStream.close();
               outputStream.close();
               socket.close();
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
            System.out.println("Client Disconnected");
            setStatus(Status.DISCONNECTED);
            notifyDisconnectedListeners();
            System.out.println("Waiting for client connection");
            try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
            connect();
         }
      }
   }

   protected void connect()
   {

      if (getStatus() == Status.CLOSED)
      {
         throw new RuntimeException("Connection is closed");
      }
      while (getStatus() == Status.DISCONNECTED)
      {
         try
         {
            socket = singleConnectionServer.accept();
            socket.setTcpNoDelay(true);
            inputStream = new DataInputStream(socket.getInputStream());
            outputStream = socket.getOutputStream();
            reset();
            setStatus(Status.CONNECTED);
            notifyConnectedListeners();
            
         }
         catch (IOException e)
         {
            System.out.println("Connecting timed out: " + e.getMessage());
            ThreadTools.sleep(1000);
         }
         System.out.println("************\n Connection Status:"+ getStatus()+ "\n**************");
      }
      System.out.println("************\n Connected to a client \n**************");
      
   }

   @Override
   public void close()
   {
      synchronized (connectionStatusSync)
      {
    	  if (getStatus() == Status.CONNECTED)
          {
             try
             {
                inputStream.close();
                outputStream.close();
                socket.close();
             }
             catch (IOException e)
             {
                e.printStackTrace();
             }
             System.out.println("Client Disconnected");
             setStatus(Status.DISCONNECTED);
             notifyDisconnectedListeners();
          }
         setStatus(Status.CLOSED);
      }
   }

}
