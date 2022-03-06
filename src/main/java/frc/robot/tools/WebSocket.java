package frc.robot.tools;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;

public class WebSocket {
    
    ServerSocket serverSocket;
    String fromClient = "";

    Runnable task = 
        () -> {

            try {
                serverSocket = new ServerSocket(8001);
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }

            System.out.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~");

			while(true){
                System.out.println("################");
                try {
                    Socket socket = serverSocket.accept();
                    System.out.println("MADE IT PAST ACCEPT");
                    BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                    while(fromClient != null) {
                        fromClient = in.readLine();
                        System.out.println("@@@@@@@@@@@@@@@@@@@@@@ FROM CLIENT: " + fromClient);
                    }
                } catch (IOException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }

        
        };

    public void startListening() {
        Thread thread = new Thread(task);
        thread.start();
    }
}
