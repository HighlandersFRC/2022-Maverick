package frc.robot.subsystems;

import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;

public class MqttPublish {

    String content      = "Goodbye Turtles";
    int qos             = 2;
    String broker       = "tcp://10.44.99.11";
    String clientId     = "clientId12";
    MemoryPersistence persistence = new MemoryPersistence();
    MqttClient sampleClient;
    MqttConnectOptions connOpts;
    public MqttPublish() {
        
        try {
            sampleClient = new MqttClient(broker, clientId, persistence);
            connOpts = new MqttConnectOptions();
            connOpts.setCleanSession(true);
            sampleClient.connect(connOpts);
            System.out.println("CONNECTED)))))))))))))))))))))))))");
            // sampleClient.publish("/pathTool", new MqttMessage("LUKE IS DUMB".getBytes()));
        } catch (MqttException e) {
            System.out.println(e + "arshdarsh is bad");
        }
    }

    public void publish(String topic, MqttMessage message) {
        System.out.println("INSIDE PUBLISH @@@@@@@@@@@@@@@@@@@");
        Runnable task = 
        () -> {
            threadSender(message, topic);
        };
        Thread thread = new Thread(task);
        thread.start();

    }

    public void threadSender(MqttMessage message, String topic) {
        try {
            sampleClient.connect();
            sampleClient.publish(topic, message);
            System.out.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% " + message.toString());
        //     // System.out.println("Connected");
        //     // System.out.println("Publishing message: "+content);
        //         // MqttMessage message = new MqttMessage(content.getBytes());
        //     message.setQos(qos);
        //     sampleClient.publish(topic, message);
        //     System.out.println("Message published: " + message);
            sampleClient.disconnect();
            // System.out.println("Disconnected");
            // sampleClient.close();
        //     // System.exit(0);
        } catch(MqttException me) {
        //     // System.out.println("reason "+me.getReasonCode());
        //     // System.out.println("msg "+me.getMessage());
        //     // System.out.println("loc "+me.getLocalizedMessage());
        //     // System.out.println("cause "+me.getCause());
        //     // System.out.println("excep "+me);
             me.printStackTrace();
        }
    }

}
