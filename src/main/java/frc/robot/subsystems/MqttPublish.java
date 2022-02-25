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

    public MqttPublish() {

    }

    public void publish(String topic, MqttMessage message) {
        Runnable task = 
        () -> {
            try {
                MqttClient sampleClient = new MqttClient(broker, clientId, persistence);
                MqttConnectOptions connOpts = new MqttConnectOptions();
                connOpts.setCleanSession(true);
                // System.out.println("Connecting to broker: "+broker);
                sampleClient.connect(connOpts);
                // System.out.println("Connected");
                // System.out.println("Publishing message: "+content);
                while(true) {
                    // MqttMessage message = new MqttMessage(content.getBytes());
                    message.setQos(qos);
                    sampleClient.publish(topic, message);
                    System.out.println("Message published");
                }   
                // sampleClient.disconnect();
                // System.out.println("Disconnected");
                // sampleClient.close();
                // System.exit(0);
            } catch(MqttException me) {
                System.out.println("reason "+me.getReasonCode());
                System.out.println("msg "+me.getMessage());
                System.out.println("loc "+me.getLocalizedMessage());
                System.out.println("cause "+me.getCause());
                System.out.println("excep "+me);
                me.printStackTrace();
            }
        };
        Thread thread = new Thread(task);
        thread.start();
    }
        
}
