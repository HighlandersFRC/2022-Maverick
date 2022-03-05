package frc.robot.subsystems;

import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;

import edu.wpi.first.wpilibj.Timer;

public class MqttSubscribe implements MqttCallback  {

	/** The broker url. */
	private static final String brokerUrl ="tcp://10.44.99.11";

	/** The client id. */
	private static final String clientId = "clientId12";

	/** The topic. */
	private static final String topic = "/sensors/camera";

	private boolean connection = false;

	private double crashTime;
	private double startTime;

    private String latestMessage = "";

	public static void main(String[] args) {

		System.out.println("Subscriber running");
		new MqttSubscribe().subscribe(topic);

	}

    public String getLatestMessage() {
		// System.out.println(latestMessage);
        return latestMessage;
    }

	public void subscribe(String topic) {
		Runnable task = 
        () -> {

			while(true){
				MemoryPersistence persistence = new MemoryPersistence();
				try
				{
					MqttClient sampleClient = new MqttClient(brokerUrl, clientId, persistence);
					MqttConnectOptions connOpts = new MqttConnectOptions();
					connOpts.setCleanSession(true);
					
					// System.out.println("checking");
					// System.out.println("Mqtt Connecting to broker: " + brokerUrl);
					
                    if(connection == false) {
                        sampleClient.connect(connOpts);
                        // System.out.println("Mqtt Connected");
                        connection = true;                        
                        sampleClient.setCallback(this);
					    sampleClient.subscribe(topic);
                    }
					
					
					// System.out.println("Subscribed");
					// System.out.println("Listening");
					startTime = Timer.getFPGATimestamp();
					// System.out.println("Start Time: " + startTime);

				} catch (MqttException me) {
					// System.out.println(me);
				}

			

			}
			
				
		
	};
	Thread thread = new Thread(task);
	thread.start();
	}

	 //Called when the client lost the connection to the broker
	public void connectionLost(Throwable arg0) {
		// System.out.println("CONNECTION LOST");
		connection = false;
		crashTime = Timer.getFPGATimestamp();
		// System.out.println("Crash Time: "+ crashTime);
		
	}

	//Called when a outgoing publish is complete
	public void deliveryComplete(IMqttDeliveryToken arg0) {

	}

	public void messageArrived(String topic, MqttMessage message) throws Exception {
        latestMessage = message.toString();
		// System.out.println("| Topic:" + topic);
		// System.out.println("| Message: " +message.toString());
		// System.out.println("-------------------------------------------------");

	}

}