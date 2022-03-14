from paho.mqtt import client as mqtt_client

broker = '10.44.99.11'
port = 1883
topic = "/pathTool"
client_id = "99H44"

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)
    # Set Connecting Client ID
    client = mqtt_client.Client(client_id)
    client.username_pw_set("4499", "4499")
    client.on_connect = on_connect
    client.connect(broker)
    return client

def subscribe(client):
    def on_message(client, userdata, msg):
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

    # print("Listening")
    client.subscribe(topic, 2)
    client.on_message = on_message    

client = connect_mqtt()

client.loop_start()

while(True):
    subscribe(client)

print("Im done")