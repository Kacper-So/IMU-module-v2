import paho.mqtt.client as mqtt

broker_host = "127.0.0.1"
broker_port = 1883

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("AHRS/data_Madgwick")
    client.subscribe("AHRS/data_dataMahony")

def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)

client.on_connect = on_connect
client.on_message = on_message

client.connect(broker_host, broker_port, 60)

client.loop_forever()
