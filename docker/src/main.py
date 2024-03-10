import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import json
import paho.mqtt.client as mqtt

broker_address = "127.0.0.1"
port = 1883
topic = "AHRS/data_Madgwick"

roll_data = []
pitch_data = []
yaw_data = []

def on_message(client, userdata, message):
    try:
        msg_data = message.payload.decode().split(';')
        roll = float(msg_data[0])
        pitch = float(msg_data[1])
        yaw = float(msg_data[2])
        
        roll_data.append(roll)
        pitch_data.append(pitch)
        yaw_data.append(yaw)

        fig = plt.figure(figsize=(10, 8))
        
        plt.subplot(2, 2, 1)
        plt.plot(roll, label='Roll')
        plt.title('Roll')
        plt.legend()
        
        plt.subplot(2, 2, 2)
        plt.plot(pitch, label='Pitch')
        plt.title('Pitch')
        plt.legend()
        
        plt.subplot(2, 2, 3)
        plt.plot(yaw, label='Yaw')
        plt.title('Yaw')
        plt.legend()
        
        ax = fig.add_subplot(2, 2, 4, projection='3d')
        ax.plot(roll, pitch, yaw, label='AHRS Data')
        ax.set_xlabel('Roll')
        ax.set_ylabel('Pitch')
        ax.set_zlabel('Yaw')
        ax.set_title('AHRS Data 3D Visualization')
        ax.legend()
        
        plt.tight_layout()
        plt.show()
        
    except Exception as e:
        print("Error:", e)

client = mqtt.Client()
client.on_message = on_message

client.connect(broker_address, port=port)
client.subscribe(topic)

client.loop_forever()

client.disconnect()
