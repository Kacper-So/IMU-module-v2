import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib.animation import FuncAnimation

class AHRSVisualizer:
    def __init__(self, mqtt_broker, mqtt_topic):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(-1, 1)

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(mqtt_broker, 1883, 60)
        self.client.loop_start()

        self.mqtt_topic = mqtt_topic

        self.box = np.array([[-0.5, -0.5, -0.5],
                             [-0.5,  0.5, -0.5],
                             [ 0.5,  0.5, -0.5],
                             [ 0.5, -0.5, -0.5],
                             [-0.5, -0.5,  0.5],
                             [-0.5,  0.5,  0.5],
                             [ 0.5,  0.5,  0.5],
                             [ 0.5, -0.5,  0.5]])

        self.edges = [[0, 1], [1, 2], [2, 3], [3, 0],
                      [4, 5], [5, 6], [6, 7], [7, 4],
                      [0, 4], [1, 5], [2, 6], [3, 7]]

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        self.client.subscribe(self.mqtt_topic)

    def on_message(self, client, userdata, msg):
        data = msg.payload.decode("utf-8").split(";")
        if len(data) == 3:
            self.roll = float(data[0]) * np.pi / 180.0  # Convert degrees to radians
            self.pitch = float(data[1]) * np.pi / 180.0  # Convert degrees to radians
            # self.yaw = float(data[2]) * np.pi / 180.0  # Convert degrees to radians
            self.yaw = 0.0  # Convert degrees to radians

    def update_plot(self, frame):
        self.ax.clear()

        # Rotate box
        rotation_matrix = self.rotation_matrix(self.roll, self.pitch, self.yaw)
        rotated_box = np.dot(self.box, rotation_matrix.T)

        # Draw edges of the rotated box
        for edge in self.edges:
            self.ax.plot3D(rotated_box[edge, 0], rotated_box[edge, 1], rotated_box[edge, 2], 'b')

        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(-1, 1)

    def rotation_matrix(self, roll, pitch, yaw):
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])

        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])

        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])

        return np.dot(R_z, np.dot(R_y, R_x))

    def visualize(self):
        ani = FuncAnimation(self.fig, self.update_plot, blit=False)
        plt.show()

if __name__ == "__main__":
    mqtt_broker = "localhost"  # Change to your MQTT broker IP or hostname
    mqtt_topic = "AHRS/data_Madgwick"  # Change to your MQTT topic for AHRS data

    visualizer = AHRSVisualizer(mqtt_broker, mqtt_topic)
    visualizer.visualize()
