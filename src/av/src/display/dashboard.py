import os
import rclpy
import tkinter as tk
from rclpy.node import Node
from tkinter import messagebox
from std_msgs.msg import Float32
from geopy.geocoders import Photon
from sensor_msgs.msg import NavSatFix

class AutowareDashboard(Node):
    def __init__(self, master):
        super().__init__('dashboard_node')

        # ROS2 initialization
        self.speed_subscriber = self.create_subscription(
            Float32, 'speed_topic', self.speed_callback, 10)
        self.brake_subscriber = self.create_subscription(
            Float32, 'brake_topic', self.brake_callback, 10)
        self.angle_subscriber = self.create_subscription(
            Float32, 'angle_topic', self.angle_callback, 10)
        self.get_logger().info('Dashboard node initialized')

        # ROS2 publishers
        self.coordinate_publisher = self.node.create_publisher(
            NavSatFix, 'gps_coordinates', 10)

        # TKinter initialization
        self.master = master
        master.title("Dashboard")
        master.configure(bg="#f0f0f0")  # Set background color

        # Logo
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.logo_img = tk.PhotoImage(
            file=os.path.join(script_dir, "logo.png"))
        self.logo_label = tk.Label(master, image=self.logo_img, bg="#f0f0f0")
        self.logo_label.grid(row=0, column=0, columnspan=5,
                             pady=10, sticky="nsew")

        # Labels for vehicle values
        self.label_speed = tk.Label(master, text="Speed: ", bg="#f0f0f0")
        self.label_speed.grid(row=1, column=0, sticky="nsew")

        self.label_steering_angle = tk.Label(
            master, text="Steering Angle: ", bg="#f0f0f0")
        self.label_steering_angle.grid(row=1, column=1, sticky="nsew")

        self.label_brake = tk.Label(master, text="Brake: ", bg="#f0f0f0")
        self.label_brake.grid(row=1, column=3, sticky="nsew")

        # Address Input
        self.address_label = tk.Label(
            master, text="Enter Destination Address:", bg="#f0f0f0")
        self.address_label.grid(row=3, column=0, sticky="nsew")
        self.address_entry = tk.Entry(master, width=50)
        self.address_entry.grid(row=3, column=1, columnspan=3, sticky="nsew")

        # Button to Convert Address
        self.convert_button = tk.Button(
            master, text="Convert Address", command=self.convert_address, bg="#4CAF50", fg="white")
        self.convert_button.grid(row=3, column=4, sticky="nsew")

        # Configure row and column weights to center content
        for i in range(4):
            master.grid_rowconfigure(i, weight=1)
        for j in range(5):
            master.grid_columnconfigure(j, weight=1)

    def convert_address(self):
        address = self.address_entry.get()
        geolocator = Photon(user_agent="measurements")
        location = geolocator.geocode(address)
        try:
            if location:
                addr = location.address
                lat = location.latitude
                lon = location.longitude
                self.publish_coordinates(lat, lon)
                messagebox.showinfo(
                    "Success", f"Location: {addr} \nLatitude: {lat}\nLongitude: {lon}")
            else:
                messagebox.showerror("Error", "Address not found!")
        except Exception as e:
            messagebox.showerror("Error", f"Error: {str(e)}")

    def publish_coordinates(self, lat, lon):
        coordinate_msg = NavSatFix()
        coordinate_msg.latitude = lat
        coordinate_msg.longitude = lon
        self.coordinate_publisher.publish(coordinate_msg)

    def speed_callback(self, msg):
        speed = msg.data
        self.label_speed.config(text=f"Speed: {speed}")

    def brake_callback(self, msg):
        brake = msg.data
        self.label_brake.config(text=f"Brake: {brake}")

    def angle_callback(self, msg):
        angle = msg.data
        self.label_steering_angle.config(text=f"Steering Angle: {angle}")


def main():
    rclpy.init()
    root = tk.Tk()
    dashboard = AutowareDashboard(root)
    rclpy.spin(dashboard)
    root.mainloop()

if __name__ == "__main__":
    main()
