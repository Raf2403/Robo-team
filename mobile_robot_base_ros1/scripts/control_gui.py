#!/usr/bin/env python

import tkinter as tk  # Import the tkinter library for creating a graphical interface
from std_msgs.msg import String  # Import the String message type from the std_msgs package
import rospy  # Import the rospy library for working with ROS in Python

class RobotControlGUI:
    def __init__(self, master):
        self.master = master  # Define the root widget
        master.title("Robot Control")  # Set the title of the main window

        rospy.init_node('robot_gui_controller', anonymous=True)  # Initialize the ROS node with the name 'robot_gui_controller'
        self.publisher = rospy.Publisher('my_motor_commands', String, queue_size=10)  # Define a publisher to send commands

        self.speed_label = tk.Label(master, text="Current Speed: 0.0")  # Label to display the current speed
        self.speed_label.pack()  # Place the label on the main window

        # Buttons to send movement commands to the robot
        tk.Button(master, text="Go", command=lambda: self.send_command("GO")).pack()
        tk.Button(master, text="Go Really Fast", command=lambda: self.send_command("GO_REALLY_FAST")).pack()
        tk.Button(master, text="Back", command=lambda: self.send_command("BACK")).pack()
        tk.Button(master, text="Left", command=lambda: self.send_command("LEFT")).pack()
        tk.Button(master, text="Right", command=lambda: self.send_command("RIGHT")).pack()
        tk.Button(master, text="Stop", command=lambda: self.send_command("STOP")).pack()

    # Method to send a movement command to the robot
    def send_command(self, command):
        self.publisher.publish(String(command))  # Publish the command to the 'my_motor_commands' topic
        self.speed_label.config(text=f"Current Speed: {self.get_speed(command)}")  # Update the label with the current speed

    # Method to get the speed based on the command
    def get_speed(self, command):
        if command == "GO":
            return "-1.0"
        elif command == "GO_REALLY_FAST":
            return "-10.0"
        elif command == "BACK":
            return "0.5"
        elif command == "LEFT" or command == "RIGHT":
            return "Variable"
        else:
            return "0.0"

if __name__ == "__main__":
    root = tk.Tk()  # Create the main application window
    gui = RobotControlGUI(root)  # Create an instance of the RobotControlGUI class
    root.mainloop()  # Run the main application loop
