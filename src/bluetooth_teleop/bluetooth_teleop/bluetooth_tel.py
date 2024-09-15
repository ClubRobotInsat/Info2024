#!/usr/bin/env python3
import rclpy
import serial
import time
from geometry_msgs.msg import Twist
from rclpy.node import Node

# Replace with the serial port and baud rate for your Bluetooth device
BLUETOOTH_PORT = '/dev/rfcomm0'  # Example for Linux, on Windows it could be 'COMx'
BAUD_RATE = 9600                 # Adjust based on your device's settings
SPEED = 1.0                      # Speed of the robot



class BtTeleop(Node):
    def __init__(self):
        super().__init__('bt_teleop')
        
        
        self.get_logger().info('Bluetooth teleop Client Started')
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.read_bluetooth_data()

    def move(self, speed_x, speed_y):
        twist = Twist()
        twist.linear.x = speed_x
        twist.linear.y = speed_y
        twist.angular.z = 0.0
        self.vel_publisher.publish(twist)
        self.get_logger().info(f"Robot should be moving at speed: x={speed_x} ; y={speed_y}.")
    
    def process_received_char(self, char):
        if char == 'z':
            print("Action: Received 'A', taking action A.")
            # Insert your action for receiving 'A' here
            self.move(SPEED, 0.0)

        elif char == 'q':
            print("Action: Received 'q', taking action q.")
            # Insert your action for receiving 'B' here
            self.move(-SPEED, 0.0)

        elif char == 's':
            print("Action: Received 's', taking action s.")
            # Insert your action for receiving 'C' here
            self.move(0.0, SPEED)
        elif char == 'd':
            print("Action: Received 'd', taking action d.")
            # Insert your action for receiving 'C' here
            self.move(0.0, SPEED)

        else:
            print(f"Received unknown character: {char}")
            # Handle other or unknown characters
            self.move(0.0, 0.0)

    def read_bluetooth_data(self):
        """
        This function establishes the connection to the Bluetooth device and reads data from it.
        It continuously listens for incoming characters and processes them.
        """
        try:
            # Open the Bluetooth serial port
            with serial.Serial(BLUETOOTH_PORT, BAUD_RATE, timeout=None) as bt_serial:
                print(f"Connected to Bluetooth device on {BLUETOOTH_PORT} at {BAUD_RATE} baud.")
                
                # while True:
                #     time.sleep(0.1)
                while True:
                    if bt_serial.in_waiting > 0:
                        # Read a single character from the serial buffer
                        char = bt_serial.read(1).decode('utf-8', errors='ignore')
                        if char:
                            # Process the received character
                            self.process_received_char(char)

                    # Add a small delay to prevent CPU overuse
                    time.sleep(0.1)

        except serial.SerialException as e:
            print(f"Error: Could not open serial port {BLUETOOTH_PORT}: {e}")

        except KeyboardInterrupt:
            print("\nBluetooth reading stopped by user.")
    
    

def main(args=None):
    try:
        rclpy.init(args=args)
        bt = BtTeleop()
            
        rclpy.spin(bt)
    except KeyboardInterrupt:
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()



