#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import threading 

class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader_node')

        self.declare_parameter('serial_port', '/dev/esp_load_cell') 
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('topic_name', 'serial_esp_data')
        self.declare_parameter('read_timeout', 0.1) 
        self.declare_parameter('loop_rate', 100.0) 


        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.read_timeout = self.get_parameter('read_timeout').get_parameter_value().double_value
        self.loop_rate = self.get_parameter('loop_rate').get_parameter_value().double_value


        self.publisher_ = self.create_publisher(String, self.topic_name, 10) # QoS profile depth 10

        self.get_logger().info(f"Attempting to open serial port: {self.serial_port_name} at {self.baud_rate} baud.")

        self.serial_port = None
        self.serial_thread = None
        self.running = False

        try:
            self.serial_port = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baud_rate,
                timeout=self.read_timeout # Read timeout
                # bytesize=serial.EIGHTBITS,
                # parity=serial.PARITY_NONE,
                # stopbits=serial.STOPBITS_ONE
            )

            time.sleep(2)

            if self.serial_port.is_open:
                self.get_logger().info(f"Serial port {self.serial_port_name} opened successfully.")
                self.running = True

                self.serial_thread = threading.Thread(target=self.read_serial_data)
                self.serial_thread.daemon = True 
                self.serial_thread.start()
            else:
                 self.get_logger().error(f"Failed to open serial port {self.serial_port_name}.")

        except serial.SerialException as e:
            self.get_logger().error(f"Error opening / reading from serial port {self.serial_port_name}: {e}")
        except Exception as e:
             self.get_logger().error(f"An unexpected error: {e}")


    def read_serial_data(self):
        """Runs in a separate thread to read serial data."""
        self.get_logger().info("Serial reading thread started.")
        loop_sleep_duration = 1.0 / self.loop_rate

        while self.running and rclpy.ok() and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting > 0:

                    line = self.serial_port.readline()

                    if line:
                        try:
                            data_str = line.decode('utf-8', errors='replace').strip()
                            if data_str: 
                                msg = String()
                                msg.data = data_str
                                self.publisher_.publish(msg)
                                # self.get_logger().debug(f'Published: "{data_str}"')
                        except UnicodeDecodeError as e:
                             self.get_logger().warn(f"Could not decode line from serial: {e} - Raw: {line}")
                        except Exception as e:
                             self.get_logger().error(f"Error processing received line: {e}")
                else:

                    time.sleep(loop_sleep_duration) 
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}. Closing port.")
                self.running = False 
                break 
            except Exception as e:
                 self.get_logger().error(f"Error in serial reading thread: {e}")
                 time.sleep(1.0) 

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed.")
        self.get_logger().info("Serial reading thread finished.")


    def shutdown_hook(self):
        """Called on node shutdown."""
        self.get_logger().info("Shutdown requested.")
        self.running = False
        if self.serial_thread:
            self.serial_thread.join(timeout=2.0) 
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed")


def main(args=None):
    rclpy.init(args=args)
    serial_reader_node = SerialReaderNode()
    try:
        rclpy.spin(serial_reader_node)
    except KeyboardInterrupt:
         serial_reader_node.get_logger().info('Keyboard interrupt.')
    except Exception as e:
         serial_reader_node.get_logger().error(f"Exception during spin: {e}")
    finally:

        serial_reader_node.shutdown_hook()
        serial_reader_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
        serial_reader_node.get_logger().info("Node shutdown.")

if __name__ == '__main__':
    main()
