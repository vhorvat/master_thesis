import serial
import struct
import time
import threading
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

class OptoForceSensor:
    CMD_HEADER = bytes([0xAA, 0x00, 0x32, 0x03])
    CONFIG_CMD_LENGTH = 9

    DATA_FRAME_START_BYTES = bytes([0xAA, 0x07, 0x08, 0x10])
    DATA_FRAME_LENGTH = 22

    class SpeedCode(Enum):
        STOP = 0      
        S_1000HZ = 1
        S_333HZ = 3
        S_100HZ = 10
        S_30HZ = 33
        S_10HZ = 100

    class FilterCode(Enum):
        NO_FILTER = 0 
        F_500HZ = 1
        F_150HZ = 2
        F_50HZ = 3
        F_15HZ = 4
        F_5HZ = 5
        F_1_5HZ = 6 

    def __init__(self, port='/dev/ttyACM0', baudrate=1000000, 
                 initial_speed_hz=1000, initial_filter_hz=30, zero_on_init=True,
                 fx_gain=33.74, fy_gain=35.61, fz_gain=4.05,
                 tx_gain=587.20, ty_gain=566.37, tz_gain=868.35,
                 timeout=0.05): 
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.buffer = bytearray()

        self.fx_gain = fx_gain
        self.fy_gain = fy_gain
        self.fz_gain = fz_gain
        self.tx_gain = tx_gain
        self.ty_gain = ty_gain
        self.tz_gain = tz_gain

        self.is_streaming = False
        self._current_speed_code = self.SpeedCode.STOP.value
        self._current_filter_code = self.FilterCode.F_15HZ.value
        self._last_zero_command = False

        print(f"OptoForce sensor initialized on {port} at {baudrate} baud.")
        print(f"Gains: Fx={fx_gain}, Fy={fy_gain}, Fz={fz_gain}, Tx={tx_gain}, Ty={ty_gain}, Tz={tz_gain}")

        self.set_config(speed_hz=initial_speed_hz, filter_hz=initial_filter_hz, zero=zero_on_init)
        time.sleep(0.1)

    def _map_speed_hz_to_code(self, speed_hz):
        if speed_hz == 1000: return self.SpeedCode.S_1000HZ.value
        if speed_hz == 333: return self.SpeedCode.S_333HZ.value
        if speed_hz == 100: return self.SpeedCode.S_100HZ.value
        if speed_hz == 30: return self.SpeedCode.S_30HZ.value
        if speed_hz == 10: return self.SpeedCode.S_10HZ.value
        if speed_hz == 0: return self.SpeedCode.STOP.value
        print(f"Warning: Invalid speed {speed_hz}Hz. Using 100Hz.")
        return self.SpeedCode.S_100HZ.value

    def _map_filter_hz_to_code(self, filter_hz):
        if filter_hz == 0: return self.FilterCode.NO_FILTER.value 
        if filter_hz == 500: return self.FilterCode.F_500HZ.value
        if filter_hz == 150: return self.FilterCode.F_150HZ.value
        if filter_hz == 50: return self.FilterCode.F_50HZ.value
        if filter_hz == 15: return self.FilterCode.F_15HZ.value
        if filter_hz == 5: return self.FilterCode.F_5HZ.value
        if filter_hz == 1.5: return self.FilterCode.F_1_5HZ.value
        print(f"Warning: Invalid filter {filter_hz}Hz. Using 15Hz.")
        return self.FilterCode.F_15HZ.value

    def set_config(self, speed_hz=None, filter_hz=None, zero=None):
        if not self.ser.is_open:
            print("Serial port not open.")
            return False

        speed_code_to_send = self._current_speed_code if speed_hz is None else self._map_speed_hz_to_code(speed_hz)
        filter_code_to_send = self._current_filter_code if filter_hz is None else self._map_filter_hz_to_code(filter_hz)
        zero_byte_to_send = 0xFF if zero else 0x00 if zero is not None else (0xFF if self._last_zero_command else 0x00)

        command = bytearray(self.CONFIG_CMD_LENGTH)
        command[0:4] = self.CMD_HEADER
        command[4] = speed_code_to_send
        command[5] = filter_code_to_send
        command[6] = zero_byte_to_send

        checksum = sum(command[0:7]) & 0xFFFF
        command[7] = (checksum >> 8) & 0xFF
        command[8] = checksum & 0xFF

        try:
            self.ser.reset_input_buffer()
            self.ser.write(command)
            self.ser.flush()

            self._current_speed_code = speed_code_to_send
            self._current_filter_code = filter_code_to_send
            if zero is not None: self._last_zero_command = zero
            self.is_streaming = speed_code_to_send != self.SpeedCode.STOP.value
            time.sleep(0.05)
            return True
        except serial.SerialException as e:
            print(f"Error sending config: {e}")
            return False

    def _calculate_data_checksum(self, frame_bytes):
        return sum(frame_bytes) & 0xFFFF

    def _parse_frame(self, frame):
        if len(frame) != self.DATA_FRAME_LENGTH or frame[0:4] != self.DATA_FRAME_START_BYTES:
            return None

        try:
            fx_raw, fy_raw, fz_raw, tx_raw, ty_raw, tz_raw = struct.unpack('>hhhhhh', frame[8:20])
            received_checksum, = struct.unpack('>H', frame[20:22])
        except struct.error:
            return None

        if received_checksum != self._calculate_data_checksum(frame[:20]):
            return None

        return {
            "Fx": fx_raw / self.fx_gain,
            "Fy": fy_raw / self.fy_gain,
            "Fz": fz_raw / self.fz_gain,
            "Tx": tx_raw / self.tx_gain,
            "Ty": ty_raw / self.ty_gain,
            "Tz": tz_raw / self.tz_gain
        }

    def read_data_packet(self):
        if not self.ser.is_open: return None

        if self.ser.in_waiting:
            self.buffer.extend(self.ser.read(self.ser.in_waiting))

        while True:
            start_index = self.buffer.find(self.DATA_FRAME_START_BYTES)
            if start_index == -1:
                self.buffer = self.buffer[-(self.DATA_FRAME_LENGTH - 1):]
                return None
            if len(self.buffer) < start_index + self.DATA_FRAME_LENGTH:
                return None

            frame = self.buffer[start_index : start_index + self.DATA_FRAME_LENGTH]
            del self.buffer[start_index : start_index + self.DATA_FRAME_LENGTH]
            parsed = self._parse_frame(frame)
            if parsed: return parsed

    def close(self):
        if self.ser and self.ser.is_open:
            if self.is_streaming:
                self.set_config(speed_hz=0)
            self.ser.close()


class ForcePublisher(Node):
    def __init__(self, sensor):
        super().__init__('optoforce_publisher')
        self.sensor = sensor
        self.publisher = self.create_publisher(WrenchStamped, 'optoforce_data', 10)
        self.timer = self.create_timer(0.01, self.publish_data)  # 100 Hz

    def publish_data(self):
        data = self.sensor.read_data_packet()
        if data:
            msg = WrenchStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'sensor_frame'
            msg.wrench.force.x = data['Fx']
            msg.wrench.force.y = data['Fy']
            msg.wrench.force.z = data['Fz']
            msg.wrench.torque.x = data['Tx']
            msg.wrench.torque.y = data['Ty']
            msg.wrench.torque.z = data['Tz']
            self.publisher.publish(msg)
            
            print(f"Fx: {data['Fx']:.2f} N, Fy: {data['Fy']:.2f} N, Fz: {data['Fz']:.2f} N | "
                  f"Tx: {data['Tx']:.3f} Nm, Ty: {data['Ty']:.3f} Nm, Tz: {data['Tz']:.3f} Nm")

def main():
    rclpy.init()
    sensor = OptoForceSensor(
        port='/dev/ttyACM1',
        baudrate=1000000,
        initial_speed_hz=100,
        initial_filter_hz=15,
        zero_on_init=True
    )
    node = ForcePublisher(sensor)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sensor.close()

if __name__ == '__main__':
    main()
