#****** THIS IS DEVELOPMENT VERSION *******
# Optoforce read script, only for ttyACM0,
# no ethernet. 
# ROS2 Package is derived from this

import serial
import struct
import time
import threading # for potential future use if strict threading is needed
from enum import Enum

class OptoForceSensor:
    # constants for the protocol used by the C++ optoDriver, if yyou have access to datasheet fix them
    CMD_HEADER = bytes([0xAA, 0x00, 0x32, 0x03]) # 170, 0, 50, 3
    CONFIG_CMD_LENGTH = 9 # header (4) + speed (1) + filter (1) + zero (1) + checksum (2)

    DATA_FRAME_START_BYTES = bytes([0xAA, 0x07, 0x08, 0x10]) # 170, 7, 8, 16
    DATA_FRAME_LENGTH = 22

    # speed codes for setConfig (from C++ driver logic)
    # user provides Hz, class maps to code
    class SpeedCode(Enum):
        STOP = 0      
        S_1000HZ = 1
        S_333HZ = 3
        S_100HZ = 10
        S_30HZ = 33
        S_10HZ = 100

    # codes for setConfig (from C++ driver logic)
    # user provides Hz, class maps to code
    class FilterCode(Enum):
        NO_FILTER = 0 
        F_500HZ = 1
        F_150HZ = 2
        F_50HZ = 3
        F_15HZ = 4
        F_5HZ = 5
        F_1_5HZ = 6 

    def __init__(self, port='/dev/ttyACM0', baudrate=1000000, # C++ driver uses 1Mbaud
                 initial_speed_hz=1000, initial_filter_hz=15, zero_on_init=True,
                 fx_gain=33.74, fy_gain=35.61, fz_gain=9.0,
                 tx_gain=100.0, ty_gain=100.0, tz_gain=100.0,
                 timeout=0.05): 
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.buffer = bytearray()

        # gains (raw_value / gain = actual_value)
        # C++ calArray would be [1/fx_gain, 1/fy_gain, ...]
        # try using Brunos yaml numbers
        self.fx_gain = fx_gain
        self.fy_gain = fy_gain
        self.fz_gain = fz_gain
        self.tx_gain = tx_gain
        self.ty_gain = ty_gain
        self.tz_gain = tz_gain

        self.is_streaming = False
        self._current_speed_code = self.SpeedCode.STOP.value # Store the actual code
        self._current_filter_code = self.FilterCode.F_15HZ.value # Default if not set 
        self._last_zero_command = False

        print(f"OptoForce sensor (C++ compatible mode) initialized on {port} at {baudrate} baud.")
        print(f"Using gains: Fx={fx_gain}, Fy={fy_gain}, Fz={fz_gain}, Tx={tx_gain}, Ty={ty_gain}, Tz={tz_gain}")

        self.set_config(speed_hz=initial_speed_hz, filter_hz=initial_filter_hz, zero=zero_on_init)
        time.sleep(0.1) # timeout time

    def _map_speed_hz_to_code(self, speed_hz):
        if speed_hz == 1000: return self.SpeedCode.S_1000HZ.value
        if speed_hz == 333: return self.SpeedCode.S_333HZ.value
        if speed_hz == 100: return self.SpeedCode.S_100HZ.value
        if speed_hz == 30: return self.SpeedCode.S_30HZ.value
        if speed_hz == 10: return self.SpeedCode.S_10HZ.value
        if speed_hz == 0: return self.SpeedCode.STOP.value
        print(f"Warning: Invalid speed {speed_hz}Hz. Using 100Hz (code 10).")
        return self.SpeedCode.S_100HZ.value

    def _map_filter_hz_to_code(self, filter_hz):
        if filter_hz == 0: return self.FilterCode.NO_FILTER.value 
        if filter_hz == 500: return self.FilterCode.F_500HZ.value
        if filter_hz == 150: return self.FilterCode.F_150HZ.value
        if filter_hz == 50: return self.FilterCode.F_50HZ.value
        if filter_hz == 15: return self.FilterCode.F_15HZ.value
        if filter_hz == 5: return self.FilterCode.F_5HZ.value
        if filter_hz == 1.5: return self.FilterCode.F_1_5HZ.value
        print(f"Warning: Invalid filter {filter_hz}Hz. Using 15Hz (code 4).")
        return self.FilterCode.F_15HZ.value

    def set_config(self, speed_hz=None, filter_hz=None, zero=None):
        if not self.ser.is_open:
            print("Serial port not open. Cannot set config.")
            return False

        speed_code_to_send = self._current_speed_code
        if speed_hz is not None:
            speed_code_to_send = self._map_speed_hz_to_code(speed_hz)

        filter_code_to_send = self._current_filter_code
        if filter_hz is not None:
            filter_code_to_send = self._map_filter_hz_to_code(filter_hz)

        zero_byte_to_send = 0x00
        if zero is not None:
            zero_byte_to_send = 0xFF if zero else 0x00
        elif self._last_zero_command: # if no zero, latest state
             zero_byte_to_send = 0xFF if self._last_zero_command else 0x00


        command = bytearray(self.CONFIG_CMD_LENGTH)
        command[0:4] = self.CMD_HEADER
        command[4] = speed_code_to_send
        command[5] = filter_code_to_send
        command[6] = zero_byte_to_send

        checksum = sum(command[0:7]) & 0xFFFF # first 7 summed up
        command[7] = (checksum >> 8) & 0xFF # checksum high byte
        command[8] = checksum & 0xFF       # checksum low byte

        try:
            self.ser.reset_input_buffer() # clean buffer
            self.ser.write(command)
            self.ser.flush() # wait for data
            print(f"Sent config: SpeedCode={speed_code_to_send}, FilterCode={filter_code_to_send}, Zero={hex(zero_byte_to_send)}. Command: {command.hex()}")

            # update current state
            self._current_speed_code = speed_code_to_send
            self._current_filter_code = filter_code_to_send
            if zero is not None: 
                self._last_zero_command = zero

            if speed_code_to_send == self.SpeedCode.STOP.value:
                self.is_streaming = False
                print("Streaming stopped via set_config.")
            else:
                self.is_streaming = True
                print("Streaming (re)started/configured via set_config.")
            time.sleep(0.05) # timeout time!
            return True
        except serial.SerialException as e:
            print(f"Error sending config: {e}")
            return False

    def start_streaming(self):
        """Starts streaming using the last known non-zero speed, or a default."""
        if not self.is_streaming:
            # if current speed is STOP, use default 100Hz or last non-zero speed
            speed_to_start = self._current_speed_code
            if speed_to_start == self.SpeedCode.STOP.value:
                print("Current speed is STOP, attempting to start with 100Hz.")
                # find the Hz value for S_100HZ code for the set_config call
                # this is a bit clunky
                start_speed_hz = 100 # Ddfault 100 HZ if was stopped
                for hz, member in [(1000, self.SpeedCode.S_1000HZ), (333, self.SpeedCode.S_333HZ),
                                   (100, self.SpeedCode.S_100HZ), (30, self.SpeedCode.S_30HZ),
                                   (10, self.SpeedCode.S_10HZ)]:
                    if self._current_speed_code == member.value and self._current_speed_code != self.SpeedCode.STOP.value:
                        start_speed_hz = hz # Use last active speed if available
                        break
            else: # map current code back to Hz
                # this mapping is just to satisfy set_config's Hz input
                # a more direct set_config(speed_code=...) might be better internally
                hz_map_rev = {v.value: k for k, v in {
                    1000: self.SpeedCode.S_1000HZ, 333: self.SpeedCode.S_333HZ,
                    100: self.SpeedCode.S_100HZ, 30: self.SpeedCode.S_30HZ,
                    10: self.SpeedCode.S_10HZ, 0: self.SpeedCode.STOP
                }.items()}
                start_speed_hz = hz_map_rev.get(self._current_speed_code, 100)


            self.set_config(speed_hz=start_speed_hz, filter_hz=None, zero=None) # use current filter and zero state
        else:
            print("Streaming already started or configured to be active.")

    def stop_streaming(self):
        """Stops streaming by setting speed to 0."""
        if self.is_streaming or self._current_speed_code != self.SpeedCode.STOP.value:
            self.set_config(speed_hz=0, filter_hz=None, zero=None) # use current filter and zero state
        else:
            print("Streaming not active or already stopped.")

    def _calculate_data_checksum(self, frame_bytes_for_sum):
        """Calculates checksum for incoming data packets (sum of first 20 bytes)."""
        return sum(frame_bytes_for_sum) & 0xFFFF

    def _parse_frame(self, frame):
        if len(frame) != self.DATA_FRAME_LENGTH:
            print(f"Error: Frame length is {len(frame)}, expected {self.DATA_FRAME_LENGTH}")
            return None

        # C++ driver checks for a 4-byte header: 170, 7, 8, 16
        if frame[0:4] != self.DATA_FRAME_START_BYTES:
            print(f"Error: Frame does not start with {self.DATA_FRAME_START_BYTES.hex()}. Got: {frame[0:4].hex()}")
            return None

        # C++ driver reads 6 int16_t values (Fx,Fy,Fz,Tx,Ty,Tz) starting from frame[8]
        # IT MUST BE big-endian
        # frame[4:8] are not explicitly parsed in the C++ driver's readData. Could be counter/status.
        # frame[20:22] are assumed to be checksum here.
        try:
            # unpack the 6 force/torque values // 12 bytes, starting at index 8
            fx_raw, fy_raw, fz_raw, \
            tx_raw, ty_raw, tz_raw = struct.unpack('>hhhhhh', frame[8:20]) # Big-endian shorts

            # assume last 2 bytes is checksum (big-endian short)
            received_checksum, = struct.unpack('>H', frame[20:22])
            
            # bytes 4,5,6,7 are not used by C++ driver's readData for F/T values.
            # we can extract them if needed, e.g., as status or counter.
            # for now, let's just acknowledge them.
            unknown_bytes = frame[4:8]

        except struct.error as e:
            print(f"Struct unpack error: {e}. Frame: {frame.hex()}")
            return None

        calculated_checksum = self._calculate_data_checksum(frame[:20])
        if received_checksum != calculated_checksum:
            print(f"Data checksum mismatch! Received: {received_checksum:04X}, Calculated: {calculated_checksum:04X}. Frame: {frame.hex()}")
            # C++ driver doesn't show explicit data checksum validation, we will
            # waring or strict out None, let's keep it strict
            return None

        fx = fx_raw / self.fx_gain
        fy = fy_raw / self.fy_gain
        fz = fz_raw / self.fz_gain
        tx = tx_raw / self.tx_gain
        ty = ty_raw / self.ty_gain
        tz = tz_raw / self.tz_gain

        return {
            "Fx": fx, "Fy": fy, "Fz": fz,
            "Tx": tx, "Ty": ty, "Tz": tz,
            "status_unknown": unknown_bytes.hex(), # placeholder for unparsed bytes
            "raw": {
                "Fx": fx_raw, "Fy": fy_raw, "Fz": fz_raw,
                "Tx": tx_raw, "Ty": ty_raw, "Tz": tz_raw,
                "checksum_received": received_checksum,
                "checksum_calculated": calculated_checksum
            }
        }

    def read_data_packet(self):
        if not self.ser.is_open:
            print("Serial not open.")
            return None
        if not self.is_streaming:
            # let's allow reading if port is open, but warn if not explicitly streaming.
            if self._current_speed_code == self.SpeedCode.STOP.value:
                 # print("Warning: Attempting to read data while configured speed is STOP.")
                 pass # allow read attempt, might get old data or nothing, debug time

        bytes_to_read = self.ser.in_waiting
        if bytes_to_read > 0:
            data = self.ser.read(bytes_to_read)
            if data:
                self.buffer.extend(data)
        elif not self.buffer: # no data in waiting and no buffer, likely no data
            return None


        while True:
            # search for the 4-byte start sequence
            start_index = self.buffer.find(self.DATA_FRAME_START_BYTES)
            if start_index == -1:
                # if no start sequence found, keep only last few bytes to avoid unbounded buffer growth
                # in case of continuous garbage data.
                if len(self.buffer) > self.DATA_FRAME_LENGTH * 2: # keep a bit more than one frame
                    # print(f"No frame start found, trimming buffer from {len(self.buffer)} to {self.DATA_FRAME_LENGTH -1}")
                    self.buffer = self.buffer[-(self.DATA_FRAME_LENGTH -1):]
                return None # no complete frame start found atm

            # if a start sequence is found, but not enough bytes for a full frame yet
            if len(self.buffer) < start_index + self.DATA_FRAME_LENGTH:
                if start_index > 0: # Discard bytes before the found start_index
                    # print(f"Partial frame, discarding {start_index} bytes from buffer start.")
                    del self.buffer[:start_index]
                return None # not enough data for full frame

            # potential frame found
            potential_frame = self.buffer[start_index : start_index + self.DATA_FRAME_LENGTH]
            
            # try and parse?
            parsed_data = self._parse_frame(potential_frame)
            
            # remove the processed (or attempted!) frame from the buffer
            del self.buffer[start_index : start_index + self.DATA_FRAME_LENGTH]

            if parsed_data:
                return parsed_data
            else:
                # if parsing failed, loop again to find next potential frame start
                # the bad frame data is already removed from buffer!
                print("Frame parsing failed, trying next.")
                continue
        return None


    def zero_sensor(self):
        """Sends a command to zero the sensor readings."""
        print("Sending zero command to sensor.")
        # use current speed and filter, only change the zero flag!
        return self.set_config(speed_hz=None, filter_hz=None, zero=True)

    def clear_zero(self):
        """Sends a command to clear sensor zeroing/offsets."""
        print("Sending clear zero (un-zero) command to sensor.")
        return self.set_config(speed_hz=None, filter_hz=None, zero=False)

    # let's be the good guy
    def close(self):
        print("Closing OptoForce sensor.")
        if self.ser and self.ser.is_open:
            if self.is_streaming: # if it was streaming, send a stop command
                print("Stopping stream before closing...")
                self.stop_streaming()
                time.sleep(0.1) # give it a moment
            self.ser.close()
            print("Serial port closed.")

    def __enter__(self):
        # the __init__ already calls set_config, which starts streaming if speed > 0.
        # if it was initialized with speed=0, this will start it with the current/default non-zero speed.
        if not self.is_streaming:
             self.start_streaming()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

if __name__ == "__main__":
    SERIAL_PORT = '/dev/ttyACM0' # Adjust if needed
    
    # these gains are examples from C++ git, use actual gains for your sensor
    # the C++ driver would have a calArray = [1/FX_GAIN, 1/FY_GAIN, ...]
    FX_GAIN = 33.74
    FY_GAIN = 35.61
    FZ_GAIN = 10.00
    TX_GAIN = 100.0
    TY_GAIN = 100.0
    TZ_GAIN = 100.0

    sensor = None # define sensor here to ensure it's in scope for finally
    try:
        # demo initialize with C++ driver's typical defaults: 100Hz, 15Hz filter, zero on init
        sensor = OptoForceSensor(
            port=SERIAL_PORT,
            baudrate=1000000, # C++ driver uses 1Mbaud
            initial_speed_hz=100,
            initial_filter_hz=15,
            zero_on_init=True,
            fx_gain=FX_GAIN, fy_gain=FY_GAIN, fz_gain=FZ_GAIN,
            tx_gain=TX_GAIN, ty_gain=TY_GAIN, tz_gain=TZ_GAIN
        )

        print("Sensor streaming should be active. Press Ctrl+C to stop.")
        
        # example: Change configuration after initialization
        # time.sleep(2)
        # print("\nChanging to 30Hz speed, 50Hz filter, no zeroing...")
        # sensor.set_config(speed_hz=30, filter_hz=50, zero=False)
        # time.sleep(1)

        # example: Zero the sensor
        # print("\nZeroing sensor...")
        # sensor.zero_sensor()
        # time.sleep(1)


        packet_count = 0
        error_count = 0
        start_time = time.time()
        last_print_time = start_time

        while True:
            data_packet = sensor.read_data_packet()
            if data_packet:
                packet_count += 1
                # only print every Nth packet or every X seconds to avoid console flood
                if packet_count: # Print every 10th packet (% 10 == 0)
                    print(f"Fx: {data_packet['Fx']:.2f} N, Fy: {data_packet['Fy']:.2f} N, Fz: {data_packet['Fz']:.2f} N | "
                          f"Tx: {data_packet['Tx']:.3f} Nm, Ty: {data_packet['Ty']:.3f} Nm, Tz: {data_packet['Tz']:.3f} Nm | "
                          f"Unk: {data_packet['status_unknown']}")
            # else:
                # error_count +=1 # No packet or parse error

            # calculate and print rate periodically
            current_time = time.time()
            if current_time - last_print_time >= 5.0: # print conn stats every 5 seconds
                elapsed_time_total = current_time - start_time
                if elapsed_time_total > 0:
                    rate = packet_count / elapsed_time_total
                    print(f"--- Total {packet_count} packets. Avg Rate: {rate:.2f} Hz. Errors: {error_count} ---")
                last_print_time = current_time
                # reset error_count for the next interval if desired, or keep cumulative
            
            # small sleep to prevent busy-waiting if no data, adjust as needed
            # the serial timeout handles blocking, but this can reduce CPU if reads are very fast
            time.sleep(0.0005) # 0.5ms, for ~1000Hz, may need adjustment

    # and exceptions, might be some more of them but it is what it is
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        print(f"Please check if the port '{SERIAL_PORT}' is correct, the sensor is connected,")
        print("and no other program is using it. Also verify baudrate and permissions.")
    except KeyboardInterrupt:
        print("\nStopping script...")
    finally:
        if sensor:
            print("Closing sensor...")
            sensor.close()
        print("Script finished.")