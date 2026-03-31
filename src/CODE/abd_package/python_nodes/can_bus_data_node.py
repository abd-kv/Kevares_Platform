#Authored by Bobby Robin
#DoneTheRightWay

import rclpy
from rclpy.node import Node
import can
import json
from std_msgs.msg import String

BITRATE = 500000
CAN_INTERFACE = "can0"

class CanBusDataNode(Node):
    def __init__(self):
        super().__init__('can_bus_data_node')

        self._setup_can()

        self.publisher = self.create_publisher(String, 'can_bus_data_node', 10)
        self.timer = self.create_timer(0.01, self._timer_callback)

    def _setup_can(self) -> None:
        #Setting up CAN connection
        try:
            self.bus = can.interface.Bus(bustype="socketcan", channel=CAN_INTERFACE, bitrate=BITRATE)
        except OSError as error: 
            self.get_logger().error(f"CAN interface initialization failed: {error}")
            raise RuntimeError("CAN interface initialization failed")
        
    def _timer_callback(self) -> None:
        try:
            #recv is a blocking method. Returns message or None.
            raw_message = self.bus.recv(timeout=0.01)
            if(raw_message):
                message = self._process_message(raw_message)
                self._publish_data(message)
        except Exception as error:
            self.get_logger().error(f"Error receiving CAN message: {error}")

    def _process_message(self, message: String) -> dict:
        raw_data: String = message.data
        output = {}

        if(message.arbitration_id == 0x261):
            output = {
                #motor low-speed data
                "id": "0x261",
                
                # Motor driver/controller voltage (0.1V resolution)
                # Usage: Monitor battery/power supply health
                # Example: 240 → 24.0V (normal range: 24-26.8V per manual)
                "drive_voltage": int.from_bytes(raw_data[0:2], byteorder='big', signed=False) * 0.1,
                
                # Motor driver/controller temperature (°C) -> Location: inside electronic speed controller (ESC)
                # Usage: Overheat protection (shutdown if >80°C)
                # Example: 50 → 50°C
                "drive_temperature": int.from_bytes(raw_data[2:4], byteorder='big'),
                
                # Motor temperature (°C)
                # Usage: Prevent motor damage from overheating
                # Example: 40 → 40°C
                "motor_temperature": raw_data[4],  # Single byte (signed)
                
                # Drive status (Boolean value)
                # Usage: Fault detection and diagnostics
                # Bitmask values (from manual Table 3.8):
                # - bit0: Voltage too low 
                # - bit1: Motor overheated
                # - bit2: Drive overcurrent
                # - bit3: Drive overheated
                # - bit4: Sensor error
                # - bit5: General drive error
                "drive_status": {
                    "raw": raw_data[5],
                    "voltage_low": bool(raw_data[5] & 0b00000001),
                    "motor_overheat": bool(raw_data[5] & 0b00000010),
                    "overcurrent": bool(raw_data[5] & 0b00000100),
                    "drive_overheat": bool(raw_data[5] & 0b00001000),
                    "sensor_error": bool(raw_data[5] & 0b00010000),
                    "drive_error": bool(raw_data[5] & 0b00100000)
                }
            }

        elif(message.arbitration_id == 0x221):
            #Shows the angle of the wheel in rad.
            steering_angle = int.from_bytes(raw_data[6:8], byteorder='big')/1000 # rad
            output = {
            "id": "0x221",
            "steering_angle": steering_angle
            }
    

        #0x251 is the motor high-speed (50Hz) feedback data of the robot. Critical data for drive control.
        elif(message.arbitration_id == 0x251):
            output = {
                #motor high-speed data
                "id": "0x251", 
                #RPM of the motor
                "motor_speed": int.from_bytes(raw_data[0:2], byteorder='big'), 
                #How much current motor is drawing instantaneouly. Useful for detecting stalling of motor. (0.1A resolution)
                "motor_current": int.from_bytes(raw_data[2:4], byteorder='big') * 0.1, 
                #It is the encoder pulse count. Useful for odometry and possibly for navigation stack.
                "motor_position": int.from_bytes(raw_data[4:8], byteorder='big')
            }

        elif (message.arbitration_id == 0x361):
            output = {
                #Battery Management System Data.
                "id": "0x361",
                
                # State of Charge (0-100%)
                # Usage: Battery level indicator
                # Example: 75 → 75% charged
                "soc": raw_data[0],  # unsigned int8 (0-100%)
                
                # State of Health (0-100%)
                # Usage: Long-term battery degradation monitoring
                # Example: 90 → 90% of original capacity
                "soh": raw_data[1],  # unsigned int8 (0-100%)
                
                # Battery Voltage (0.01V resolution)
                # Usage: Detect undervoltage/overvoltage conditions
                # Example: 2520 → 25.20V (normal range: 24.0-26.8V per manual)
                "voltage": int.from_bytes(raw_data[2:4], byteorder='big') * 0.01,
                
                # Battery Current (0.1A resolution, signed)
                # Usage: Charge/discharge rate monitoring
                # Example: -50 → -5.0A (discharging), 30 → 3.0A (charging)
                "current": int.from_bytes(raw_data[4:6], byteorder='big') * 0.1,
                
                # Battery Temperature (0.1°C resolution)
                # Usage: Thermal safety (Li-ion batteries degrade >45°C)
                # Example: 320 → 32.0°C
                "temperature": int.from_bytes(raw_data[6:8], byteorder='big') * 0.1
            }
        return output

    def can_bus_shutdown(self) -> None:
        self.bus.shutdown()

    def _publish_data(self, data: dict) -> None:
        #Convert to json...
        msg = String(data=json.dumps(data))
        self.publisher.publish(msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = CanBusDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.can_bus_shutdown()
        node.destroy_node()
        if rclpy.ok():
            node.get_logger().info("Node is shutting down")
            rclpy.shutdown()
            
        
if __name__ == "__main__":
    main()