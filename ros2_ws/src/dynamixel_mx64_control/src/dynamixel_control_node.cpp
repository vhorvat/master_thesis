#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_msgs/msg/string.hpp"

#include <string>
#include <mutex>
#include <atomic>
#include <algorithm> // For std::max
#include <sstream>   // For std::stringstream
#include <iomanip>   // For std::fixed, std::setprecision


#define ADDR_PRO_TORQUE_ENABLE        64
#define ADDR_PRO_GOAL_CURRENT         102
#define ADDR_PRO_PRESENT_CURRENT      126  // Data Length: 2 bytes
#define ADDR_PRO_OPERATING_MODE       11
#define ADDR_PRO_PRESENT_VELOCITY     128  // Data Length: 4 bytes
#define ADDR_PRO_PRESENT_POSITION     132  // Data Length: 4 bytes


#define PROTOCOL_VERSION              2.0

#define DXL_ID                        1
#define BAUDRATE                      1000000
#define DEVICENAME                    "/dev/ttyUSB0"

#define CURRENT_CONVERSION_FACTOR     4.5 // mA per unit

class DynamixelController : public rclcpp::Node
{
public:
    DynamixelController() : Node("dynamixel_controller"), desired_current_mA_(0.0), shutdown_requested_(false)
    {
        port_handler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
        packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (!port_handler_->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open port %s", DEVICENAME);
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully opened port %s", DEVICENAME);

        if (!port_handler_->setBaudRate(BAUDRATE)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate %d", BAUDRATE);
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully set baudrate %d", BAUDRATE);

        // Disable torque before changing operating mode
        uint8_t dxl_error = 0;
        int dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to disable torque: %s", packet_handler_->getTxRxResult(dxl_comm_result));
            rclcpp::shutdown();
            return;
        } else if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Dynamixel error on disabling torque: %s", packet_handler_->getRxPacketError(dxl_error));
            rclcpp::shutdown();
            return;
        }

        uint8_t operating_mode = 0;
        dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_OPERATING_MODE, operating_mode, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set operating mode: %s", packet_handler_->getTxRxResult(dxl_comm_result));
            rclcpp::shutdown();
            return;
        } else if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Dynamixel error on setting operating mode: %s", packet_handler_->getRxPacketError(dxl_error));
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully set operating mode to %d", operating_mode);

        // Enable torque
        dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable torque: %s", packet_handler_->getTxRxResult(dxl_comm_result));
            rclcpp::shutdown();
            return;
        } else if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Dynamixel error on enabling torque: %s", packet_handler_->getRxPacketError(dxl_error));
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully enabled torque.");

        update_goal_current(); // Initialize goal current

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/serial_esp_data", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                try {
                    float input_val = std::stof(msg->data);
                    // RCLCPP_INFO(this->get_logger(), "Received value: %.2f", input_val);
        
                    if (input_val >= 0.0f) { // Only process negative values
                        // set current to 0 for non-negative inputs:
                        // {
                        //     std::lock_guard<std::mutex> lock(current_mutex_);
                        //     desired_current_mA_ = 0.0f;
                        // }
                        // update_goal_current();
                        return;
                    }
        
                    float clamped_val = std::max(input_val, -50000.0f); // clamp input to [-30000, 0]
                    float scaled_val = 0.0f;



                    if (clamped_val >= -8000.0f) { 
                        // segment 1: clamped_val in [-10000, 0]
                        // maps (0, 250) and (-10000, -1)
                        scaled_val = 0.0251f * clamped_val + 250.0f;
                    } else if (clamped_val <= -12000.0f) { 
                        // segment 2: clamped_val in [-30000, -10000)
                        // maps (-10000, -1) and (-15000, -100), extrapolates to -30000
                        scaled_val = 0.0118f * clamped_val + 197.0f;
                    }
        
                    {
                        std::lock_guard<std::mutex> lock(current_mutex_);
                        desired_current_mA_ = (-1) * scaled_val; // Apply the negation
                    }

                    RCLCPP_INFO(this->get_logger(), "Input: %.2f, Clamped: %.2f, Scaled (intermediate): %.2f, Desired Current (mA): %.2f",
                                input_val, clamped_val, scaled_val, desired_current_mA_);

                    update_goal_current();
                } catch (const std::invalid_argument &e) {
                    RCLCPP_WARN(this->get_logger(), "Invalid numeric data received on /serial_esp_data: '%s' (%s)", msg->data.c_str(), e.what());
                } catch (const std::out_of_range &e) {
                    RCLCPP_WARN(this->get_logger(), "Numeric data out of range on /serial_esp_data: '%s' (%s)", msg->data.c_str(), e.what());
                }
            });


        motor_status_publisher_ = this->create_publisher<std_msgs::msg::String>("/motor_status", 10);


        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&DynamixelController::publish_motor_status, this)
        );
        RCLCPP_INFO(this->get_logger(), "DynamixelController node started successfully.");
    }

    ~DynamixelController()
    {
        shutdown_requested_.store(true);

        if (status_timer_) {
            status_timer_->cancel(); 
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));


        RCLCPP_INFO(this->get_logger(), "Attempting to disable torque on shutdown...");
        uint8_t dxl_error = 0;
        int dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "Failed to disable torque on shutdown: %s", packet_handler_->getTxRxResult(dxl_comm_result));
        } else if (dxl_error != 0) {
            RCLCPP_WARN(this->get_logger(), "Dynamixel error on disabling torque during shutdown: %s", packet_handler_->getRxPacketError(dxl_error));
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully disabled torque on shutdown.");
        }

        if (port_handler_ != nullptr) {
            port_handler_->closePort();
            RCLCPP_INFO(this->get_logger(), "Port closed.");
        }
    }

private:
    void publish_motor_status()
    {
        if (shutdown_requested_.load()) {
            return;
        }

        uint8_t dxl_error = 0;
        int dxl_comm_result;

        int32_t present_position_raw = 0;
        int32_t present_velocity_raw = 0;
        int16_t present_current_raw = 0;

        std::string pos_str = "N/A";
        std::string vel_str = "N/A";
        std::string cur_str = "N/A";

        // present position
        dxl_comm_result = packet_handler_->read4ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&present_position_raw, &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0) {
            pos_str = std::to_string(present_position_raw);
        } else {
            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_DEBUG(this->get_logger(), "Comm fail (pos): %s", packet_handler_->getTxRxResult(dxl_comm_result));
            } else {
                RCLCPP_DEBUG(this->get_logger(), "DXL error (pos): %s", packet_handler_->getRxPacketError(dxl_error));
            }
        }

        // present velocity
        dxl_error = 0; 
        dxl_comm_result = packet_handler_->read4ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_PRESENT_VELOCITY, (uint32_t*)&present_velocity_raw, &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0) {
            vel_str = std::to_string(present_velocity_raw);
        } else {
            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_DEBUG(this->get_logger(), "Comm fail (vel): %s", packet_handler_->getTxRxResult(dxl_comm_result));
            } else {
                RCLCPP_DEBUG(this->get_logger(), "DXL error (vel): %s", packet_handler_->getRxPacketError(dxl_error));
            }
        }

        // present current
        dxl_error = 0; 
        dxl_comm_result = packet_handler_->read2ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_PRESENT_CURRENT, (uint16_t*)&present_current_raw, &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0) {
            float current_mA = static_cast<float>(present_current_raw) * CURRENT_CONVERSION_FACTOR;
            std::stringstream cur_ss;
            cur_ss << std::fixed << std::setprecision(2) << current_mA;
            cur_str = cur_ss.str();
        } else {
            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_DEBUG(this->get_logger(), "Comm fail (cur): %s", packet_handler_->getTxRxResult(dxl_comm_result));
            } else {
                RCLCPP_DEBUG(this->get_logger(), "DXL error (cur): %s", packet_handler_->getRxPacketError(dxl_error));
            }
        }

        std_msgs::msg::String msg;
        std::stringstream ss;
        ss << "Position: " << pos_str
           << ", Velocity: " << vel_str
           << ", Current_mA: " << cur_str;
        msg.data = ss.str();

        motor_status_publisher_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "Published motor status: %s", msg.data.c_str()); // Uncomment for verbose logging
    }

    void update_goal_current()
    {
        if (shutdown_requested_.load()) {
            return;
        }
        
        float current_to_set_mA;
        {
            std::lock_guard<std::mutex> lock(current_mutex_);
            current_to_set_mA = desired_current_mA_;
        }

        int16_t goal_current_raw = static_cast<int16_t>(current_to_set_mA / CURRENT_CONVERSION_FACTOR);

        uint8_t dxl_error = 0;
        int dxl_comm_result = packet_handler_->write2ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_GOAL_CURRENT, goal_current_raw, &dxl_error);
        
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "Failed to set goal current (%d raw / %.2f mA): %s",
                        goal_current_raw, current_to_set_mA, packet_handler_->getTxRxResult(dxl_comm_result));
        } else if (dxl_error != 0) {
            RCLCPP_WARN(this->get_logger(), "Dynamixel error on setting goal current (%d raw / %.2f mA): %s",
                        goal_current_raw, current_to_set_mA, packet_handler_->getRxPacketError(dxl_error));
        } else {
            // RCLCPP_INFO(this->get_logger(), "Successfully set goal current to %d (%.2f mA)", goal_current_raw, current_to_set_mA);
        }
    }

    dynamixel::PortHandler *port_handler_;
    dynamixel::PacketHandler *packet_handler_;
    
    rclcpp::TimerBase::SharedPtr status_timer_; // Renamed from timer_ for clarity
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_status_publisher_; // New publisher

    std::mutex current_mutex_;
    std::atomic<bool> shutdown_requested_;
    float desired_current_mA_; // this is the value in mA
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto dynamixel_controller_node = std::make_shared<DynamixelController>();
    
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(dynamixel_controller_node);
    
    RCLCPP_INFO(dynamixel_controller_node->get_logger(), "Spinning DynamixelController node...");
    exec.spin();
    
    // rclcpp::spin(dynamixel_controller_node); // alternative if not using custom executor logic

    RCLCPP_INFO(dynamixel_controller_node->get_logger(), "Shutting down DynamixelController node.");
    rclcpp::shutdown();
    return 0;
}