#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_msgs/msg/string.hpp"

#include <string>
#include <mutex>
#include <atomic>
#include <algorithm>

#define ADDR_PRO_TORQUE_ENABLE        64
#define ADDR_PRO_GOAL_CURRENT         102
#define ADDR_PRO_PRESENT_CURRENT      126
#define ADDR_PRO_OPERATING_MODE       11

#define PROTOCOL_VERSION              2.0

#define DXL_ID                        1
#define BAUDRATE                      1000000
#define DEVICENAME                    "/dev/ttyUSB0"

#define CURRENT_CONVERSION_FACTOR     4.5 // from dynamixel wizard, why? /SDK says 4.5

class DynamixelController : public rclcpp::Node
{
public:
    DynamixelController() : Node("dynamixel_controller"), desired_current_mA_(0.0), shutdown_requested_(false)
    {
        port_handler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
        packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (!port_handler_->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open port");
            rclcpp::shutdown();
        }

        if (!port_handler_->setBaudRate(BAUDRATE)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate");
            rclcpp::shutdown();
        }

        uint8_t torque_enable = 0; 
        uint8_t ret = 0;
        int dxl_ret = packet_handler_->write1ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_TORQUE_ENABLE, torque_enable, &ret);
        if (dxl_ret != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to disable torque");
            rclcpp::shutdown();
        }

        uint8_t operating_mode = 0;
        dxl_ret = packet_handler_->write1ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_OPERATING_MODE, operating_mode, &ret);
        if (dxl_ret != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set operating mode");
            rclcpp::shutdown();
        }

        torque_enable = 1;
        dxl_ret = packet_handler_->write1ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_TORQUE_ENABLE, torque_enable, &ret);
        if (dxl_ret != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable torque");
            rclcpp::shutdown();
        }

        update_goal_current();

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/serial_esp_data", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                try {
                    float input_val = std::stof(msg->data);
                    RCLCPP_INFO(this->get_logger(), "Received value: %.2f", input_val);

                    float clamped_val = std::max(std::min(input_val, 200000.0f), -200000.0f);
                    float scaled_val = (clamped_val / 200000.0f) * 1000.0f;

                    {
                        std::lock_guard<std::mutex> lock(current_mutex_);
                        desired_current_mA_ = scaled_val;
                    }

                    RCLCPP_INFO(this->get_logger(), "Scaled current (mA): %.2f", scaled_val);

                    update_goal_current();
                } catch (const std::exception &e) {
                    RCLCPP_WARN(this->get_logger(), "Invalid data received on /serial_esp_data: '%s'", msg->data.c_str());
                }
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&DynamixelController::check_current, this)
        );
    }

    ~DynamixelController()
    {
        shutdown_requested_ = true;

        uint8_t torque_enable = 0;
        uint8_t ret = 0;
        int dxl_ret = packet_handler_->write1ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_TORQUE_ENABLE, torque_enable, &ret);
        if (dxl_ret != COMM_SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "Failed to disable torque on shutdown");
        }
    }

private:
    void check_current()
    {
        int16_t present_current = 0;
        uint8_t ret = 0;
        int dxl_ret = packet_handler_->read2ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_PRESENT_CURRENT, (uint16_t*)&present_current, &ret);
        if (dxl_ret != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read present current");
            rclcpp::shutdown();
        }

        float current_mA = present_current * CURRENT_CONVERSION_FACTOR;
        // RCLCPP_INFO(this->get_logger(), "Current: %.2f mA", current_mA);
    }

    void update_goal_current()
    {
        int goal_current_raw = static_cast<int>(desired_current_mA_ / CURRENT_CONVERSION_FACTOR);

        uint8_t ret = 0;
        int dxl_ret = packet_handler_->write2ByteTxRx(port_handler_, DXL_ID, ADDR_PRO_GOAL_CURRENT, goal_current_raw, &ret);
        if (dxl_ret != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set goal current");
        }
    }

    dynamixel::PortHandler *port_handler_;
    dynamixel::PacketHandler *packet_handler_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::mutex current_mutex_;
    std::atomic<bool> shutdown_requested_;
    float desired_current_mA_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto dynamixel_controller = std::make_shared<DynamixelController>();
    exec.add_node(dynamixel_controller);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
