#include <rclcpp/rclcpp.hpp>
#include "SCServo.h"

// Define the Angle Limit EPROM Registers
#define SMS_STS_MIN_ANGLE_LIMIT_L 9
#define SMS_STS_MAX_ANGLE_LIMIT_L 11

class RemoveLimits : public rclcpp::Node
{
public:
    RemoveLimits() : Node("remove_limits")
    {
        this->declare_parameter<int>("id", 1);
        this->declare_parameter<std::string>("device_port", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 1000000);

        id_ = static_cast<uint8_t>(this->get_parameter("id").as_int());
        device_port_ = this->get_parameter("device_port").as_string();
        baud_rate_   = this->get_parameter("baud_rate").as_int();

        if (!remove_limits())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to remove limits for servo ID %d", id_);
        }
    }

private:
    SMS_STS sm_st;
    uint8_t id_;
    std::string device_port_;
    int baud_rate_;

    bool remove_limits()
    {
        if (!sm_st.begin(baud_rate_, device_port_.c_str())) 
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot connect to port %s", device_port_.c_str());
            return false;
        }
        
        // 1. Unlock the EPROM to allow modifying protected memory
        if (!sm_st.unLockEprom(id_)) 
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot unlock EPROM for servo with ID %d", id_);
            sm_st.end();
            return false;
        }
        
        // 2. Write 0 to Min Angle Limit (Disables the lower limit)
        if (!sm_st.writeWord(id_, SMS_STS_MIN_ANGLE_LIMIT_L, 0))
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot write Min Angle Limit for ID %d", id_);
            sm_st.LockEprom(id_);
            sm_st.end();
            return false;
        }
        
        // 3. Write 0 to Max Angle Limit (Disables the upper limit)
        if (!sm_st.writeWord(id_, SMS_STS_MAX_ANGLE_LIMIT_L, 0))
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot write Max Angle Limit for ID %d", id_);
            sm_st.LockEprom(id_);
            sm_st.end();
            return false;
        }
        
        // 4. Lock the EPROM
        sm_st.LockEprom(id_);
        sm_st.end();
        
        RCLCPP_INFO(this->get_logger(), "Successfully removed angle limits for ID %d", id_);
        return true;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin_some(std::make_shared<RemoveLimits>());
    rclcpp::shutdown();
    return 0;
}