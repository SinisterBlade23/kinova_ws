#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <libserial/SerialPort.h>

using namespace std::chrono_literals;
using namespace LibSerial;

class orientation_pubsub: public rclcpp::Node
{
    public:
        orientation_pubsub(): Node("orientation_pubsub")
        {

            serial_port_.Open("/dev/ttyUSB0");
            serial_port_.SetBaudRate(BaudRate::BAUD_115200);
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            serial_port_.SetParity(Parity::PARITY_NONE);
            serial_port_.SetStopBits(StopBits::STOP_BITS_1);


            publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/cartesian_motion_controller/target_frame",10);

            timer_ = this->create_wall_timer(0.001s,std::bind(&orientation_pubsub::publishQuaternions,this));
        }
    private:   

    void publishQuaternions()
    {
        std::string line;
        try
        {
            serial_port_.ReadLine(line, '\n', 5);
        }
        catch (const LibSerial::ReadTimeout&)
        {
            return;
        }

        std::string working = line;
        size_t pos;

        pos = working.find(',');
        float qi = std::stof(working.substr(0, pos));
        working.erase(0, pos + 1);

        pos = working.find(',');
        float qj = std::stof(working.substr(0, pos));
        working.erase(0, pos + 1);

        pos = working.find(',');
        float qk = std::stof(working.substr(0, pos));
        working.erase(0, pos + 1);

        pos = working.find(',');
        float qr = std::stof(working.substr(0, pos));
        working.erase(0, pos + 1);

        int fsr = std::stoi(working);
        (void)fsr;

        // Fixed mounting-correction offset (right-multiplied)
        const float ox = -0.7071068f;
        const float oy = 0.0f;
        const float oz = -0.7071068f;
        const float ow = 0.0f;

        // q_target = q_imu_current * q_offset  (Hamilton product, offset on the RIGHT)
        float w = qr*ow - qi*ox - qj*oy - qk*oz;
        float x = qr*ox + qi*ow + qj*oz - qk*oy;
        float y = qr*oy - qi*oz + qj*ow + qk*ox;
        float z = qr*oz + qi*oy - qj*ox + qk*ow;

        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "base_link";
        msg.header.stamp = this->now();
        msg.pose.position.x = 0.002;
        msg.pose.position.y = -0.248;
        msg.pose.position.z = 0.508;
        msg.pose.orientation.x = x;
        msg.pose.orientation.y = y;
        msg.pose.orientation.z = z;
        msg.pose.orientation.w = w;

        publisher_->publish(msg);
    }

    SerialPort serial_port_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};



int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<orientation_pubsub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}