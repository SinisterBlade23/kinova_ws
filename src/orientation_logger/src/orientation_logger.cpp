    #include "rclcpp/rclcpp.hpp"
    #include "geometry_msgs/msg/pose_stamped.hpp"
    #include <fstream>
    #include <string>

    class OrientationLogger : public rclcpp::Node
    {
    public:
        OrientationLogger() : Node("orientation_logger")
        {
            csv_file_.open("orientation_log.csv");
            csv_file_ << "timestamp_sec,topic,x,y,z,w\n";
            csv_file_ << std::fixed << std::setprecision(6);

            target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/cartesian_motion_controller/target_frame", 10,
                std::bind(&OrientationLogger::targetCallback, this, std::placeholders::_1));

            current_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/cartesian_motion_controller/current_pose", 10,
                std::bind(&OrientationLogger::currentCallback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Logging to orientation_log.csv");
        }

        ~OrientationLogger()
        {
            csv_file_.close();
        }

    private:
        void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            writeRow("target", msg);
        }

        void currentCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            writeRow("current", msg);
        }

        void writeRow(const std::string & topic_label, const geometry_msgs::msg::PoseStamped::SharedPtr & msg)
        {
            double t = this->now().seconds();
            csv_file_ << t << ","
                    << topic_label << ","
                    << msg->pose.orientation.x << ","
                    << msg->pose.orientation.y << ","
                    << msg->pose.orientation.z << ","
                    << msg->pose.orientation.w << "\n";
            csv_file_.flush();
        }

        std::ofstream csv_file_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_sub_;
    };

    int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<OrientationLogger>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }