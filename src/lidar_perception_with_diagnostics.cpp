#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"

// Dummy Lidar Publisher Class
class DummyLidarPublisher : public rclcpp::Node
{
public:
    DummyLidarPublisher() : Node("dummy_lidar_publisher")
    {
        RCLCPP_INFO(this->get_logger(), "Dummy Lidar Publisher initialized.");
        lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_points", 10);
        // triggers data publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&DummyLidarPublisher::publishLidarData, this));
    }

private:
    void publishLidarData()
    {
        auto msg = sensor_msgs::msg::PointCloud2();
        msg.header.stamp = this->now();
        msg.header.frame_id = "lidar_frame";
        msg.height = 1;
        msg.width = 1;
        msg.fields.resize(1);
        msg.fields[0].name = "x";
        msg.fields[0].offset = 0;
        msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[0].count = 1;
        msg.is_bigendian = false;
        msg.point_step = 4;
        msg.row_step = msg.point_step * msg.width;
        msg.data.resize(msg.row_step * msg.height);
        msg.is_dense = true;

        lidar_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published dummy PointCloud2 message.");
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// Lidar Perception Node Class
class LidarPerceptionNode : public rclcpp::Node
{
public:
    LidarPerceptionNode()
        : Node("lidar_perception_node"),
          diagnostic_updater_(this->get_node_base_interface(), this->get_node_clock_interface(),
                              this->get_node_logging_interface(),
                              this->get_node_parameters_interface(),
                              this->get_node_timers_interface(), this->get_node_topics_interface())
    {
        // Initialize subscriber and publisher
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar_points", 10, std::bind(&LidarPerceptionNode::lidarCallback, this, std::placeholders::_1));

        bbox_pub_ = this->create_publisher<std_msgs::msg::String>("/bounding_boxes", 10);

        diagnostic_updater_.setHardwareID("lidar_perception_node");

        // Diagnostics for the lidar subscription with custom thresholds
        diag_pub_input = std::make_shared<diagnostic_updater::TopicDiagnostic>(
            "/lidar_points", diagnostic_updater_,
            diagnostic_updater::FrequencyStatusParam(&min_freq_lidar_, &max_freq_lidar_, 0.1, 10),
            diagnostic_updater::TimeStampStatusParam(-1, 1));

        // Diagnostics for the bounding box publisher with custom thresholds
        diag_pub_output = std::make_shared<diagnostic_updater::TopicDiagnostic>(
            "/bounding_boxes", diagnostic_updater_,
            diagnostic_updater::FrequencyStatusParam(&min_freq_bbox_, &max_freq_bbox_, 0.1, 10),
            diagnostic_updater::TimeStampStatusParam(-1, 1));
    }

private:
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "PointCloud2 message received.");
        diag_pub_input->tick(msg->header.stamp); // Update diagnostics for lidar points subscriber

        auto bbox_msg = std_msgs::msg::String();
        bbox_msg.data = "Bounding Box Data";

        bbox_pub_->publish(bbox_msg);
        diag_pub_output->tick(this->now());  // Update diagnostics for bounding box publisher
        RCLCPP_INFO(this->get_logger(), "Bounding box published.");
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bbox_pub_;
    diagnostic_updater::Updater diagnostic_updater_;
    std::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_pub_input;
    std::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_pub_output;

    // Set custom frequency thresholds for diagnostics
    double min_freq_lidar_ = 8.0;  // Minimum acceptable frequency for lidar points (1 Hz)
    double max_freq_lidar_ = 30;  // Maximum frequency threshold to avoid warnings for lidar points

    double min_freq_bbox_ = 8.0;  // Minimum acceptable frequency for bounding boxes (1 Hz)
    double max_freq_bbox_ = 30; // Maximum frequency threshold for bounding boxes
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto dummy_lidar_node = std::make_shared<DummyLidarPublisher>();
    auto lidar_perception_node = std::make_shared<LidarPerceptionNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(dummy_lidar_node);
    executor.add_node(lidar_perception_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
