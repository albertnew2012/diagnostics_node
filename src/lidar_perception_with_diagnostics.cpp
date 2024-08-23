#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

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
            "/lidar_points", 10,
            std::bind(&LidarPerceptionNode::lidarCallback, this, std::placeholders::_1));

        bbox_pub_ = this->create_publisher<std_msgs::msg::String>("/bounding_boxes", 10);

        // Initialize diagnostic updater
        diagnostic_updater_.setHardwareID("lidar_perception_node");
        diagnostic_updater_.add("Lidar Perception Status", this,
                                &LidarPerceptionNode::diagnosticsCallback);

        // Initialize diagnostic publisher for the point cloud topic
        diag_pub_ = std::make_shared<diagnostic_updater::TopicDiagnostic>(
            "/lidar_points", diagnostic_updater_,
            diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_, 0.1, 10),
            diagnostic_updater::TimeStampStatusParam(-1, 1));
    }

  private:
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "PointCloud2 message received.");
        diag_pub_->tick(msg->header.stamp);

        auto bbox_msg = std_msgs::msg::String();
        bbox_msg.data = "Bounding Box Data"; // Replace with actual bounding box data

        bbox_pub_->publish(bbox_msg);
        RCLCPP_INFO(this->get_logger(), "Bounding box published.");
    }

    void diagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        // Here you can check the internal state of your node and update diagnostics
        if (bbox_pub_->get_subscription_count() > 0)
        {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                         "Lidar Perception Node is operating normally.");
        }
        else
        {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                         "No subscribers to bounding box topic.");
        }

        // Add additional key-value pairs to the diagnostics message if necessary
        stat.add("Bounding Box Publisher Count", bbox_pub_->get_subscription_count());
        // You can add more diagnostic checks here
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bbox_pub_;
    diagnostic_updater::Updater diagnostic_updater_;
    std::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_pub_;

    // Frequency parameters (you may need to adjust these based on your application)
    double min_freq_ = 1.0;
    double max_freq_ = 10.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarPerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
