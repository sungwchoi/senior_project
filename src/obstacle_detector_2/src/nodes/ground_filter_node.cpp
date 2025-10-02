#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

class GroundFilterNode : public rclcpp::Node
{
public:
    GroundFilterNode()
    : Node("ground_filter_node")
    {
        // 파라미터 선언
        this->declare_parameter<std::string>("input_topic", "/velodyne_points");
        this->declare_parameter<std::string>("output_topic", "/velodyne_points_filtered");
        this->declare_parameter<std::string>("filter_field_name", "z");
        this->declare_parameter<double>("filter_limit_min", 0.1);
        this->declare_parameter<double>("filter_limit_max", 1.25);

        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        filter_field_ = this->get_parameter("filter_field_name").as_string();
        filter_min_ = this->get_parameter("filter_limit_min").as_double();
        filter_max_ = this->get_parameter("filter_limit_max").as_double();

        // 퍼블리셔/서브스크라이버
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10,
            std::bind(&GroundFilterNode::pointCloudCallback, this, std::placeholders::_1));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*msg, *cloud);

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName(filter_field_);
        pass.setFilterLimits(filter_min_, filter_max_);
        pass.filter(*cloud_filtered);

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header = msg->header;

        publisher_->publish(output_msg);
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string filter_field_;
    double filter_min_;
    double filter_max_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundFilterNode>());
    rclcpp::shutdown();
    return 0;
}
