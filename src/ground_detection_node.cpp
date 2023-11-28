#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ground_detection/ground_detection.hpp"
#include <Eigen/Dense>

class PointCloudSubscriberNode : public rclcpp::Node {
public:
    PointCloudSubscriberNode() : Node("point_cloud_subscriber") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "kitti/velo/pointcloud", 10, std::bind(&PointCloudSubscriberNode::PointCloudCallback, this, std::placeholders::_1)
        );

        publisher1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_points", 10);
        publisher2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("non_ground_points", 10);

        config.cellSizeX = 1;
        config.cellSizeY = 1;
        config.cellSizeZ = 0.5;

        ground_detection = new PointCloudGrid(config);

    }

private:

    PointCloudGrid* ground_detection;

    GridConfig config;

    void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

      sensor_msgs::msg::PointCloud2::SharedPtr gp = std::make_shared<sensor_msgs::msg::PointCloud2>();
      sensor_msgs::msg::PointCloud2::SharedPtr ngp = std::make_shared<sensor_msgs::msg::PointCloud2>();

        // Convert the ROS 2 PointCloud2 message to a PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);
        Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0);

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl_cloud);
        ground_detection->setInputCloud(pcl_cloud_ptr, orientation);


        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points = ground_detection->extractGroundPoints();
        pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_points = ground_detection->extractNonGroundPoints();

        // Convert PCL PointCloud to ROS PointCloud2 message
        pcl::toROSMsg(*ground_points, *gp);
        pcl::toROSMsg(*non_ground_points, *ngp);

        // Set the frame ID and timestamp
        gp->header.frame_id = "velo_link";
        gp->header.stamp = this->now();
        // Set the frame ID and timestamp
        ngp->header.frame_id = "velo_link";
        ngp->header.stamp = this->now();

        // Publish the message
        publisher1_->publish(*gp);
        publisher2_->publish(*ngp);

    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher1_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher2_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudSubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
