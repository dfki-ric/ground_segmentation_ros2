#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include <Eigen/Dense>
#include <memory>

#include <pointcloud_obstacle_detection/ground_detection.hpp>

using namespace pointcloud_obstacle_detection;


class PointCloudSubscriberNode : public rclcpp::Node {
public:
    PointCloudSubscriberNode() : Node("point_cloud_subscriber") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "kitti/velo/pointcloud", 10, std::bind(&PointCloudSubscriberNode::PointCloudCallback, this, std::placeholders::_1)
        );

        publisher1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_points", 10);
        publisher2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("non_ground_points", 10);

        //config.radialCellSize = 2;
        //config.angularCellSize = 0.785398;
        //config.cellSizeZ = 1;

        ground_detection = std::make_unique<PointCloudGrid>(config);

    }

private:
    std::unique_ptr<PointCloudGrid> ground_detection;
    GridConfig config;
    void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::msg::PointCloud2::SharedPtr gp = std::make_shared<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::msg::PointCloud2::SharedPtr ngp = std::make_shared<sensor_msgs::msg::PointCloud2>();
        // Convert the ROS 2 PointCloud2 message to a PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);
        Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0);

        CloudXYZ pcl_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl_cloud);

        std::cout << "Input Cloud: " << pcl_cloud_ptr->points.size() << std::endl;

        ground_detection->setInputCloud(pcl_cloud_ptr, orientation);


        CloudPair result = ground_detection->segmentPoints();
        CloudXYZ ground_points = result.first;
        CloudXYZ non_ground_points = result.second;

        std::cout << "GN Cloud: " << result.first->points.size() << std::endl;
        std::cout << "NG Cloud: " << result.second->points.size() << std::endl;

        // Convert PCL PointCloud to ROS PointCloud2 message
        pcl::toROSMsg(*ground_points, *gp);
        pcl::toROSMsg(*non_ground_points, *ngp);

        // Set the frame ID and timestamp
        gp->header.frame_id = "husky/base_link/front_laser";
        gp->header.stamp = this->now();
        // Set the frame ID and timestamp
        ngp->header.frame_id = "husky/base_link/front_laser";
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
