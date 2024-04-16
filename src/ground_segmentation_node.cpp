#include "rclcpp/rclcpp.hpp"
#include <pointcloud_obstacle_detection/utils.hpp>

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/impl/transforms.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <pointcloud_obstacle_detection/ground_detection.hpp>

#include <Eigen/Dense>
#include <memory>
#include <chrono>

using namespace pointcloud_obstacle_detection;
using PointType = PointXYZILID;

class GroundSegmentatioNode : public rclcpp::Node {
public:
    GroundSegmentatioNode(rclcpp::NodeOptions options) : Node("ground_segmentation",options) {

        subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ground_segmentation/input", 10, std::bind(&GroundSegmentatioNode::PointCloudCallback, this, std::placeholders::_1)
        );

        publisher_ground_points = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/ground_points", 10);
        publisher_obstacle_points = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/obstacle_points", 10);
        publisher_raw_ground_points = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/filtered_input", 10);

        double radialCellSize = this->get_parameter("radialCellSize").as_double();
        double angularCellSize = this->get_parameter("angularCellSize").as_double();
        double cellSizeX = this->get_parameter("cellSizeX").as_double();
        double cellSizeY = this->get_parameter("cellSizeY").as_double();
        double cellSizeZ = this->get_parameter("cellSizeZ").as_double();
        double gridmaxX = this->get_parameter("gridmaxX").as_double();
        double gridmaxY = this->get_parameter("gridmaxY").as_double();
        double gridmaxZ = this->get_parameter("gridmaxZ").as_double();
        double startCellDistanceThreshold = this->get_parameter("startCellDistanceThreshold").as_double();
        double slopeThresholdDegrees = this->get_parameter("slopeThresholdDegrees").as_double();
        double groundInlierThreshold = this->get_parameter("groundInlierThreshold").as_double();
        double neighborsIndexThreshold = this->get_parameter("neighborsIndexThreshold").as_int();
        double minPoints = this->get_parameter("minPoints").as_int();
        double ransac_iterations = this->get_parameter("ransac_iterations").as_int();
        int grid_type = this->get_parameter("grid_type").as_int();
        bool returnGroundPoints = this->get_parameter("returnGroundPoints").as_bool();

        pre_processor_config.radialCellSize = radialCellSize;
        pre_processor_config.angularCellSize = angularCellSize;
        pre_processor_config.cellSizeX = cellSizeX;
        pre_processor_config.cellSizeY = cellSizeY;
        pre_processor_config.cellSizeZ = cellSizeZ;
        pre_processor_config.maxX = gridmaxX;
        pre_processor_config.maxY = gridmaxY;
        pre_processor_config.maxZ = gridmaxZ;
        pre_processor_config.startCellDistanceThreshold = startCellDistanceThreshold;
        pre_processor_config.slopeThresholdDegrees = slopeThresholdDegrees;
        pre_processor_config.groundInlierThreshold = groundInlierThreshold;             
        pre_processor_config.neighborsIndexThreshold = neighborsIndexThreshold;
        pre_processor_config.minPoints = minPoints;
        pre_processor_config.ransac_iterations = ransac_iterations;
        pre_processor_config.grid_type = static_cast<GridType>(grid_type);
        pre_processor_config.returnGroundPoints = returnGroundPoints;

        post_processor_config = pre_processor_config;
        post_processor_config.cellSizeZ = 0.5;
        post_processor_config.processing_phase = 2;

        pre_processor = std::make_unique<PointCloudGrid<PointType>>(pre_processor_config);
        post_processor = std::make_unique<PointCloudGrid<PointType>>(post_processor_config);

        // controller feedback (via TF)
        buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*buffer);

        precision = 0.0;
        recall = 0.0;
        sample_count = 1;
    }

private:

    double precision, recall;

    int sample_count;

    std::shared_ptr<tf2_ros::Buffer> buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    ProcessPointCloud<PointType> processor;
    std::unique_ptr<PointCloudGrid<PointType>> pre_processor;
    std::unique_ptr<PointCloudGrid<PointType>> post_processor;
    GridConfig pre_processor_config;
    GridConfig post_processor_config;
    void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::msg::PointCloud2::SharedPtr raw_ground_points = std::make_shared<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::msg::PointCloud2::SharedPtr ground_points = std::make_shared<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::msg::PointCloud2::SharedPtr obstacle_points = std::make_shared<sensor_msgs::msg::PointCloud2>();

        //Start time
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // Convert the ROS 2 PointCloud2 message to a PCL PointCloud
        typename pcl::PointCloud<PointType> input_cloud;
        typename pcl::PointCloud<PointType> transformed_cloud;
        pcl::fromROSMsg(*msg, input_cloud);

        std::string target_frame = this->get_parameter("target_frame").as_string();
        double maxX = this->get_parameter("maxX").as_double();
        double minX = this->get_parameter("minX").as_double();
        double maxY = this->get_parameter("maxY").as_double();
        double minY = this->get_parameter("minY").as_double();
        double maxZ = this->get_parameter("maxZ").as_double();
        double minZ = this->get_parameter("minZ").as_double();
        bool downsample = this->get_parameter("downsample").as_bool();
        double downsample_distance = this->get_parameter("downsample_distance").as_double();

        typename pcl::PointCloud<PointType>::Ptr input_cloud_ptr;
        if (target_frame != msg->header.frame_id){
            try {
                // Lookup transform from lidar frame to base_link frame
                const geometry_msgs::msg::TransformStamped transformStamped = buffer->lookupTransform(
                    target_frame, msg->header.frame_id, tf2::TimePointZero);

                // Transform each point
                pcl_ros::transformPointCloud(input_cloud, transformed_cloud, transformStamped);

                // Process transformed point cloud
                // Your processing logic here
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "Transform exception: %s", ex.what());
                return;
            }        
            input_cloud_ptr = std::make_shared<typename pcl::PointCloud<PointType>>(transformed_cloud);
        }
        else{
            input_cloud_ptr = std::make_shared<typename pcl::PointCloud<PointType>>(input_cloud);
        }
        
        //TODO: Read robot orientation
        Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0);
        std::cout << "Before: Input Cloud Points: " << input_cloud_ptr->points.size() << std::endl;

        Eigen::Vector4f min{minX,minY,minZ, 1};
        Eigen::Vector4f max{maxX,maxY,maxZ,1};

        typename pcl::PointCloud<PointType>::Ptr filtered_cloud_ptr = processor.FilterCloud(input_cloud_ptr, downsample, downsample_distance, min, max);
        std::cout << "After: Input Cloud Points: " << filtered_cloud_ptr->points.size() << std::endl;

        //PRE
        pre_processor->setInputCloud(filtered_cloud_ptr, orientation);
        std::pair< typename pcl::PointCloud<PointType>::Ptr,  typename pcl::PointCloud<PointType>::Ptr> pre_result = pre_processor->segmentPoints();
        typename pcl::PointCloud<PointType>::Ptr pre_ground_points = pre_result.first;
        typename pcl::PointCloud<PointType>::Ptr pre_non_ground_points = pre_result.second;

        //POST
        post_processor->setInputCloud(pre_ground_points, orientation);
        std::pair< typename pcl::PointCloud<PointType>::Ptr,  typename pcl::PointCloud<PointType>::Ptr> post_result = post_processor->segmentPoints();
        typename pcl::PointCloud<PointType>::Ptr post_ground_points = post_result.first;
        typename pcl::PointCloud<PointType>::Ptr post_non_ground_points  = post_result.second;

        for (typename pcl::PointCloud<PointType>::iterator it = post_non_ground_points->begin(); it != post_non_ground_points->end(); ++it)
        {
            pre_non_ground_points->points.push_back(*it);
        }

        double curr_pre = 0.0;
        double curr_rec = 0.0;

        // Estimation
        calculate_precision_recall(*filtered_cloud_ptr, *post_ground_points, curr_pre, curr_rec);

        precision = precision + curr_pre;
        recall = recall + curr_rec;

        double avg_precision = precision / sample_count;
        double avg_recall = recall / sample_count;

        sample_count++;

        cout << "\033[1;32m P: " << curr_pre << " | R: " << curr_rec << "\033[0m" << endl;
        cout << "\033[1;32m Avg P: " << avg_precision << " | Avg R: " << avg_recall << "\033[0m" << endl;

        std::cout << "Ground Points: " << post_ground_points->points.size() << std::endl;
        std::cout << "Obstacle Points: " << pre_non_ground_points->points.size() << std::endl;

        // Convert PCL PointCloud to ROS PointCloud2 message
        pcl::toROSMsg(*filtered_cloud_ptr, *raw_ground_points);
        pcl::toROSMsg(*post_ground_points, *ground_points);
        pcl::toROSMsg(*pre_non_ground_points, *obstacle_points);

        raw_ground_points->header.frame_id = target_frame;
        raw_ground_points->header.stamp = this->now();

        // Set the frame ID and timestamp
        ground_points->header.frame_id = target_frame;
        ground_points->header.stamp = this->now();
        // Set the frame ID and timestamp
        obstacle_points->header.frame_id = target_frame;
        obstacle_points->header.stamp = this->now();

        // Publish the message
        publisher_ground_points->publish(*ground_points);
        publisher_obstacle_points->publish(*obstacle_points);
        publisher_raw_ground_points->publish(*raw_ground_points);

        //End time
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() * 0.001 << "[ms]" << std::endl;

    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_raw_ground_points;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_ground_points;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_obstacle_points;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<GroundSegmentatioNode>(options));
    rclcpp::shutdown();
    return 0;
}
