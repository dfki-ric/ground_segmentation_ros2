#define PCL_NO_PRECOMPILE
#include "rclcpp/rclcpp.hpp"
#include <pointcloud_obstacle_detection/ground_detection_types.hpp>
#include <pointcloud_obstacle_detection/ground_detection.hpp>
#include <pointcloud_obstacle_detection/pointcloud_processor.hpp>

#include "common.hpp"

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_eigen/tf2_eigen.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

#include "message_filters/time_synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <Eigen/Dense>
#include <memory>
#include <chrono>
#include <vector>
#include <limits>
#include <type_traits>

using namespace pointcloud_obstacle_detection;

#define USE_POINTXYZILID  // or USE_POINTXYZILID

#ifdef USE_POINTXYZ
    using PointType = pcl::PointXYZ;
#elif defined(USE_POINTXYZILID)
    using PointType = PointXYZILID;
#endif

void checkSubscriptionConnection(
    const rclcpp::SubscriptionBase::SharedPtr& subscription,
    const std::string& topic_name,
    const rclcpp::Logger& logger)
{
    if (subscription) {
        size_t count = subscription->get_publisher_count();
        if (count == 0) {
            RCLCPP_WARN(logger, "No publishers connected to topic for: %s", topic_name.c_str());
        } else {
            RCLCPP_INFO(logger, "%ld publisher(s) connected to topic for: %s", count, topic_name.c_str());
        }
    } else {
        RCLCPP_ERROR(logger, "Failed to access subscription object to topic for: %s", topic_name.c_str());
    }
}

void injectSyntheticGroundDisk(pcl::PointCloud<PointType>::Ptr& cloud,
                               float max_radius,
                               float z,
                               int rings,
                               int points_per_ring) {
    for (int r = 1; r <= rings; ++r) {
        float radius = max_radius * r / rings;
        int num_points = points_per_ring * r; // more points on outer rings
        for (int i = 0; i < num_points; ++i) {
            float angle = 2.0f * M_PI * i / num_points;
            float x = radius * std::cos(angle);
            float y = radius * std::sin(angle);

            PointType pt;
            pt.x = x;
            pt.y = y;
            pt.z = z;
#ifdef USE_POINTXYZILID
            pt.intensity = 0.0f;
            pt.label = 1;  // optional: label as ground
            pt.id = 9999;  // optional: ID for synthetic point
#endif
            cloud->points.push_back(pt);
        }
    }

    // Optionally add a center point
    PointType center;
    center.x = 0.0f;
    center.y = 0.0f;
    center.z = z;
#ifdef USE_POINTXYZILID
    center.intensity = 0.0f;
    center.label = 1;
    center.id = 9999;
#endif
    cloud->points.push_back(center);

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
}

class GroundSegmentatioNode : public rclcpp::Node {
public:
    GroundSegmentatioNode(rclcpp::NodeOptions options) : Node("ground_segmentation",options) {
        publisher_clusters = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/clusters", 10);
        publisher_ground_points = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/ground_points", 10);
        publisher_obstacle_points = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/obstacle_points", 10);
        publisher_raw_points = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/raw_points", 10);

        if (this->get_parameter("use_imu_orientation").as_bool()){
            subscriber_synced_pointcloud = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "/ground_segmentation/input_pointcloud");
            subscriber_synced_imu = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, "/ground_segmentation/input_imu");
 
            auto imu_sub_ptr = subscriber_synced_imu->getSubscriber();
            auto pc_sub_ptr  = subscriber_synced_pointcloud->getSubscriber();

            checkSubscriptionConnection(imu_sub_ptr, "IMU", this->get_logger());
            checkSubscriptionConnection(pc_sub_ptr, "Pointcloud2", this->get_logger());
 
            sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(50), *subscriber_synced_pointcloud, *subscriber_synced_imu);
            sync->registerCallback(std::bind(&GroundSegmentatioNode::syncedCallback, this, std::placeholders::_1, std::placeholders::_2));
        }
        else {
            subscriber_pointcloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/ground_segmentation/input_pointcloud", 10, std::bind(&GroundSegmentatioNode::callback, this, std::placeholders::_1));
            checkSubscriptionConnection(subscriber_pointcloud, "Pointcloud2", this->get_logger());
        }

        show_benchmark = this->get_parameter("show_benchmark").as_bool();
        if(show_benchmark){
            pub_tp = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/TP", 10);
            pub_fn = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/FN", 10);
            pub_fp = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/FP", 10);
        }

        compute_clusters = this->get_parameter("compute_clusters").as_bool();
   
        robot_frame = this->get_parameter("robot_frame").as_string();

        pre_processor_config.cellSizeX = this->get_parameter("cellSizeX").as_double();
        pre_processor_config.cellSizeY = this->get_parameter("cellSizeY").as_double();
        pre_processor_config.cellSizeZ = this->get_parameter("cellSizeZ").as_double();
        pre_processor_config.slopeThresholdDegrees = this->get_parameter("slopeThresholdDegrees").as_double();
        pre_processor_config.groundInlierThreshold = this->get_parameter("groundInlierThreshold").as_double();
        pre_processor_config.distToGround = this->get_parameter("dist_to_ground").as_double();
        pre_processor_config.centroidSearchRadius = this->get_parameter("centroidSearchRadius").as_double();
        

        post_processor_config = pre_processor_config;
        post_processor_config.cellSizeZ = 0.2;
        post_processor_config.processing_phase = 2;

        pre_processor = std::make_unique<PointCloudGrid<PointType>>(pre_processor_config);
        post_processor = std::make_unique<PointCloudGrid<PointType>>(post_processor_config);

        // controller feedback (via TF)
        buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*buffer);

        precision = 0.0;
        recall = 0.0;
        final_non_ground_points = std::make_shared<pcl::PointCloud<PointType>>(); 
        final_ground_points = std::make_shared<pcl::PointCloud<PointType>>(); 
        injected_points_ptr = std::make_shared<pcl::PointCloud<PointType>>(); 

        double dist_to_ground = this->get_parameter("dist_to_ground").as_double();
        double robot_radius = this->get_parameter("robot_radius").as_double();

        injectSyntheticGroundDisk(injected_points_ptr, robot_radius, -dist_to_ground, 5, 20);
    }

private:

    std::string robot_frame;

    bool show_benchmark, show_grid, compute_clusters;
    double precision, recall;
    std::vector<double> runtime, recall_arr, prec_arr, recall_o_arr, prec_o_arr;

    std::shared_ptr<tf2_ros::Buffer> buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<PointCloudGrid<PointType>> pre_processor, post_processor;
    GridConfig pre_processor_config, post_processor_config;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr 
        publisher_clusters,
        publisher_raw_points, 
        publisher_ground_points, 
        publisher_obstacle_points, 
        pub_tp, 
        pub_fn, 
        pub_fp;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr 
        publisher_start_cells;
    
    rclcpp::Publisher<ground_segmentation::msg::GridMap>::SharedPtr 
        pre_grid_map_publisher, 
        post_grid_map_publisher;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_pointcloud;


    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> subscriber_synced_pointcloud;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> subscriber_synced_imu;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Imu>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

    ProcessCloudProcessor<PointType> processor;

    typename pcl::PointCloud<PointType>::Ptr final_non_ground_points;
    typename pcl::PointCloud<PointType>::Ptr final_ground_points;
    typename pcl::PointCloud<PointType>::Ptr injected_points_ptr;

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg){
        segmentation(pointcloud_msg);
    }

    void syncedCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointcloud_msg, const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg){
        segmentation(pointcloud_msg, imu_msg);
    }

    void segmentation(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointcloud_msg, const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg = nullptr){
        sensor_msgs::msg::PointCloud2::SharedPtr raw_points = std::make_shared<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::msg::PointCloud2::SharedPtr ground_points = std::make_shared<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::msg::PointCloud2::SharedPtr obstacle_points = std::make_shared<sensor_msgs::msg::PointCloud2>();

        sensor_msgs::msg::PointCloud2::SharedPtr msg_TP = std::make_shared<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::msg::PointCloud2::SharedPtr msg_FP = std::make_shared<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::msg::PointCloud2::SharedPtr msg_FN = std::make_shared<sensor_msgs::msg::PointCloud2>();

        // Convert the ROS 2 PointCloud2 message to a PCL PointCloud
        typename pcl::PointCloud<PointType> input_cloud;
        typename pcl::PointCloud<PointType> transformed_cloud;
        pcl::fromROSMsg(*pointcloud_msg, input_cloud);

        double maxX = this->get_parameter("maxX").as_double();
        double minX = this->get_parameter("minX").as_double();
        double maxY = this->get_parameter("maxY").as_double();
        double minY = this->get_parameter("minY").as_double();
        double maxZ = this->get_parameter("maxZ").as_double();
        double minZ = this->get_parameter("minZ").as_double();
        bool downsample = this->get_parameter("downsample").as_bool();
        double downsample_resolution = this->get_parameter("downsample_resolution").as_double();


        pcl::PointCloud<PointType> combined_cloud = input_cloud; // Copy or assign input_cloud
        combined_cloud += *injected_points_ptr;

        //Idea: We could also align the whole pointcloud with gravity.
        typename pcl::PointCloud<PointType>::Ptr input_cloud_ptr;

        if (robot_frame != pointcloud_msg->header.frame_id){
            try {
                const geometry_msgs::msg::TransformStamped transformStamped = buffer->lookupTransform(
                    robot_frame, pointcloud_msg->header.frame_id, pointcloud_msg->header.stamp,tf2::durationFromSec(0.1));
		Eigen::Affine3f transformEigen = tf2::transformToEigen(transformStamped.transform).cast<float>();
                pcl::transformPointCloud(combined_cloud, transformed_cloud, transformEigen);

            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "Pointcloud Transform exception: %s", ex.what());
                return;
            }        
            input_cloud_ptr = std::make_shared<typename pcl::PointCloud<PointType>>(transformed_cloud);
        }
        else{
            input_cloud_ptr = std::make_shared<typename pcl::PointCloud<PointType>>(combined_cloud);
        }

        tf2::Quaternion robot_in_gravity(0,0,0,1);
        if (imu_msg != nullptr){
            tf2::Quaternion imu_in_gravity(imu_msg->orientation.x,
                                           imu_msg->orientation.y,
                                           imu_msg->orientation.z,
                                           imu_msg->orientation.w);
                tf2::Quaternion robot_in_imu;
                try
                {
                    geometry_msgs::msg::TransformStamped robot_in_imu_transform = buffer->lookupTransform(
                        imu_msg->header.frame_id, robot_frame, imu_msg->header.stamp,tf2::durationFromSec(0.1));
                    robot_in_imu.setX(robot_in_imu_transform.transform.rotation.x);
                    robot_in_imu.setY(robot_in_imu_transform.transform.rotation.y);
                    robot_in_imu.setZ(robot_in_imu_transform.transform.rotation.z);
                    robot_in_imu.setW(robot_in_imu_transform.transform.rotation.w);
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_ERROR(this->get_logger(), "IMU Transform exception: %s", ex.what());
                    return;
                }

                robot_in_gravity = imu_in_gravity * robot_in_imu;
                robot_in_gravity.normalize();
        }

        Eigen::Quaterniond robot_orientation{robot_in_gravity.getW(),
                                             robot_in_gravity.getX(),
                                             robot_in_gravity.getY(),
                                             robot_in_gravity.getZ()};

        Eigen::Vector4f min{minX,minY,minZ, 1};
        Eigen::Vector4f max{maxX,maxY,maxZ,1};

        typename pcl::PointCloud<PointType>::Ptr filtered_cloud_ptr = processor.filterCloud(input_cloud_ptr, downsample, downsample_resolution, min, max, false);

        //Start time

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        //PRE
        pre_processor->setInputCloud(filtered_cloud_ptr, robot_orientation);
        std::pair< typename pcl::PointCloud<PointType>::Ptr,  typename pcl::PointCloud<PointType>::Ptr> pre_result = pre_processor->segmentPoints();
        typename pcl::PointCloud<PointType>::Ptr pre_ground_points = pre_result.first;
        typename pcl::PointCloud<PointType>::Ptr pre_non_ground_points = pre_result.second;

        //POST
        post_processor->setInputCloud(pre_ground_points, robot_orientation);
        std::pair< typename pcl::PointCloud<PointType>::Ptr,  typename pcl::PointCloud<PointType>::Ptr> post_result = post_processor->segmentPoints();
        typename pcl::PointCloud<PointType>::Ptr post_ground_points = post_result.first;
        typename pcl::PointCloud<PointType>::Ptr post_non_ground_points  = post_result.second;

        double robot_radius = this->get_parameter("robot_radius").as_double();
        double dist_to_ground = this->get_parameter("dist_to_ground").as_double();

        Eigen::Vector4f min_radius{-robot_radius,-robot_radius,-dist_to_ground, 1};
        Eigen::Vector4f max_radius{robot_radius,robot_radius,-dist_to_ground+1,1};

        final_ground_points = processor.filterCloud(post_ground_points, downsample,
                                                    downsample_resolution, min_radius, max_radius, true);

        *final_non_ground_points = *pre_non_ground_points + *post_non_ground_points;

        final_non_ground_points = processor.filterCloud(final_non_ground_points, downsample,
                                                    downsample_resolution, min_radius, max_radius, true);
        
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();



        if (compute_clusters){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
#ifdef USE_POINTXYZ
            std::vector<pcl::PointCloud<PointType>::Ptr> clusters = processor.fastEuclideanClustering(final_non_ground_points,1,3,1000000);
#else 
            std::vector<pcl::PointCloud<PointType>::Ptr> clusters = processor.euclideanClustering(final_non_ground_points,1,3,1000000);
#endif
            for (const auto& cluster : clusters){
                // Assign a unique color to this cluster
                uint8_t r = rand() % 256;
                uint8_t g = rand() % 256;
                uint8_t b = rand() % 256;
                uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
                
                for (typename pcl::PointCloud<PointType>::iterator it = cluster->begin(); it != cluster->end(); ++it){
                    pcl::PointXYZRGB point;
                    point.x = it->x;
                    point.y = it->y;
                    point.z = it->z;
                    point.rgb = *reinterpret_cast<float*>(&rgb); // Assign the color

                    colored_cloud->points.push_back(point);
                }
            }  
            colored_cloud->width = colored_cloud->points.size();
            colored_cloud->height = 1;
            colored_cloud->is_dense = true;  

            // Convert PCL PointCloud back to ROS PointCloud2
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*colored_cloud, output);
            output.header = pointcloud_msg->header;
            publisher_clusters->publish(output);
        }

        if (show_benchmark) {
            // End time
            double rt = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() * 0.001;
            runtime.push_back(rt);
            double rt_mean = std::accumulate(runtime.begin(), runtime.end(), 0.0) / runtime.size();
            std::cout << "Avg Time difference = " << rt_mean << "[ms]" << ", " << cal_stdev(runtime) << std::endl;

#ifdef USE_POINTXYZILID
            // Estimation
            double precision = 0.0, recall = 0.0;
            static std::vector<int> TPFNs; // TP, FP, FN, TF order

            calculate_precision_recall(*filtered_cloud_ptr, *final_ground_points, precision, recall, TPFNs);

            //calculate_precision_recall_non_ground(*filtered_cloud_ptr, *final_non_ground_points, precision, recall, TPFNs);

            if (precision >= 0 && recall >= 0) { // extra safety check
                prec_arr.push_back(precision);
                recall_arr.push_back(recall);

                double recall_mean = std::accumulate(recall_arr.begin(), recall_arr.end(), 0.0) / recall_arr.size();
                double prec_mean = std::accumulate(prec_arr.begin(), prec_arr.end(), 0.0) / prec_arr.size();

                double f1_score = 0.0;
                if ((prec_mean + recall_mean) > 0)
                    f1_score = 2 * (prec_mean * recall_mean) / (prec_mean + recall_mean);

                std::cout << "\033[1;32m Avg P: " << prec_mean << ", " << cal_stdev(prec_arr)
                        << " | Avg R: " << recall_mean << ", " << cal_stdev(recall_arr)
                        << " | Avg F1: " << f1_score << "\033[0m" << std::endl;
            } else {
                std::cout << "Warning: Invalid precision/recall values detected, skipping this entry.\n";
            }

            // Publish msg
            pcl::PointCloud<PointType> TP, FP, FN, TN;

            discern_ground(*final_ground_points, TP, FP);
            discern_ground(*final_non_ground_points, FN, TN);

            pcl::toROSMsg(TP, *msg_TP);
            pcl::toROSMsg(FP, *msg_FP);
            pcl::toROSMsg(FN, *msg_FN);

            msg_TP->header.frame_id = robot_frame;
            msg_TP->header.stamp = this->now();

            msg_FP->header.frame_id = robot_frame;
            msg_FP->header.stamp = this->now();
            msg_FN->header.frame_id = robot_frame;
            msg_FN->header.stamp = this->now();

            pub_tp->publish(*msg_TP);
            pub_fp->publish(*msg_FP);
            pub_fn->publish(*msg_FN);
#endif
        }

        final_non_ground_points->width = final_non_ground_points->points.size();
        final_non_ground_points->height = 1;
        final_non_ground_points->is_dense = true;  

        final_ground_points->width = final_ground_points->points.size();
        final_ground_points->height = 1;
        final_ground_points->is_dense = true;  

        pcl::toROSMsg(*filtered_cloud_ptr, *raw_points);
        pcl::toROSMsg(*final_ground_points, *ground_points);
        pcl::toROSMsg(*final_non_ground_points, *obstacle_points);

        ground_points->header.frame_id = robot_frame;
        obstacle_points->header.frame_id = robot_frame;
        raw_points->header.frame_id = robot_frame;

        ground_points->header.stamp = this->now();
        obstacle_points->header.stamp = this->now();
        raw_points->header.stamp = this->now();

        // Publish the message
        publisher_ground_points->publish(*ground_points);
        publisher_obstacle_points->publish(*obstacle_points);
        publisher_raw_points->publish(*raw_points);

        final_ground_points->clear();
        final_non_ground_points->clear();
    }
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
