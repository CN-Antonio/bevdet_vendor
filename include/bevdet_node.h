/*
 *ROS related .h file
 */
#ifndef __BEVDET_NODE_H__
#define __BEVDET_NODE_H__

// CPP
#include <chrono>

#include "bevdet.h"

#include "cpu_jpegdecoder.h"

#ifdef ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>  // rosmsg2pcl

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h> // sync 6 cam imgs
#include <message_filters/sync_policies/approximate_time.h> // 时间相近同步
#elif ROS_FOUND
#endif // ROS/2_FOUND

namespace bevdet::bevdet_ros
{
// TODO: remove
using std::chrono::high_resolution_clock;
using std::chrono::duration;
using namespace std::chrono_literals; // 500ms

class BEVDet_Node : public rclcpp::Node, public BEVDet
{
public:
    explicit BEVDet_Node(const rclcpp::NodeOptions & node_options);
    ~BEVDet_Node();

    // for test
    void TestNuscenes(YAML::Node &config);
    void publish_imgs(const std::vector<std::vector<char>> &cam_data);
protected:
    size_t img_N_;
    int img_w_; 
    int img_h_;
    uchar* imgs_dev_ = nullptr; 
private:
    void ROSInitParams(void);  // get ros params with <declare_parameters>
    // ROS Init
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_stitched_img; // test
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_top;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers;
    // synced msgs sub
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_cloud_top_{}; 
    // RGB
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_fl_{};
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_f_{};
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_fr_{}; 
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_b_{}; 
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_bl_{};
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_br_{};
    // Compressed
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_cpimg_fl_{};
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_cpimg_f_{};
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_cpimg_fr_{}; 
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_cpimg_b_{}; 
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_cpimg_bl_{};
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_cpimg_br_{};

    // sync 6 cam imgs
    int sync_queue_size_;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image,
        sensor_msgs::msg::Image, sensor_msgs::msg::Image,
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;
    typename std::shared_ptr<Sync> sync_ptr_;

    // sync 6 compressed cam imgs
    using SyncCpPolicy = message_filters::sync_policies::ApproximateTime<
        // sensor_msgs::msg::PointCloud2, 
        sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage,
        sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage,
        sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage>;
    using SyncCp = message_filters::Synchronizer<SyncCpPolicy>;
    typename std::shared_ptr<SyncCp> sync_cp_ptr_;

    void callback(
        const sensor_msgs::msg::Image::ConstSharedPtr & img_fl_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_f_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_fr_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_bl_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_b_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_br_msg
    );

    void callbackCompressed(//const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_cloud, 
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_fl_msg,
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_f_msg,
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_fr_msg,
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_b_msg,
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_bl_msg,
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_br_msg
    );

    const std::string base_frame_;

    // test
    size_t timer_count_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
};

}  // namespace bevdet::bevdet_ros

#endif // __BEVDET_NODE_H__
