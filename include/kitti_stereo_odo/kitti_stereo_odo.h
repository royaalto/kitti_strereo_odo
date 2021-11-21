#ifndef KITTI_STEREO_ODO_H
#define KITTI_STEREO_ODO_H
#include <string>
#include <memory>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <thread>
using ImagePolicy = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image>;
class KittiStereoOdoNode
{
public:
    /**
     * @brief  Construct a new KittiStereoOdoNode.
     *
     * @param  private_nh :
     */
    KittiStereoOdoNode(const ros::NodeHandle& private_nh);

    /**
     * @brief  Destroy the KittiStereoOdoNode.
     *
     */
    ~KittiStereoOdoNode();

    void init();

private:
    /**
     * @brief
     *
     */
    void imageCallBack(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image);

    //ROS node handle.
    ros::NodeHandle nh_;

    //thread
    std::thread input_;

    //camera topic
    std::string left_cam_topic_;
    std::string right_cam_topic_;

    //image sub
    ros::Subscriber image_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> left_cam_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> right_cam_sub_;

    //time sync

    std::unique_ptr<message_filters::Synchronizer<ImagePolicy>> sync_;

    //pub
    ros::Publisher odo_pub_;
};
#endif
