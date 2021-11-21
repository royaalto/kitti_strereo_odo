
#include "kitti_stereo_odo/kitti_stereo_odo.h"
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <math.h>

KittiStereoOdoNode::KittiStereoOdoNode(const ros::NodeHandle& nh)
    : nh_(nh)
{
    nh_.param<std::string>("left_cam_topic", left_cam_topic_, "camera/left/image_raw");
    nh_.param<std::string>("right_cam_topic", right_cam_topic_, "camera/right/image_raw");
}

KittiStereoOdoNode::~KittiStereoOdoNode()
{
}

void KittiStereoOdoNode::init()
{
    std::cout<<"get topic name :" << left_cam_topic_<<std::endl;
    std::cout<<"get topic name :" << right_cam_topic_ <<std::endl;
    left_cam_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, left_cam_topic_, 1));
    right_cam_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, right_cam_topic_, 1));
    sync_.reset(new message_filters::Synchronizer<ImagePolicy>(ImagePolicy(10), *left_cam_sub_, *right_cam_sub_));
    sync_->registerCallback(boost::bind(&KittiStereoOdoNode::imageCallBack, this, _1, _2));
}

void KittiStereoOdoNode::imageCallBack(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image)
{
    std::cout<< "left image :" << left_image->header.stamp <<std::endl;

    std::cout<< "right image :" << right_image->header.stamp <<std::endl;

}
