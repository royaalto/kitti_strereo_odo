
#include "kitti_stereo_odo/kitti_stereo_odo.h"
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <math.h>

KittiStereoOdoNode::KittiStereoOdoNode(const ros::NodeHandle& nh)
    : nh_(nh)
    // , stereo_image_handle_thread_{&KittiStereoOdoNode::stereo_image_handle_thread_, this}
{
    nh_.param<std::string>("left_cam_topic", left_cam_topic_, "camera/left/image_raw");
    nh_.param<std::string>("right_cam_topic", right_cam_topic_, "camera/right/image_raw");
}

KittiStereoOdoNode::~KittiStereoOdoNode()
{
    if(stereo_image_handle_thread_.joinable())
    {
        stereo_image_handle_thread_.join();
    }
}

void KittiStereoOdoNode::init()
{
    std::cout << "get topic name :" << left_cam_topic_ << std::endl;
    std::cout << "get topic name :" << right_cam_topic_ << std::endl;
    left_cam_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, left_cam_topic_, 1));
    right_cam_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, right_cam_topic_, 1));
    sync_.reset(new message_filters::Synchronizer<ImagePolicy>(ImagePolicy(10), *left_cam_sub_, *right_cam_sub_));
    sync_->registerCallback(boost::bind(&KittiStereoOdoNode::imageCallBack, this, _1, _2));
    stereo_image_handle_thread_ = std::thread(&KittiStereoOdoNode::stereoImageHandle, this);
}

void KittiStereoOdoNode::imageCallBack(const sensor_msgs::ImageConstPtr& left_image,
                                       const sensor_msgs::ImageConstPtr& right_image)
{
    std::cout << "left image :" << left_image->header.stamp << std::endl;
    std::cout << "right image :" << right_image->header.stamp << std::endl;

    std::vector<sensor_msgs::Image> image_pair;
    image_pair.push_back(*left_image);
    image_pair.push_back(*right_image);
    std::lock_guard<std::mutex> lock(image_buffer_mutex_);
    image_buffer_.push_back(image_pair);
}

void KittiStereoOdoNode::stereoImageHandle()
{
    while (true)
    {
        if (image_buffer_.size() > 0)
        {
            std::cout<<"image_buffer size"<< image_buffer_.size()<<std::endl;
            std::lock_guard<std::mutex> lock(image_buffer_mutex_);
            image_buffer_.pop_front();
        }
    }
}
