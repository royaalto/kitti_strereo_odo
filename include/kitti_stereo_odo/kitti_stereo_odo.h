#ifndef KITTI_STEREO_ODO_H
#define KITTI_STEREO_ODO_H
#include <string>
#include <memory>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <thread>

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

private:


    //ROS node handle.
    ros::NodeHandle nh_;

    //thread
    std::thread input_;

    //odometry sub
    ros::Subscriber image_sub_;

    //pub
    ros::Publisher odo_pub_;
};
#endif
