
#include "kitti_stereo_odo/kitti_stereo_odo.h"
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <math.h>

KittiStereoOdoNode::KittiStereoOdoNode(const ros::NodeHandle& nh)
    : nh_(nh)
{

}

KittiStereoOdoNode::~KittiStereoOdoNode()
{

}
