#include <signal.h>
#include <execinfo.h>
#include <stdexcept>
#include <memory>
#include <ros/ros.h>
#include "kitti_stereo_odo/kitti_stereo_odo.h"
void handler(int sig)
{
    void* array[10];
    size_t size;

    // get void*'s for all entries on the stack
    size = backtrace(array, 10);

    // print out all the frames to stderr
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

/* Main node entry point. */
int main(int argc, char** argv)
{
    signal(SIGSEGV, handler);
    ros::init(argc, argv, "kitti_stereo_odo");
    ros::NodeHandle nh("~");

    KittiStereoOdoNode kitti_stereo_odo_node(nh);
    kitti_stereo_odo_node.init();

    double rate_Hz = 0;
    nh.param<double>("rate_Hz", rate_Hz, 10);
    ros::Rate r(rate_Hz);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
