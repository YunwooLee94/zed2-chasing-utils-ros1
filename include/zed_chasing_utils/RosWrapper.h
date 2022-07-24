//
// Created by larr-desktop on 22. 7. 22.
//

#ifndef ZED_CHASING_UTILS_ROSWRAPPER_H
#define ZED_CHASING_UTILS_ROSWRAPPER_H

#include <ros/ros.h>
#include <zed_chasing_utils/ChasingClient.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/point_cloud2_iterator.h>


#include <compressed_depth_image_transport/codec.h>
#include <compressed_depth_image_transport/compression_common.h>
#include <compressed_image_transport/compression_common.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage,sensor_msgs::CameraInfo, zed_interfaces::ObjectsStamped>
        CompressedImageMaskBbSync;

using namespace std;

Pose getPoseFromGeoMsgs(const geometry_msgs::PoseStamped & poseStamped);
Pose getPoseFromTfMsgs(const tf::StampedTransform& tfStamped);
tf::StampedTransform toTF(const Pose &pose, const string &worldFrameName, const string &frameName, const ros::Time &time);


class RosWrapper{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it;
    string global_frame_id;

    message_filters::Subscriber<sensor_msgs::CompressedImage> * subDepthComp;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *subCamInfo;
    message_filters::Subscriber<zed_interfaces::ObjectsStamped> *subZedOd;

    message_filters::Synchronizer<CompressedImageMaskBbSync>* subSync;

    ChasingClient cc;

    image_transport::Publisher pubDepthMaskImg;
    ros::Publisher pubPointsCorrection; // publish after masking
    ros::Publisher pubPointsMasked; // publish masked point-cloud


    void zedSyncCallback(const sensor_msgs::CompressedImageConstPtr &,
                            const sensor_msgs::CameraInfoConstPtr &,
                            const zed_interfaces::ObjectsStampedConstPtr &
                            );
public:
    RosWrapper();
    void run();
};

#endif //ZED_CHASING_UTILS_ROSWRAPPER_H
