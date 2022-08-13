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
namespace enc = sensor_msgs::image_encodings;


class RosWrapper{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it;

    string global_frame_id;
    int pcl_stride;
    int mask_padding_x;
    int mask_padding_y;
    ChasingClient cc;
    ros::Time zedCallTime;

    bool isCameraPoseReceived;
    bool isObjectPoseReceived;
    bool isDepthImageReceived;
    bool isPclCreated;

    message_filters::Subscriber<sensor_msgs::CompressedImage> * subDepthComp;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *subCamInfo;
    message_filters::Subscriber<zed_interfaces::ObjectsStamped> *subZedOd;
    message_filters::Synchronizer<CompressedImageMaskBbSync>* subSync;

    tf::TransformListener* tfListenerPtr;
    tf::TransformBroadcaster* tfBroadcasterPtr;

    //    ros::Publisher pubCameraPose;
    ros::Publisher pubPointsMasked; // publish masked point-cloud
    ros::Publisher pubObjectPos; // publish masked
    image_transport::Publisher pubDepthMaskImg;



    void zedSyncCallback(const sensor_msgs::CompressedImageConstPtr &,
                            const sensor_msgs::CameraInfoConstPtr &,
                            const zed_interfaces::ObjectsStampedConstPtr &
                            );

    Pose tfCallBack(const sensor_msgs::CompressedImageConstPtr&);
    Pose tfObjCallBack(const zed_interfaces::ObjectsStampedConstPtr&);

    cv::Mat pngDecompressDepth(const sensor_msgs::CompressedImageConstPtr& depthPtr);
    string getDepthImageFrameId(const sensor_msgs::CompressedImageConstPtr& depthCompPtr);
    uint64_t getDepthImageTimeStamp(const sensor_msgs::CompressedImageConstPtr& depthCompPtr);

    static Pose getPoseFromGeoMsgs(const geometry_msgs::PoseStamped & poseStamped);
    static Pose getPoseFromTfMsgs(const tf::StampedTransform& tfStamped);
    static tf::StampedTransform toTF(const Pose &pose, const string &worldFrameName, const string &frameName, const ros::Time &time);
    geometry_msgs::PoseStamped poseToGeoMsgs(const Pose &pose);
    geometry_msgs::PointStamped poseToGeoMsgsPoint(const Pose &pose);
    sensor_msgs::ImagePtr imageToROSmsg(const cv::Mat& img, const std::string encodingType, std::string frameId, ros::Time t);

public:
    RosWrapper();
    void run();
};

#endif //ZED_CHASING_UTILS_ROSWRAPPER_H
