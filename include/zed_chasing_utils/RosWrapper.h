//
// Created by larr-desktop on 22. 7. 22.
//

#ifndef ZED_CHASING_UTILS_ROSWRAPPER_H
#define ZED_CHASING_UTILS_ROSWRAPPER_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <zed_chasing_utils/ChasingClient.h>

#include <compressed_depth_image_transport/codec.h>
#include <compressed_depth_image_transport/compression_common.h>
#include <compressed_image_transport/compression_common.h>

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::CompressedImage, sensor_msgs::CameraInfo, zed_interfaces::ObjectsStamped>
    CompressedImageMaskBbSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage,
                                                        sensor_msgs::CameraInfo>
    CompressedImageMaskBbSyncMock;

using namespace std;
namespace enc = sensor_msgs::image_encodings;

class RosWrapper {
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it;
  chasing_client::Param param_;

  chasing_client::ChasingClient cc;
  ros::Time zedCallTime;

  bool isCameraPoseReceived;
  bool isObjectPoseReceived;
  bool isDepthImageReceived;
  bool isPclCreated;

  message_filters::Subscriber<sensor_msgs::CompressedImage> *subDepthComp;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCamInfo;
  message_filters::Subscriber<zed_interfaces::ObjectsStamped> *subZedOd;
  message_filters::Synchronizer<CompressedImageMaskBbSync> *subSync;
  message_filters::Synchronizer<CompressedImageMaskBbSyncMock> *subSyncMock;

  tf::TransformListener *tfListenerPtr;
  tf::TransformBroadcaster *tfBroadcasterPtr;

  //    ros::Publisher pubCameraPose;
  ros::Publisher pubPointsMasked;           // publish masked point-cloud
  std::vector<ros::Publisher> pubObjectPos; // publish objects

  image_transport::Publisher pubDepthMaskImg;

  void zedSyncCallback(const sensor_msgs::CompressedImageConstPtr &,
                       const sensor_msgs::CameraInfoConstPtr &,
                       const zed_interfaces::ObjectsStampedConstPtr &);

  void zedSyncCallbackMock(const sensor_msgs::CompressedImageConstPtr &,
                       const sensor_msgs::CameraInfoConstPtr &);

  Pose tfCallBack(const sensor_msgs::CompressedImageConstPtr &);
  vector<Pose> tfObjCallBack(const zed_interfaces::ObjectsStampedConstPtr &);

  cv::Mat pngDecompressDepth(const sensor_msgs::CompressedImageConstPtr &depthPtr);
  string getDepthImageFrameId(const sensor_msgs::CompressedImageConstPtr &depthCompPtr);
  uint64_t getDepthImageTimeStamp(const sensor_msgs::CompressedImageConstPtr &depthCompPtr);

  static Pose getPoseFromGeoMsgs(const geometry_msgs::PoseStamped &poseStamped);
  static Pose getPoseFromTfMsgs(const tf::StampedTransform &tfStamped);
  static tf::StampedTransform toTF(const Pose &pose, const string &worldFrameName,
                                   const string &frameName, const ros::Time &time);
  geometry_msgs::PoseStamped poseToGeoMsgs(const Pose &pose);
  geometry_msgs::PointStamped poseToGeoMsgsPoint(const Pose &pose, const ros::Time &stamp);
  sensor_msgs::ImagePtr imageToROSmsg(const cv::Mat &img, const std::string encodingType,
                                      std::string frameId, ros::Time t);

public:
  RosWrapper();
  void run();
};

#endif // ZED_CHASING_UTILS_ROSWRAPPER_H
