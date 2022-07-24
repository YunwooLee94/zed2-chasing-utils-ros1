//
// Created by larr-desktop on 22. 7. 22.
//

#ifndef ZED_CHASING_UTILS_CHASINGCLIENT_H
#define ZED_CHASING_UTILS_CHASINGCLIENT_H

#include <utils/math_util.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <zed_interfaces/ObjectsStamped.h>


class ChasingClient{
private:

    struct State{
        bool isTargetTracked = false;

};

public:
    ChasingClient();
    void depthCallback(const sensor_msgs::CompressedImageConstPtr &depthCompPtr,
                      const sensor_msgs::CameraInfoConstPtr &cameraInfoPtr, const zed_interfaces::ObjectsStampedConstPtr &zedOdPtr);
};

#endif //ZED_CHASING_UTILS_CHASINGCLIENT_H
