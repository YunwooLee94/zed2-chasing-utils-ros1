//
// Created by larr-desktop on 22. 7. 22.
//
#include <zed_chasing_utils/ChasingClient.h>

void ChasingClient::depthCallback(const sensor_msgs::CompressedImageConstPtr &depthCompPtr,
                                 const sensor_msgs::CameraInfoConstPtr &camInfoPtr,
                                 const zed_interfaces::ObjectsStampedConstPtr &zedOdPtr) {
    ros::Time curSensorTime = depthCompPtr->header.stamp;

}

ChasingClient::ChasingClient() {

}
