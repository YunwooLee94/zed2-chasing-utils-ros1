//
// Created by larr-desktop on 22. 7. 22.
//

#ifndef ZED_CHASING_UTILS_CHASINGCLIENT_H
#define ZED_CHASING_UTILS_CHASINGCLIENT_H

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

class ChasingClient{
private:
    image_transport::ImageTransport it;
public:
    explicit ChasingClient(const ros::NodeHandle &nh);
};

#endif //ZED_CHASING_UTILS_CHASINGCLIENT_H
