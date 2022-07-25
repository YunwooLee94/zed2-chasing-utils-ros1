//
// Created by larr-desktop on 22. 7. 22.
//
#include <zed_chasing_utils/ChasingClient.h>

void ChasingClient::depthCallback(const sensor_msgs::CompressedImageConstPtr &depthCompPtr,
                                 const sensor_msgs::CameraInfoConstPtr &camInfoPtr,
                                 const zed_interfaces::ObjectsStampedConstPtr &zedOdPtr) {
    image_geometry::PinholeCameraModel model_;
    model_.fromCameraInfo(*camInfoPtr);
    double camera_cx = model_.cx();
    double camera_cy = model_.cy();
    double camera_fx = model_.fx();
    double camera_fy = model_.fy();
    double camera_factor = 1;

    drone_state.pclObjectsRemoved.clear();
    drone_state.pclObjectsRemoved.header.frame_id = depthCompPtr->header.frame_id;
    drone_state.pclObjectsRemoved.header.stamp = (uint64_t)(depthCompPtr->header.stamp.toNSec()/1000ull);

    int rGlobalMin = 0;
    int cGlobalMin = 0;
    int rGlobalMax = decompDepthImg.rows - 1;
    int cGlobalMax = decompDepthImg.cols - 1;

    int rMin = std::max(int(zedOdPtr->objects[0].bounding_box_2d.corners[0].kp[1]) - param.mask_padding_y, rGlobalMin);
    int cMin = std::max(int(zedOdPtr->objects[0].bounding_box_2d.corners[0].kp[0]) - param.mask_padding_x, cGlobalMin);
    int rMax = std::min(int(zedOdPtr->objects[0].bounding_box_2d.corners[2].kp[1]) + param.mask_padding_y, rGlobalMax);
    int cMax = std::min(int(zedOdPtr->objects[0].bounding_box_2d.corners[2].kp[0]) + param.mask_padding_x, cGlobalMax);

    for (int r = 0; r < decompDepthImg.rows; r += param.pcl_stride) {
        for (int c = 0; c < decompDepthImg.cols; c += param.pcl_stride) {
            float d = decompDepthImg.ptr<float>(r)[c];
            if (d != d) // nan
                continue;
            if( not (rMin<r and r <rMax and cMin<c and c<cMax))
            {
                pcl::PointXYZ p;
                p.z = (float) (d/ camera_factor);
                p.x = (float) ((c - camera_cx) * p.z / camera_fx);
                p.y = (float) ((r - camera_cy) * p.z / camera_fy);
                drone_state.pclObjectsRemoved.points.push_back(p);
            }
        }
    }

}

ChasingClient::ChasingClient() {

}

void ChasingClient::setPose(const Pose &pose) {
    drone_state.T_wc = pose;
    drone_state.T_cw = drone_state.T_wc;
    drone_state.T_cw.inverse();
}

void ChasingClient::setObjPose(const Pose& pose) {
    drone_state.T_wo = pose;
}

void ChasingClient::setDecompDepth(const cv::Mat &decompDepth_) {
    decompDepthImg = decompDepth_;
}

void ChasingClient::setParam(const std::string &global_frame_id_, const int &pcl_stride_,
                             const int &mask_padding_x, const int &mask_padding_y) {
    param.global_frame_id = global_frame_id_;
    param.pcl_stride = pcl_stride_;
    param.mask_padding_x = mask_padding_x;
    param.mask_padding_y = mask_padding_y;
}