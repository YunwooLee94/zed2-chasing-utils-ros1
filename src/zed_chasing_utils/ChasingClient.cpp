//
// Created by larr-desktop on 22. 7. 22.
//
#include <zed_chasing_utils/ChasingClient.h>

void ChasingClient::depthCallback(const sensor_msgs::CameraInfoConstPtr &camInfoPtr,
                                 const zed_interfaces::ObjectsStampedConstPtr &zedOdPtr) {
    image_geometry::PinholeCameraModel model_;
    model_.fromCameraInfo(*camInfoPtr);
    double camera_cx = model_.cx();
    double camera_cy = model_.cy();
    double camera_fx = model_.fx();
    double camera_fy = model_.fy();
    double camera_factor = 1;

    drone_state.pclObjectsRemoved.clear();
    drone_state.pclObjectsRemoved.header.frame_id = param.global_frame_id;
    drone_state.pclObjectsRemoved.header.stamp = depthImgTimeStamp;

    int rGlobalMin = 0;
    int cGlobalMin = 0;
    int rGlobalMax = decompDepthImg.rows - 1;
    int cGlobalMax = decompDepthImg.cols - 1;
    int rMin = rGlobalMin;
    int rMax = rGlobalMax;
    int cMin = cGlobalMin;
    int cMax = cGlobalMax;
    for(int idx=0;idx<zedOdPtr->objects.size();idx++)
    {
        rMin = std::max(int(zedOdPtr->objects[idx].bounding_box_2d.corners[0].kp[1])/2 - param.mask_padding_y, rGlobalMin);
        cMin = std::max(int(zedOdPtr->objects[idx].bounding_box_2d.corners[0].kp[0])/2 - param.mask_padding_x, cGlobalMin);
        rMax = std::min(int(zedOdPtr->objects[idx].bounding_box_2d.corners[2].kp[1])/2 + param.mask_padding_y, rGlobalMax);
        cMax = std::min(int(zedOdPtr->objects[idx].bounding_box_2d.corners[2].kp[0])/2 + param.mask_padding_x, cGlobalMax);

        const float NaN = std::numeric_limits<float>::quiet_NaN() ;
        // Depth Image Masking
        for (int r = rMin; r< rMax; r++){
            for(int c = cMin; c< cMax; c++){
                decompDepthImg.ptr<float>(r)[c] = NaN;
            }
        }
        drone_state.bbox_2d[0] = rMin;
        drone_state.bbox_2d[1] = cMin;
        drone_state.bbox_2d[2] = rMax;
        drone_state.bbox_2d[3] = cMax;
    }
    if(zedOdPtr->objects.empty()){
        const float NaN = std::numeric_limits<float>::quiet_NaN() ;
        // Depth Image Masking
        for (int r = drone_state.bbox_2d[0]; r< drone_state.bbox_2d[2]; r++){
            for(int c = drone_state.bbox_2d[1]; c< drone_state.bbox_2d[3]; c++){
                decompDepthImg.ptr<float>(r)[c] = NaN;
            }
        }
    }

    // Point-cloud Generation
    float  pcl_pts[3];
    for (int r = 0; r < decompDepthImg.rows; r += param.pcl_stride) {
        for (int c = 0; c < decompDepthImg.cols; c += param.pcl_stride) {
            float d = decompDepthImg.ptr<float>(r)[c];
            if (not isnan(d))
            {
                pcl_pts[2] = (float) (d/ camera_factor);
                pcl_pts[0] = (float) ((c - camera_cx) * pcl_pts[2]/ camera_fx);
                pcl_pts[1] = (float) ((r - camera_cy) * pcl_pts[2] / camera_fy);
                // point cloud transformation
                Point p_w = drone_state.T_wc.poseMat * Point(pcl_pts[0], pcl_pts[1], pcl_pts[2]).toEigen();
                pcl::PointXYZ p;
                p.x = p_w.x;
                p.y = p_w.y;
                p.z = p_w.z;
                drone_state.pclObjectsRemoved.points.push_back(p);
            }
        }
    }
}

ChasingClient::ChasingClient(){

}


void ChasingClient::setPose(const Pose &pose) {
    drone_state.T_wc = pose;
    drone_state.T_cw = drone_state.T_wc;
    drone_state.T_cw.inverse();
}

void ChasingClient::setObjPose(const Pose& pose) {
    if(not (isnan(pose.getTranslation().x) or isnan(pose.getTranslation().y) or isnan(pose.getTranslation().z)))
    {
        drone_state.T_wo = pose;
    }
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