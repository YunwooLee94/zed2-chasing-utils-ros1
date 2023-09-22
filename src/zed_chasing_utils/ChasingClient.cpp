//
// Created by larr-desktop on 22. 7. 22.
//
#include <zed_chasing_utils/ChasingClient.h>

void chasing_client::ChasingClient::depthCallback(const sensor_msgs::CameraInfoConstPtr &camInfoPtr,
                                 const zed_interfaces::ObjectsStampedConstPtr &zedOdPtr) {
    image_geometry::PinholeCameraModel model_;
    model_.fromCameraInfo(*camInfoPtr);
    double camera_cx = model_.cx();
    double camera_cy = model_.cy();
    double camera_fx = model_.fx();
    double camera_fy = model_.fy();
    double camera_factor = 1;

    state_.pclObjectsRemoved.clear();
    state_.pclObjectsRemoved.header.frame_id = param_.global_frame_id;
    state_.pclObjectsRemoved.header.stamp = depthImgTimeStamp;

    int rGlobalMin = 0;
    int cGlobalMin = 0;
    int rGlobalMax = decompDepthImg_.rows - 1;
    int cGlobalMax = decompDepthImg_.cols - 1;
    int rMin = rGlobalMin;
    int rMax = rGlobalMax;
    int cMin = cGlobalMin;
    int cMax = cGlobalMax;
    for(int idx=0;idx<zedOdPtr->objects.size();idx++)
    {
        rMin = std::max(int(zedOdPtr->objects[idx].bounding_box_2d.corners[0].kp[1])/2 - param_.mask_padding_y, rGlobalMin);
        cMin = std::max(int(zedOdPtr->objects[idx].bounding_box_2d.corners[0].kp[0])/2 - param_.mask_padding_x, cGlobalMin);
        rMax = std::min(int(zedOdPtr->objects[idx].bounding_box_2d.corners[2].kp[1])/2 + param_.mask_padding_y, rGlobalMax);
        cMax = std::min(int(zedOdPtr->objects[idx].bounding_box_2d.corners[2].kp[0])/2 + param_.mask_padding_x, cGlobalMax);

        const float NaN = std::numeric_limits<float>::quiet_NaN() ;
        // Depth Image Masking
        for (int r = rMin; r< rMax; r++){
            for(int c = cMin; c< cMax; c++){
                decompDepthImg_.ptr<float>(r)[c] = NaN;
            }
        }
        state_.bbox_2d[0] = rMin;
        state_.bbox_2d[1] = cMin;
        state_.bbox_2d[2] = rMax;
        state_.bbox_2d[3] = cMax;
    }
    if(zedOdPtr->objects.empty()){
        const float NaN = std::numeric_limits<float>::quiet_NaN() ;
        // Depth Image Masking
        for (int r = state_.bbox_2d[0]; r< state_.bbox_2d[2]; r++){
            for(int c = state_.bbox_2d[1]; c< state_.bbox_2d[3]; c++)
                decompDepthImg_.ptr<float>(r)[c] = NaN;
        }
    }

    // Point-cloud Generation
    float  pcl_pts[3];
    for (int r = 0; r < decompDepthImg_.rows; r += param_.pcl_stride) {
        for (int c = 0; c < decompDepthImg_.cols; c += param_.pcl_stride) {
            float d = decompDepthImg_.ptr<float>(r)[c];
            if (not isnan(d))
            {
                pcl_pts[2] = (float) (d/ camera_factor);
                pcl_pts[0] = (float) ((c - camera_cx) * pcl_pts[2]/ camera_fx);
                pcl_pts[1] = (float) ((r - camera_cy) * pcl_pts[2] / camera_fy);
                // point cloud transformation
                Point p_w = state_.T_wc.poseMat * Point(pcl_pts[0], pcl_pts[1], pcl_pts[2]).toEigen();
                pcl::PointXYZ p;
                p.x = p_w.x;
                p.y = p_w.y;
                p.z = p_w.z;
                state_.pclObjectsRemoved.points.push_back(p);
            }
        }
    }
}

chasing_client::ChasingClient::ChasingClient() {}


void chasing_client::ChasingClient::setPose(const Pose &pose) {
    state_.T_wc = pose;
    state_.T_cw = state_.T_wc;
    state_.T_cw.inverse();
}

void chasing_client::ChasingClient::setObjPose(const std::vector<Pose>& pose) {
    state_.T_wo.clear();
    int num_object{0};
    for(int i =0;i<pose.size();i++){
        if(not (isnan(pose[i].getTranslation().x) or isnan(pose[i].getTranslation().y) or isnan(pose[i].getTranslation().z))){
            num_object++;
        }
    }
    if(param_.target_number<=num_object){
        static bool is_first_time = true;
        static std::vector<Pose> target_pose_temp(param_.target_number);
        if(is_first_time){
            for(int i =0;i<param_.target_number;i++){
                target_pose_temp[i] = pose[i];
                state_.T_wo.push_back(target_pose_temp[i]);
            }
            is_first_time = false;
        }
        else{
            for(int i =0;i<param_.target_number;i++){
                float distance_squared_array[param_.target_number];
                int index_array[param_.target_number];
                for(int j=0;j<param_.target_number;j++){
                    distance_squared_array[j] = 99999999.0f;
                    distance_squared_array[j] = -1;
                }
                float temp_distance_squared = 0.0f;
                for(int j =0;j<pose.size();j++){
                    temp_distance_squared = powf(target_pose_temp[i].getTranslation().x-pose[j].getTranslation().x,2)+
                            powf(target_pose_temp[i].getTranslation().y-pose[j].getTranslation().y,2)+
                            powf(target_pose_temp[i].getTranslation().z-pose[j].getTranslation().z,2);
                    if(temp_distance_squared<distance_squared_array[i] and temp_distance_squared<powf(param_.separate_threshold,2)){
                        distance_squared_array[i] = temp_distance_squared;
                        index_array[i]=j;
                    }
                }
                target_pose_temp[i] = pose[index_array[i]];
                state_.T_wo.push_back(pose[index_array[i]]);
            }
        }
    }
}

void chasing_client::ChasingClient::setDecompDepth(const cv::Mat &decompDepth_) {
    decompDepthImg_ = decompDepth_;
}

void chasing_client::ChasingClient::setParam(const chasing_client::Param &param) {
    param_ = param;
}
