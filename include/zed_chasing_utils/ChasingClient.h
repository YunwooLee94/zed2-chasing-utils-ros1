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

#include <compressed_depth_image_transport/codec.h>
#include <compressed_depth_image_transport/compression_common.h>
#include <compressed_image_transport/compression_common.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <numeric>
#include <vector>

namespace chasing_client{
    struct Param{
        std::string global_frame_id;
        int pcl_stride;
        int mask_padding_x;
        int mask_padding_y;
        int target_number;
        float separate_threshold;
        float min_depth;
        bool do_target_mock;
    };
    class ChasingClient{
    private:
        struct {
            bool isTargetTracked = false;
            Pose T_cw; // world to cam (optical)
            Pose T_cd; // zed cam to rear camera frame
            Pose T_wc;
            std::vector<Pose> T_wo; // world to object (x-forwarding)
            pcl::PointCloud<pcl::PointXYZ> pclObjectsRemoved;
            int bbox_2d[4]; //rMin, cMin, rMax, cMax
            ros::Time zedLastCallTime;
            ros::Time clientLastCallTime;
        }state_;
        Param param_;
        cv::Mat decompDepthImg_;
        cv::Mat depthImgMasked_;
        std::string depthImgFrameId_;
        u_int64_t depthImgTimeStamp;
    public:
        ChasingClient();
        void setParam(const chasing_client::Param & param);
        void depthCallback(const sensor_msgs::CameraInfoConstPtr &cameraInfoPtr, const zed_interfaces::ObjectsStampedConstPtr &zedOdPtr);
        void depthCallbackMock(const sensor_msgs::CameraInfoConstPtr &cameraInfoPtr);

        void setPose(const Pose & pose);
        void setObjPose(const std::vector<Pose> & pose);
        void setDecompDepth(const cv::Mat & decompDepth_);
        void setDepthFrameId(const std::string &frame_id) {
            depthImgFrameId_ = frame_id;
        }
        void setDepthTimeStamp(const u_int64_t &time_stamp) {
            depthImgTimeStamp = time_stamp;
        }
        cv::Mat getMaskedImage() {
            return decompDepthImg_;
        }
        pcl::PointCloud<pcl::PointXYZ> getMaskedPointCloud(){
            return state_.pclObjectsRemoved;
        }
        Pose getCameraPose() {
            return state_.T_wc;
        }
        std::vector<Pose> getObjectPose(){
            return state_.T_wo;
        }
    };
}



#endif //ZED_CHASING_UTILS_CHASINGCLIENT_H
