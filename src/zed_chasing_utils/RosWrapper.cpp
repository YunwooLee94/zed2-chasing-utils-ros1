//
// Created by larr-desktop on 22. 7. 22.
//
#include<zed_chasing_utils/RosWrapper.h>

Pose RosWrapper::getPoseFromGeoMsgs(const geometry_msgs::PoseStamped & poseStamped){
    Pose tempPose;
    tempPose.poseMat.setIdentity();
    Eigen::Vector3f loc((float) poseStamped.pose.position.x,
                        (float) poseStamped.pose.position.y,
                        (float) poseStamped.pose.position.z);
    tempPose.poseMat.translate(loc);
    Eigen::Quaternionf quaternionf;
    quaternionf.setIdentity();
    quaternionf.w() = (float) poseStamped.pose.orientation.w;
    quaternionf.x() = (float) poseStamped.pose.orientation.x;
    quaternionf.y() = (float) poseStamped.pose.orientation.y;
    quaternionf.z() = (float) poseStamped.pose.orientation.z;
    tempPose.poseMat.rotate(quaternionf);
    return tempPose;
}
Pose RosWrapper::getPoseFromTfMsgs(const tf::StampedTransform& tfStamped){
    Pose tempPose;
    tempPose.poseMat.setIdentity();
    Eigen::Vector3f loc((float)tfStamped.getOrigin().x(),(float)tfStamped.getOrigin().y(),(float)tfStamped.getOrigin().z());
    tempPose.poseMat.translate(loc);
    Eigen::Quaternionf quaternionf;
    quaternionf.setIdentity();
    quaternionf.w() = (float) tfStamped.getRotation().w();
    quaternionf.x() = (float) tfStamped.getRotation().x();
    quaternionf.y() = (float) tfStamped.getRotation().y();
    quaternionf.z() = (float) tfStamped.getRotation().z();
    tempPose.poseMat.rotate(quaternionf);
}

tf::StampedTransform RosWrapper::toTF(const Pose &pose, const string &worldFrameName, const string &frameName, const ros::Time &time){
    tf::StampedTransform stampedTransform;
    stampedTransform.frame_id_ = worldFrameName;
    stampedTransform.child_frame_id_ = frameName;
    stampedTransform.stamp_ = time;
    stampedTransform.setIdentity();

    tf::Vector3 vec(pose.poseMat.translation().x(),pose.poseMat.translation().y(),pose.poseMat.translation().z());
    stampedTransform.setOrigin(vec);

    Eigen::Quaternionf quatt(pose.poseMat.rotation());
    tf::Quaternion quat(quatt.x(),quatt.y(),quatt.z(),quatt.w());
    stampedTransform.setRotation(quat);
    return stampedTransform;
}


RosWrapper::RosWrapper():nh_("~"), it(nh_) {
    nh_.param<std::string>("global_frame_id",global_frame_id,"map");
    nh_.param<int>("pcl_stride",pcl_stride,2);
    nh_.param<int>("mask_padding_x",mask_padding_x,10);
    nh_.param<int>("mask_padding_y",mask_padding_y,10);
    cc.setParam(global_frame_id,pcl_stride,mask_padding_x,mask_padding_y);

    subDepthComp = new message_filters::Subscriber<sensor_msgs::CompressedImage>(nh_,"/zed2i/zed_node/depth/depth_registered/compressedDepth",1);
    subCamInfo = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_,"/zed2i/zed_node/rgb/camera_info",1);
    subZedOd = new message_filters::Subscriber<zed_interfaces::ObjectsStamped>(nh_,"/zed2i/zed_node/obj_det/objects",1);
    {
        subSync = new message_filters::Synchronizer<CompressedImageMaskBbSync>(CompressedImageMaskBbSync(10),*this->subDepthComp,*this->subCamInfo, *this->subZedOd);
        subSync->registerCallback(boost::bind(&RosWrapper::zedSyncCallback,this, _1,_2,_3));
    }

    pubDepthMaskImg = it.advertise("image_depth_masekd",1);
    pubPointsCorrection = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("points_corrected",1);
    pubPointsMasked = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("points_masked",1);

    isCameraPoseReceived = false;
    isObjectPoseReceived = false;
    isDepthImageReceived = false;
    isPclCreated = false;


    tfListenerPtr = new tf::TransformListener;
    tfBroadcasterPtr = new tf::TransformBroadcaster;

}

void RosWrapper::run() {

}

void
RosWrapper::zedSyncCallback(const sensor_msgs::CompressedImageConstPtr &compDepthImgPtr, const sensor_msgs::CameraInfoConstPtr &cameraInfoPtr,
                               const zed_interfaces::ObjectsStampedConstPtr &objPtr) {
        cc.setPose(this->tfCallBack(compDepthImgPtr));
        if(isCameraPoseReceived){
            cc.setObjPose(this->tfObjCallBack(objPtr));
            if(isObjectPoseReceived){
                cc.setDecompDepth(this->pngDecompressDepth(compDepthImgPtr));
                cc.setDepthFrameId(this->getDepthImageFrameId(compDepthImgPtr));
                cc.setDepthTimeStamp(this->getDepthImageTimeStamp(compDepthImgPtr));
                if(isDepthImageReceived){
                    cc.depthCallback(cameraInfoPtr, objPtr);
                    if(isPclCreated){
                        
                    }
                }
            }
        }
}

Pose RosWrapper::tfCallBack(const sensor_msgs::CompressedImageConstPtr &compDepthImgPtr) {
    ros::Time curSensorTime = compDepthImgPtr->header.stamp;
    zedCallTime = curSensorTime;

//    double fps = 1.0 / (curSensorTime - this->zedLastCallTime).toSec();
    tf::StampedTransform transform_temp;
    try {
        // time 0 in lookup was intended
        tfListenerPtr->lookupTransform(global_frame_id, compDepthImgPtr->header.frame_id,
                                       curSensorTime, transform_temp);
        isCameraPoseReceived = true;
        return getPoseFromTfMsgs(transform_temp);
    }catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM(ex.what());
        ROS_ERROR("[ZedClient] no transform between map and object header. Cannot process further.");
        Pose dummy;
        dummy.setTranslation(0.0,0.0,0.0);
        dummy.setRotation(Eigen::Quaternionf (1.0, 0.0, 0.0, 0.0));
        isCameraPoseReceived = false;
        return dummy;
    }
}

Pose RosWrapper::tfObjCallBack(const zed_interfaces::ObjectsStampedConstPtr & objPtr) {
    ros::Time curObjTime = objPtr->header.stamp;
    tf::StampedTransform transform_temp;
    try {
        // time 0 in lookup was intended
        tfListenerPtr->lookupTransform(global_frame_id, objPtr->header.frame_id,
                                       curObjTime, transform_temp);
        isObjectPoseReceived = true;
        return getPoseFromTfMsgs(transform_temp);
    }catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM(ex.what());
        ROS_ERROR("[ZedClient] no transform between map and object header. Cannot process further.");
        Pose dummy;
        dummy.setTranslation(0.0,0.0,0.0);
        dummy.setRotation(Eigen::Quaternionf (1.0, 0.0, 0.0, 0.0));
        isObjectPoseReceived = false;
        return dummy;
    }


}

cv::Mat RosWrapper::pngDecompressDepth(const sensor_msgs::CompressedImageConstPtr &depthPtr) {

    cv::Mat decompressedTemp;
    cv::Mat decompressed;
    const size_t split_pos = depthPtr->format.find(';');
    const std::string image_encoding =depthPtr->format.substr(0, split_pos);

    if (depthPtr->data.size() > sizeof(compressed_depth_image_transport::ConfigHeader))
    {
//        Timer timer;
        // Read compression type from stream
        compressed_depth_image_transport::ConfigHeader compressionConfig{};
        memcpy(&compressionConfig, &depthPtr->data[0], sizeof(compressionConfig));
        const std::vector<uint8_t> imageData(depthPtr->data.begin() + sizeof(compressionConfig),depthPtr->data.end());

        if (enc::bitDepth(image_encoding) == 32)
            try{
                decompressedTemp = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);

            }
            catch (cv::Exception& e){
                ROS_ERROR("%s", e.what());
                isDepthImageReceived = false;
                return (cv::Mat(1,1, CV_32FC1));
//                return false ;
            }
        size_t rows = decompressedTemp.rows;
        size_t cols = decompressedTemp.cols;


        if ((rows > 0) && (cols > 0)) {
            decompressed = cv::Mat(rows, cols, CV_32FC1);

            // Depth conversion
            auto itDepthImg = decompressed.begin<float>(),
                    itDepthImg_end = decompressed.end<float>();
            auto itInvDepthImg = decompressedTemp.begin<unsigned short>(),
                    itInvDepthImg_end = decompressedTemp.end<unsigned short>();

            float depthQuantA = compressionConfig.depthParam[0];
            float depthQuantB = compressionConfig.depthParam[1];

            for (; (itDepthImg != itDepthImg_end) &&
                   (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg) {
                // check for NaN & max depth
                if (*itInvDepthImg) {
                    *itDepthImg = depthQuantA / ((float) *itInvDepthImg - depthQuantB);
                } else {
                    *itDepthImg = std::numeric_limits<float>::quiet_NaN();
                }
            }
//            double elapseDecomp = timer.stop();
//            ROS_DEBUG("depth decomp took %f ms", elapseDecomp);
            isDepthImageReceived = true;
            return decompressed;
        }
        else
        {
            isDepthImageReceived = false;
            return (cv::Mat(1,1, CV_32FC1));
        }
    }
    else
    {
        isDepthImageReceived = false;
        return (cv::Mat(1,1, CV_32FC1));
    }
}

string RosWrapper::getDepthImageFrameId(const sensor_msgs::CompressedImageConstPtr &depthCompPtr) {
    return depthCompPtr->header.frame_id;
}

uint64_t RosWrapper::getDepthImageTimeStamp(const sensor_msgs::CompressedImageConstPtr &depthCompPtr) {
    return (uint64_t)(depthCompPtr->header.stamp.toNSec()/1000ull);
}

