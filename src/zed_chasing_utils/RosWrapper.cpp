//
// Created by larr-desktop on 22. 7. 22.
//
#include<zed_chasing_utils/RosWrapper.h>

Pose getPoseFromGeoMsgs(const geometry_msgs::PoseStamped & poseStamped){
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
Pose getPoseFromTfMsgs(const tf::StampedTransform& tfStamped){
    Pose tempPose;
    tempPose.poseMat.setIdentity();
    Eigen::Vector3f loc(tfStamped.getOrigin().x(),tfStamped.getOrigin().y(),tfStamped.getOrigin().z());
    tempPose.poseMat.translate(loc);
    Eigen::Quaternionf quaternionf;
    quaternionf.setIdentity();
    quaternionf.w() = tfStamped.getRotation().w();
    quaternionf.x() = tfStamped.getRotation().x();
    quaternionf.y() = tfStamped.getRotation().y();
    quaternionf.z() = tfStamped.getRotation().z();
    tempPose.poseMat.rotate(quaternionf);
}

tf::StampedTransform toTF(const Pose &pose, const string &worldFrameName, const string &frameName, const ros::Time &time){
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

    tf::TransformListener* tfListenerPtr;
    tf::TransformBroadcaster* tfBroadcasterPtr;

}

void RosWrapper::run() {

}

void
RosWrapper::zedSyncCallback(const sensor_msgs::CompressedImageConstPtr &compDepthImgPtr, const sensor_msgs::CameraInfoConstPtr &cameraInfoPtr,
                               const zed_interfaces::ObjectsStampedConstPtr &objPtr) {
        cc.depthCallback(compDepthImgPtr, cameraInfoPtr, objPtr);
}
