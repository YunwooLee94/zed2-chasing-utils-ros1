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
    return tempPose;
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
//    std::cout<<"mask_padding_x: "<<mask_padding_x<<" mask_padding_y: "<<mask_padding_y<<std::endl;
    subDepthComp = new message_filters::Subscriber<sensor_msgs::CompressedImage>(nh_,"/zed2i/zed_node/depth/depth_registered/compressedDepth",1);
    subCamInfo = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_,"/zed2i/zed_node/rgb/camera_info",1);
    subZedOd = new message_filters::Subscriber<zed_interfaces::ObjectsStamped>(nh_,"/zed2i/zed_node/obj_det/objects",1);
    {
        subSync = new message_filters::Synchronizer<CompressedImageMaskBbSync>(CompressedImageMaskBbSync(10),*this->subDepthComp,*this->subCamInfo, *this->subZedOd);
        subSync->registerCallback(boost::bind(&RosWrapper::zedSyncCallback,this, _1,_2,_3));
    }
    pubPointsMasked = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("points_masked",1);
    pubDepthMaskImg = it.advertise("image_depth_masked",1);
    pubObjectPos = nh_.advertise<geometry_msgs::PointStamped>("object_pos",1);
//    pubPointsCorrection = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("points_corrected",1);
//    pubCameraPose = nh_.advertise<geometry_msgs::PoseStamped>("camera_pose",1);

    isCameraPoseReceived = false;
    isObjectPoseReceived = false;
    isDepthImageReceived = false;
    isPclCreated = false;


    tfListenerPtr = new tf::TransformListener;
    tfBroadcasterPtr = new tf::TransformBroadcaster;

}

void RosWrapper::run() {
    ros::Rate loop_rate(20.0);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void
RosWrapper::zedSyncCallback(const sensor_msgs::CompressedImageConstPtr &compDepthImgPtr, const sensor_msgs::CameraInfoConstPtr &cameraInfoPtr,
                               const zed_interfaces::ObjectsStampedConstPtr &objPtr) {
    cc.setPose(this->tfCallBack(compDepthImgPtr));
    cc.setObjPose(this->tfObjCallBack(objPtr));
    cc.setDecompDepth(this->pngDecompressDepth(compDepthImgPtr));
    cc.setDepthFrameId(this->getDepthImageFrameId(compDepthImgPtr));
    cc.setDepthTimeStamp(this->getDepthImageTimeStamp(compDepthImgPtr));
    cc.depthCallback(cameraInfoPtr, objPtr);

    if( (not isnan( cc.getObjectPose().getTranslation().x)) and (not isnan( cc.getObjectPose().getTranslation().y))
    and (not isnan( cc.getObjectPose().getTranslation().z))){
        pubObjectPos.publish(poseToGeoMsgsPoint(cc.getObjectPose()));
    }
    if(not cc.getMaskedPointCloud().points.empty())
    {
        pubPointsMasked.publish(cc.getMaskedPointCloud());
    }
//    std::cout<<"HHHH"<<std::endl;

    pubDepthMaskImg.publish(imageToROSmsg(cc.getMaskedImage(),enc::TYPE_32FC1, compDepthImgPtr->header.frame_id, compDepthImgPtr->header.stamp));

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
//        std::cout<<"x: "<<transform_temp.getOrigin().x()<<" y: "<<transform_temp.getOrigin().y()<<" z: "<<
//                 transform_temp.getOrigin().z()<<std::endl;
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
    float temp_x=0.0;
    float temp_y=0.0;
    float temp_z=0.0;
    bool isNanHeadPos = false;
//    std::cout<<"AAAAA"<<std::endl;
//    std::cout<<"ObjectPosition size: "<<objPtr->objects[0].head_position.size()<<std::endl;
    for(int idx=0;idx<objPtr->objects.size();idx++)
    {
        for(int i =0;i<3;i++){
//        std::cout<<"YYYYY"<<std::endl;
            if(isnan(objPtr->objects[idx].head_position.elems[i])){
                isNanHeadPos = true;
            }
        }
//    std::cout<<"SSSSS"<<std::endl;
        if(not isNanHeadPos){
            temp_x = objPtr->objects[idx].head_position.elems[0];
            temp_y = objPtr->objects[idx].head_position.elems[1];
            temp_z = objPtr->objects[idx].head_position.elems[2];
//        std::cout<<"TTTTT"<<std::endl;
        }
        else
        {
            bool isNanHeadBbox = false;
//        std::cout<<"BBBBB"<<std::endl;
            for(int i = 0;i<8;i++){
                for(int j = 0;j<3;j++){
                    if(isnan(objPtr->objects[idx].head_bounding_box_3d.corners.elems[i].kp[j]))
                    {
                        isNanHeadBbox = true;
                    }
                }
            }
//        std::cout<<"CCCCC"<<std::endl;
            if (not isNanHeadBbox)
            {
                vector<Point> head_bbox;
                Point tempCorners{0.0,0.0,0.0};
                for(auto elem : objPtr->objects[idx].head_bounding_box_3d.corners.elems){
                    tempCorners.x = elem.kp[0];
                    tempCorners.y = elem.kp[1];
                    tempCorners.z = elem.kp[2];
                    head_bbox.push_back(tempCorners);
                }

                for(auto & i : head_bbox){
                    temp_x += i.x;
                    temp_y += i.y;
                    temp_z += i.z;
                }
                temp_x = temp_x/(float)head_bbox.size();
                temp_y = temp_y/(float)head_bbox.size();
                temp_z = temp_z/(float)head_bbox.size();
            }
            else
            {
                bool isNanBodyBbox = false;
//            std::cout<<"DDDDD"<<std::endl;
                for(int i = 0;i<8;i++){
                    for(int j =0; j<3; j++){
                        if(isnan(objPtr->objects[idx].bounding_box_3d.corners.elems[i].kp[j]))
                        {
                            isNanBodyBbox = true;
                        }
                    }
                }
                if(not isNanBodyBbox){
                    vector<Point> body_bbox;
                    Point tempCorners{0.0,0.0,0.0};
                    for(auto elem : objPtr->objects[idx].bounding_box_3d.corners.elems){
                        tempCorners.x = elem.kp[0];
                        tempCorners.y = elem.kp[1];
                        tempCorners.z = elem.kp[2];
                        body_bbox.push_back(tempCorners);
                    }

                    for(auto & i : body_bbox){
                        temp_x += i.x;
                        temp_y += i.y;
                        temp_z += i.z;
                    }
                    temp_x = temp_x/(float)body_bbox.size();
                    temp_y = temp_y/(float)body_bbox.size();
                    temp_z = temp_z/(float)body_bbox.size();
                }
                else
                {
                    const float NaN = std::numeric_limits<float>::quiet_NaN() ;
                    temp_x = NaN;
                    temp_y = NaN;
                    temp_z = NaN;
                }

            }
        }
    }
    if(objPtr->objects.empty())
    {
        const float NaN = std::numeric_limits<float>::quiet_NaN() ;
        temp_x = NaN;
        temp_y = NaN;
        temp_z = NaN;
    }

//    std::cout<<"PPPPP"<<std::endl;
    Pose tempPose;
    tempPose.setTranslation(temp_x,
                            temp_y,temp_z);
    tempPose.setRotation(Eigen::Quaternionf(1.0,0.0,0.0,0.0));
//    std::cout<<"QQQQQ"<<std::endl;
    return tempPose;
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

//        std::cout<<"DECOMP ROWS: "<<rows<<" DECOMP COLS: "<<cols<<std::endl;

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

sensor_msgs::ImagePtr
RosWrapper::imageToROSmsg(const cv::Mat &img, const std::string encodingType, std::string frameId, ros::Time t) {
    sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
    sensor_msgs::Image& imgMessage = *ptr;
    imgMessage.header.stamp = t;
    imgMessage.header.frame_id = frameId;
    imgMessage.height = img.rows;
    imgMessage.width = img.cols;
    imgMessage.encoding = encodingType;
    int num = 1; //for endianness detection
    imgMessage.is_bigendian = !(*(char *) &num == 1);
    imgMessage.step = img.cols * img.elemSize();
    size_t size = imgMessage.step * img.rows;
    imgMessage.data.resize(size);

    if (img.isContinuous())
        memcpy((char*) (&imgMessage.data[0]), img.data, size);
    else {
        uchar* opencvData = img.data;
        uchar* rosData = (uchar*) (&imgMessage.data[0]);
        for (unsigned int i = 0; i < img.rows; i++) {
            memcpy(rosData, opencvData, imgMessage.step);
            rosData += imgMessage.step;
            opencvData += img.step;
        }
    }
    return ptr;
}

geometry_msgs::PoseStamped RosWrapper::poseToGeoMsgs(const Pose &pose) {
    geometry_msgs::PoseStamped tempPose;
    tempPose.pose.position.x = pose.getTranslation().x;
    tempPose.pose.position.y = pose.getTranslation().y;
    tempPose.pose.position.z = pose.getTranslation().z;
//    std::cout<<"x: "<< tempPose.pose.position.x<<" y: "<<tempPose.pose.position.y
//    <<" z: "<<tempPose.pose.position.z<<std::endl;
    tempPose.pose.orientation.w = pose.getQuaternion().w();
    tempPose.pose.orientation.x = pose.getQuaternion().x();
    tempPose.pose.orientation.y = pose.getQuaternion().y();
    tempPose.pose.orientation.z = pose.getQuaternion().z();
//    std::cout<<"qx: "<<tempPose.pose.orientation.x<<" qy: "<<tempPose.pose.orientation.y<<" qz: "<<tempPose.pose.orientation.z<<
//    " qw: "<<tempPose.pose.orientation.w<<std::endl;
    tempPose.header.frame_id = global_frame_id;
    return tempPose;
}

geometry_msgs::PointStamped RosWrapper::poseToGeoMsgsPoint(const Pose &pose) {
    geometry_msgs::PointStamped tempPoint;
    tempPoint.point.x = pose.getTranslation().x;
    tempPoint.point.y = pose.getTranslation().y;
    tempPoint.point.z = pose.getTranslation().z;
    tempPoint.header.frame_id = global_frame_id;
    return tempPoint;
}

