//
// Created by larr-desktop on 22. 7. 22.
//
#include<zed_chasing_utils/RosWrapper.h>

RosWrapper::RosWrapper():nh_("~") {
    ChasingClient cc(nh_);
    nh_.param<std::string>("global_frame_id",global_frame_id,"map");

    subDepthComp = new message_filters::Subscriber<sensor_msgs::CompressedImage>(nh_,"/zed2i/zed_node/depth/depth_registered/compressedDepth",1);
    subCamInfo = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_,"/zed2i/zed_node/rgb/camera_info",1);
    {
        subSync = new message_filters::Synchronizer<CompressedImageMaskBbSync>(CompressedImageMaskBbSync(10),*this->subDepthComp,*this->subCamInfo);
//        subSync->registerCallback(boost::bind())
    }


}

void RosWrapper::run() {

}
