//
// Created by larr-desktop on 22. 7. 22.
//
#include <zed_chasing_utils/RosWrapper.h>

int main(int argc, char * argv[]){
    ros::init(argc,argv,"zed_client");
    RosWrapper zed_client_wrapper;
    zed_client_wrapper.run();
}