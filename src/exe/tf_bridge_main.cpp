//
// Created by larr-desktop on 23. 10. 17.
//
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
void publishTF(const ros::Time &time) {
  static tf::TransformBroadcaster br;
  { // left optical frame
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0.06, 0));
    tf::Quaternion q{0.5, -0.5, 0.5, -0.5};
    transform.setRotation(q);
    br.sendTransform(
        tf::StampedTransform(transform, time, "base_link", "zed2_left_camera_optical_frame"));
  }
  {
    // left lense frame
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0.06, 0));
    tf::Quaternion q{0.0, 0.0, 0.0, 1.0};
    transform.setRotation(q);
    br.sendTransform(
        tf::StampedTransform(transform, time, "base_link", "zed2_left_camera_frame"));
  }
}
void cameraInfoCallback(const sensor_msgs::CameraInfo &camera_info) {
  publishTF(camera_info.header.stamp);
}
void targetInfoCallback(const geometry_msgs::PointStamped &target_info) {
  publishTF(target_info.header.stamp);
}
void rawPublishTF() {
  static tf::TransformBroadcaster br;
  { // optical frame
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0.06, 0));
    tf::Quaternion q{0.5, -0.5, 0.5, -0.5};
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "base_link",
                                          "zed2_left_camera_optical_frame"));
  }
  { // left lense frame
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0.06, 0));
    tf::Quaternion q{0.0, 0.0, 0.0, 1.0};
    transform.setRotation(q);
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time(0), "base_link", "zed2_left_camera_frame"));
  }
}
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "zed_tf_bridge");
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(200.0);
  ros::Subscriber subCameraInfo;
  ros::Subscriber subTargetInfo;
  subCameraInfo = nh.subscribe("/zed2/zed_node/rgb/camera_info", 1, cameraInfoCallback);
  //  subTargetInfo = nh.subscribe("/zed_client/object1_position",1,targetInfoCallback);
  while (ros::ok()) {
    //    rawPublishTF();
    ros::spinOnce();
    loop_rate.sleep();
  }
}