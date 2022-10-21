#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>

#include "TagDetector.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"

// #define BACKWARD_HAS_DW 1
// #include "backward.hpp"
// namespace backward
// {
// backward::SignalHandling sh;
// }

// Finite state machine
// VIO-->TAG-->TAG2VIO--
//        ^            |
//        |            |
//        --------------
enum FEEDBACK_STATE
{
    VIO,
    TAG,
    TAG2VIO,
};

FEEDBACK_STATE fb_state;

ros::Publisher odom_pub, pose_pub, tag_pose_pub;

Eigen::Affine3f T_W_B0, T_W_Bt, T_W_Bk;

Eigen::Affine3f T_C0_A, T_Ck_A;

Eigen::Affine3f T_W_BA0, T_W_BAk, T_W_BAt;

cv::Mat         intrinsic;
cv::Mat         distort;
Eigen::Affine3f T_B_C;

void odom_callback(const nav_msgs::OdometryConstPtr &vio_msg)
{
    Eigen::Vector3f    p(vio_msg->pose.pose.position.x, vio_msg->pose.pose.position.y,
                         vio_msg->pose.pose.position.z);
    Eigen::Quaternionf q(vio_msg->pose.pose.orientation.w, vio_msg->pose.pose.orientation.x,
                         vio_msg->pose.pose.orientation.y, vio_msg->pose.pose.orientation.z);

    T_W_Bt.matrix().block<3, 3>(0, 0) = q.toRotationMatrix();
    T_W_Bt.matrix().block<3, 1>(0, 3) = p;

    if (fb_state == VIO)
    {
        T_W_BAt = T_W_Bt;
        odom_pub.publish(vio_msg);
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = vio_msg->header;
        pose_msg.pose   = vio_msg->pose.pose;
        pose_pub.publish(pose_msg);
    }
    else if (fb_state == TAG)
    {
        Eigen::Affine3f T_B0_BAk      = T_B_C * T_C0_A * T_Ck_A.inverse() * T_B_C.inverse();
        T_W_BAk                       = T_W_BA0 * T_B0_BAk;
        Eigen::Affine3f T_BAk_BAt     = T_W_Bk.inverse() * T_W_Bt;
        T_W_BAt                       = T_W_BAk * T_BAk_BAt;
        nav_msgs::Odometry odom_msg   = *vio_msg;
        odom_msg.pose.pose.position.x = T_W_BAt.translation().x();
        odom_msg.pose.pose.position.y = T_W_BAt.translation().y();
        odom_msg.pose.pose.position.z = T_W_BAt.translation().z();
        // 不用位姿补偿，可能频率不够
        odom_pub.publish(odom_msg);
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = odom_msg.header;
        pose_msg.pose   = odom_msg.pose.pose;
        pose_pub.publish(pose_msg);
    }
    else if (fb_state == TAG2VIO)
    {
        T_W_BAt                       = T_W_BA0 * T_W_B0.inverse() * T_W_Bt;
        nav_msgs::Odometry odom_msg   = *vio_msg;
        odom_msg.pose.pose.position.x = T_W_BAt.translation().x();
        odom_msg.pose.pose.position.y = T_W_BAt.translation().y();
        odom_msg.pose.pose.position.z = T_W_BAt.translation().z();
        // 不用位姿补偿，可能频率不够
        odom_pub.publish(odom_msg);
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = odom_msg.header;
        pose_msg.pose   = odom_msg.pose.pose;
        pose_pub.publish(pose_msg);
    }
}

std::vector<std::vector<cv::Point3f>> tag_coord;
void                                  img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv::Mat fisheye_img;
    try
    {
        fisheye_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    int tag_len = TagDetector(fisheye_img, tag_coord, intrinsic, distort, T_Ck_A);

    // std::cout << tag_len << std::endl;
    // std::cout << T_Ct_A.matrix() << std::endl;
    if (tag_len > 0)  // TODO: 添加距离判断？（ < 2m）
    {
        Eigen::Affine3f    T_W_A = T_W_BAt * T_B_C * T_Ck_A;
        nav_msgs::Odometry tag_pose;
        tag_pose.header.frame_id      = "map";
        tag_pose.header.stamp         = img_msg->header.stamp;
        tag_pose.pose.pose.position.x = T_W_A.translation().x();
        tag_pose.pose.pose.position.y = T_W_A.translation().y();
        tag_pose.pose.pose.position.z = T_W_A.translation().z();
        Eigen::Quaternionf q(T_W_A.matrix().block<3, 3>(0, 0));
        tag_pose.pose.pose.orientation.w = q.w();
        tag_pose.pose.pose.orientation.x = q.x();
        tag_pose.pose.pose.orientation.y = q.y();
        tag_pose.pose.pose.orientation.z = q.z();
        tag_pose_pub.publish(tag_pose);
        if (fb_state == VIO)
        {
            T_W_BA0    = T_W_BAt;
            T_W_Bk     = T_W_Bt;
            T_C0_A     = T_Ck_A;
            fb_state = TAG;

            // std::cout << "T_W_V0.matrix()" << std::endl << T_W_B0.matrix() << std::endl;
            // std::cout << "T_C0_A.matrix()" << std::endl << T_C0_A.matrix() << std::endl;
            // std::cout << gFuseState << std::endl;
        }
        else if (fb_state == TAG)
        {
            T_W_Bk = T_W_Bt;
        }
        else if (fb_state == TAG2VIO)
        {
            T_W_BA0    = T_W_BAt;
            T_W_Bk     = T_W_Bt;
            T_C0_A     = T_Ck_A;
            fb_state = TAG;
            // std::cout << "T_W_V0.matrix()" << std::endl << T_W_B0.matrix() << std::endl;
            // std::cout << "T_C0_A.matrix()" << std::endl << T_C0_A.matrix() << std::endl;
            // std::cout << gFuseState << std::endl;
        }
    }
    else if (fb_state == TAG)
    {
        T_W_B0     = T_W_Bt;
        T_W_BA0    = T_W_BAt;
        fb_state = TAG2VIO;
        // std::cout << "T_W_V0.matrix()" << std::endl << T_W_B0.matrix() << std::endl;
        // std::cout << "T_W_VA0.matrix()" << std::endl << T_W_BA0.matrix() << std::endl;
        // std::cout << gFuseState << std::endl;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tag_aided_loc");
    ros::NodeHandle nh("~");

    ros::Subscriber vio_sub =
        nh.subscribe<nav_msgs::Odometry>("/vins_estimator/imu_propagate", 100, odom_callback,
                                         ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    ros::Subscriber img_sub =
        nh.subscribe<sensor_msgs::Image>("/usb_cam/image_raw", 100, img_callback);

    odom_pub     = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    pose_pub     = nh.advertise<geometry_msgs::PoseStamped>("/pose", 100);
    tag_pose_pub = nh.advertise<nav_msgs::Odometry>("/tag_pose", 100);

    int  tag_size = 15;
    bool ret =
        LoadBoard("/home/chrisliu/FEIBI_ws/src/tag_aided_loc/board.txt", tag_size, tag_coord);
    // LoadBoard("/home/nros/FEIBI_ws/src/tag_aided_loc/board.txt", tag_size, tag_coord);

    fb_state = VIO;
    intrinsic  = (cv::Mat_<float>(3, 3) << 393.07800238, 0, 319.65949468, 0, 393.22628119,
                 240.06435046, 0, 0, 1);
    distort    = (cv::Mat_<float>(1, 4) << 0.012564748398953971, -0.01475250908709762,
               0.0030947317078050427, 0.0006145522119168964);

    // TODO: 读取外参
    T_B_C.matrix() << -0.01860761, 0.99975974, 0.01158533, -0.10637797, 0.99939245, 0.01825674,
        0.02968864, 0.0111378, 0.02947, 0.01213072, -0.99949205, -0.14963229, 0., 0., 0., 1.;

    ros::spin();
    return 0;
}