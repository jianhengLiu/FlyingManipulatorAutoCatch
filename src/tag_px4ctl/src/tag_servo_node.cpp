#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>

#include "TagDetector.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "ros/init.h"
#include "sensor_msgs/Image.h"

// #define BACKWARD_HAS_DW 1
// #include "backward.hpp"
// namespace backward
// {
// backward::SignalHandling sh;
// }

using namespace std;

#define SIM 1

// Finite state machine
// VIO<-->TAG
enum FEEDBACK_STATE
{
    VIO,
    TAG
};

enum MODE
{
    INIT,
    POSITION,
    TAG_SERVO,
    LAND
};

FEEDBACK_STATE fb_state;

MODE mode;

mavros_msgs::State px4_state, px4_state_prev;

#define DEAD_ZONE 0.25
#define MAX_MANUAL_VEL 1.0
#define RC_REVERSE_PITCH 0
#define RC_REVERSE_ROLL 0
#define RC_REVERSE_THROTTLE 0
#define TAKEOFF_ALTITUDE 0.5
double last_set_hover_pose_time;

ros::Publisher     target_pose_pub, tag_pose_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

Eigen::Affine3f T_W_B0, T_W_Bt;

cv::Mat         intrinsic;
cv::Mat         distort;
Eigen::Affine3f T_B_C;

std::vector<std::vector<cv::Point3f>> tag_coord;

// 0: 初始时刻； k：看到TAG时刻； t:实时
Eigen::Affine3f T_B0_DES;
Eigen::Affine3f T_W_DES, T_W_DES_prev;
Eigen::Affine3f T_W_TAGk, T_W_TAG;

Eigen::Affine3f T_Ck_TAG;

void odom_callback(const geometry_msgs::PoseStampedConstPtr &odom_msg)
{
    Eigen::Vector3f    p(odom_msg->pose.position.x, odom_msg->pose.position.y,
                         odom_msg->pose.position.z);
    Eigen::Quaternionf q(odom_msg->pose.orientation.w, odom_msg->pose.orientation.x,
                         odom_msg->pose.orientation.y, odom_msg->pose.orientation.z);

    T_W_Bt.matrix().block<3, 3>(0, 0) = q.toRotationMatrix();
    T_W_Bt.matrix().block<3, 1>(0, 3) = p;
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
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
    int tag_len = TagDetector(fisheye_img, tag_coord, intrinsic, distort, T_Ck_TAG);

    if (tag_len > 2)
    {
        T_W_TAG = T_W_Bt * T_B_C * T_Ck_TAG;

        geometry_msgs::PoseStamped pose;
        pose.header.stamp    = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = T_W_TAG.translation().x();
        pose.pose.position.y = T_W_TAG.translation().y();
        pose.pose.position.z = T_W_TAG.translation().z();
        Eigen::Quaternionf q(T_W_TAG.matrix().block<3, 3>(0, 0));
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();

        tag_pose_pub.publish(pose);
        if (fb_state == VIO)
        {
            T_W_TAGk = T_W_TAG;
            ROS_INFO("Feeback state: TAG");
            fb_state = TAG;
        }
    }
    else if (fb_state == TAG)
    {
        Eigen::Vector3f p_B0_DES   = (T_W_B0.inverse() * T_W_DES).translation();
        T_B0_DES.translation().x() = p_B0_DES.x();
        T_B0_DES.translation().y() = p_B0_DES.y();
        fb_state                   = VIO;
        ROS_INFO("Feeback state: VIO");
    }
}

void rc_callback(const mavros_msgs::RCInConstPtr &rc_msg)
{
    double rc_ch[4];
    for (int i = 0; i < 4; i++)
    {
        // 归一化遥控器输入
        rc_ch[i] = ((double)rc_msg->channels[i] - 1500.0) / 500.0;
        if (rc_ch[i] > DEAD_ZONE)
            rc_ch[i] = (rc_ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (rc_ch[i] < -DEAD_ZONE)
            rc_ch[i] = (rc_ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            rc_ch[i] = 0.0;
    }

    if (rc_msg->channels[4] < 1250)
    {
        if (px4_state.mode != "STABILIZED")
        {
            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "STABILIZED";
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Switch to STABILIZED!");
                px4_state_prev      = px4_state;
                px4_state_prev.mode = "STABILIZED";
            }
            else
            {
                ROS_WARN("Failed to enter STABILIZED!");
                return;
            }
        }
        mode = INIT;
    }
    else if (rc_msg->channels[4] > 1250 && rc_msg->channels[4] < 1750)  // heading to POSITION
    {
        if (mode == INIT)
        {
            if (rc_ch[0] == 0.0 && rc_ch[1] == 0.0 && rc_ch[2] == 0.0 && rc_ch[3] == 0.0)
            {
                T_W_B0                   = T_W_Bt;
                mode                     = POSITION;
                last_set_hover_pose_time = ros::Time::now().toSec();
                T_B0_DES                 = Eigen::Affine3f::Identity();
                T_B0_DES.translation()   = Eigen::Vector3f(0.0, 0.0, TAKEOFF_ALTITUDE);
                T_W_TAG                  = Eigen::Affine3f::Identity();
                T_W_DES_prev             = T_W_B0 * T_B0_DES;
                ROS_INFO("Switch to POSITION succeed!");
            }
            else
            {
                ROS_WARN("Switch to POSITION failed! Rockers are not in reset middle!");
                return;
            }
        }
        else if (mode == TAG_SERVO)
        {
            Eigen::Vector3f p_B0_DES   = (T_W_B0.inverse() * T_W_DES).translation();
            T_B0_DES.translation().x() = p_B0_DES.x();
            T_B0_DES.translation().y() = p_B0_DES.y();
            mode                       = POSITION;
            ROS_INFO("Swith to POSITION succeed!");
        }
    }
    else if (rc_msg->channels[4] > 1750)
    {
        if (mode == POSITION)
        {
            mode = TAG_SERVO;
            ROS_INFO("Swith to TAG_SERVO succeed!");
        }
    }

    if (!SIM)
    {
        if (rc_msg->channels[5] > 1750)
        {
            if (mode == POSITION)
            {
                if (rc_ch[0] == 0.0 && rc_ch[1] == 0.0 && rc_ch[2] == 0.0 && rc_ch[3] == 0.0 &&
                    !px4_state.armed)
                {
                    if (px4_state.mode != "OFFBOARD")
                    {
                        mavros_msgs::SetMode offb_set_mode;
                        offb_set_mode.request.custom_mode = "OFFBOARD";
                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                        {
                            ROS_INFO("Offboard enabled");
                            px4_state_prev      = px4_state;
                            px4_state_prev.mode = "OFFBOARD";
                        }
                        else
                        {
                            ROS_WARN("Failed to enter OFFBOARD!");
                            return;
                        }
                    }
                    else if (px4_state.mode == "OFFBOARD")
                    {
                        mavros_msgs::CommandBool arm_cmd;
                        arm_cmd.request.value = true;

                        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                        {
                            ROS_INFO("Vehicle armed");
                        }
                        else
                        {
                            ROS_ERROR("Failed to armed");
                            return;
                        }
                    }
                }
                else if (!px4_state.armed)
                {
                    ROS_WARN("Arm denied! Rockers are not in reset middle!");
                    return;
                }
            }
        }
        else if (rc_msg->channels[5] > 1250 && rc_msg->channels[5] < 1750)
        {
            if (px4_state_prev.mode == "OFFBOARD")
            {
                // AUTO.LAND 会导致降落时机头变化，待排查
                // mavros_msgs::SetMode offb_set_mode;
                // offb_set_mode.request.custom_mode = "AUTO.LAND";
                // if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                // {
                //     ROS_INFO("AUTO.LAND enabled");
                //     px4_state_prev      = px4_state;
                //     px4_state_prev.mode = "AUTO.LAND";
                // }
                // else
                // {
                //     ROS_WARN("Failed to enter AUTO.LAND!");
                //     return;
                // }
                mode = LAND;
            }
        }
        else if (rc_msg->channels[5] < 1250)
        {
            if (px4_state.armed)
            {
                mavros_msgs::CommandBool arm_cmd;
                arm_cmd.request.value = false;

                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle disarmed");
                }
                else
                {
                    ROS_ERROR("Failed to disarmed");
                    return;
                }
                fb_state = VIO;
                mode     = INIT;
                ROS_INFO("Swith to INIT state!");
            }
        }
    }

    if (mode != INIT)
    {
        double now               = ros::Time::now().toSec();
        double delta_t           = now - last_set_hover_pose_time;
        last_set_hover_pose_time = now;

        // body frame: x-forward, y-left, z-up
        T_B0_DES.translation().x() +=
            rc_ch[1] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_PITCH ? -1 : 1);
        T_B0_DES.translation().y() +=
            rc_ch[3] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_ROLL ? 1 : -1);
        if (mode == LAND)
        {
            T_B0_DES.translation().z() -= 0.2 * delta_t;
        }
        else
        {
            T_B0_DES.translation().z() +=
                rc_ch[2] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_THROTTLE ? -1 : 1);
        }
        // hover_pose(3) +=
        //     rc_ch[3] * param.max_manual_vel * delta_t * (param.rc_reverse.yaw ? 1 : -1);

        if (T_B0_DES.translation().z() < -0.3)
            T_B0_DES.translation().z() = -0.3;
        else if (T_B0_DES.translation().z() > 1.0)
            T_B0_DES.translation().z() = 1.0;
    }
}
void px4_state_callback(const mavros_msgs::StateConstPtr &state_msg)
{
    px4_state = *state_msg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tag_positioning");
    ros::NodeHandle nh("~");
    ros::Rate       rate(30);

    ros::Subscriber odom_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/vision_pose/pose", 100, odom_callback, ros::VoidConstPtr(), ros::TransportHints());

    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state", 10, px4_state_callback,
                                         ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>(
        "/mavros/rc/in", 10, rc_callback, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    ros::Subscriber img_sub =
        nh.subscribe<sensor_msgs::Image>("/usb_cam/image_raw", 100, img_callback);

    target_pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 100);
    tag_pose_pub    = nh.advertise<geometry_msgs::PoseStamped>("/tag_pose", 100);
    arming_client   = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    int  tag_size = 15;
    bool ret =
        LoadBoard("/home/chrisliu/FEIBI_ws/src/tag_px4ctl/board.txt", tag_size, tag_coord);
        // LoadBoard("/home/nros/FEIBI_ws/src/tag_px4ctl/board.txt", tag_size, tag_coord);

    intrinsic = (cv::Mat_<float>(3, 3) << 393.07800238, 0, 319.65949468, 0, 393.22628119,
                 240.06435046, 0, 0, 1);
    distort   = (cv::Mat_<float>(1, 4) << 0.012564748398953971, -0.01475250908709762,
               0.0030947317078050427, 0.0006145522119168964);

    // TODO: 读取外参
    T_B_C.matrix() << -0.01860761, 0.99975974, 0.01158533, -0.10637797, 0.99939245, 0.01825674,
        0.02968864, 0.0111378, 0.02947, 0.01213072, -0.99949205, -0.14963229, 0., 0., 0., 1.;

    fb_state = VIO;
    mode     = INIT;
    while (ros::ok())
    {
        if (mode != INIT)
        {
            T_W_DES = T_W_B0 * T_B0_DES;
            if (mode == POSITION)
            {
                if (fb_state == TAG)
                {
                    // 解释版：T_W_DES = T_W_TAG * T_W_TAGk.inverse() * T_W_DESk * T_W_DESk.inverse() * T_W_B0 * T_B0_DES;

                    Eigen::Vector3f p_delta = T_W_TAG.translation() - T_W_TAGk.translation();
                    T_W_DES.translation().x() += p_delta.x();
                    T_W_DES.translation().y() += p_delta.y();
                }
            }
            else if (mode == TAG_SERVO)
            {
                if (fb_state == TAG)
                {
                    T_W_DES.translation().x() = T_W_TAG.translation().x();
                    T_W_DES.translation().y() = T_W_TAG.translation().y();
                }
            }

            geometry_msgs::PoseStamped pose;
            pose.header.stamp    = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose.position.x = T_W_DES.translation().x();
            pose.pose.position.y = T_W_DES.translation().y();
            pose.pose.position.z = T_W_DES.translation().z();
            Eigen::Quaternionf q(T_W_DES.matrix().block<3, 3>(0, 0));
            pose.pose.orientation.w = q.w();
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            target_pose_pub.publish(pose);

            T_W_DES_prev = T_W_DES;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}