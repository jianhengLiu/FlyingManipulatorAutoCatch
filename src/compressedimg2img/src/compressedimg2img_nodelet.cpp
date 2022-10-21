/*
 * @Author: Jianheng Liu
 * @Date: 2021-12-09 21:40:57
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-02-19 19:19:34
 * @Description: Description
 */

#include "sensor_msgs/image_encodings.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <nodelet/nodelet.h> // 基类Nodelet所在的头文件
#include <pluginlib/class_list_macros.h>

using namespace std;

// Compression formats
enum compressionFormat { UNDEFINED = -1, INV_DEPTH };

// Compression configuration
struct ConfigHeader {
  // compression format
  compressionFormat format;
  // quantization parameters (used in depth image compression)
  float depthParam[2];
};

namespace compressedimg2img_nodelet_ns {
class Compressedimg2ImgNodelet : public nodelet::Nodelet {
public:
  Compressedimg2ImgNodelet() {}

private:
  virtual void onInit() {
    ros::NodeHandle &nh = getMTNodeHandle();

    sub_img = nh.subscribe("/camera/color/image_raw/compressed", 10,
                           &Compressedimg2ImgNodelet::img_callback, this);
    sub_depth =
        nh.subscribe("/camera/aligned_depth_to_color/image_raw/compressedDepth",
                     10, &Compressedimg2ImgNodelet::depth_callback, this);
    pub_img = nh.advertise<sensor_msgs::Image>("/decompressed_img", 10);
    pub_depth = nh.advertise<sensor_msgs::Image>("/decompressed_depth_img", 10);
  }

  ros::Subscriber sub_img, sub_depth;
  ros::Publisher pub_img, pub_depth;

  void img_callback(const sensor_msgs::CompressedImageConstPtr &img_msg) {
    // double start_t = ros::Time::now().toSec();
    pub_img.publish(
        cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)
            ->toImageMsg());
    // cout << "Decompressed Img Cost: "
    //      << to_string(ros::Time::now().toSec() - start_t) << endl;
  }

  // Get compressed image data
  // https://answers.ros.org/question/51490/sensor_msgscompressedimage-decompression/
  // https://github.com/heleidsn/heleidsn.github.io/blob/c02ed13cb4ffe85ee8f03a9ad93fa55336f84f7c/source/_posts/realsense-depth-image.md
  // https://sourcegraph.com/github.com/ros-perception/image_transport_plugins/-/blob/compressed_depth_image_transport/include/compressed_depth_image_transport/codec.h
  void depth_callback(const sensor_msgs::CompressedImageConstPtr &depth_msg) {
    // double start_t = ros::Time::now().toSec();
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    // Copy message header
    cv_ptr->header = depth_msg->header;

    // Assign image encoding
    const size_t split_pos = depth_msg->format.find(';');
    const std::string image_encoding = depth_msg->format.substr(0, split_pos);
    cv_ptr->encoding = image_encoding;
    if (depth_msg->data.size() > sizeof(ConfigHeader)) {
      // Read compression type from stream
      ConfigHeader compressionConfig;
      memcpy(&compressionConfig, &depth_msg->data[0],
             sizeof(compressionConfig));

      const std::vector<uint8_t> imageData(depth_msg->data.begin() +
                                               sizeof(ConfigHeader),
                                           depth_msg->data.end());

      if (image_encoding == "mono16" || image_encoding == "16UC1") {
        cv_ptr->image = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
        pub_depth.publish(cv_ptr->toImageMsg());
      } else if (image_encoding == "32FC1") {
        cv::Mat decompressed = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);

        if ((decompressed.rows > 0) && (decompressed.cols > 0)) {
          cv_ptr->image =
              cv::Mat(decompressed.rows, decompressed.cols, CV_32FC1);
          // Depth conversion
          cv::MatIterator_<float> itDepthImg = cv_ptr->image.begin<float>(),
                                  itDepthImg_end = cv_ptr->image.end<float>();
          cv::MatConstIterator_<unsigned short>
              itInvDepthImg = decompressed.begin<unsigned short>(),
              itInvDepthImg_end = decompressed.end<unsigned short>();

          for (; (itDepthImg != itDepthImg_end) &&
                 (itInvDepthImg != itInvDepthImg_end);
               ++itDepthImg, ++itInvDepthImg) {
            // check for NaN & max depth
            if (*itInvDepthImg) {
              *itDepthImg =
                  compressionConfig.depthParam[0] /
                  ((float)*itInvDepthImg - compressionConfig.depthParam[1]);
              // *itDepthImg = depthQuantA / ((float)*itInvDepthImg -
              // depthQuantB);
            } else {
              *itDepthImg = std::numeric_limits<float>::quiet_NaN();
            }
          }
          pub_depth.publish(cv_ptr->toImageMsg());
        }
      }
      // cout << "Decompressed DepthImg Cost: "
      //      << to_string(ros::Time::now().toSec() - start_t) << endl;
    }
  }
};
PLUGINLIB_EXPORT_CLASS(compressedimg2img_nodelet_ns::Compressedimg2ImgNodelet,
                       nodelet::Nodelet)

} // namespace compressedimg2img_nodelet_ns
