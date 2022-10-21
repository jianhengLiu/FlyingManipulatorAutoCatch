#include <Eigen/Dense>
#include <Eigen/Core>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <iostream>

#include "TagDetector.hpp"

int main(int argc, char *argv[])
{
    std::string file  = "../test.jpg";
    cv::Mat     image = cv::imread(file);

    int                                   tag_size = 15;
    std::vector<std::vector<cv::Point3f>> tag_coord;
    bool                                  ret = LoadBoard("../board.txt", tag_size, tag_coord);

    cv::Mat         inrinsic = (cv::Mat_<float>(3, 3) << 1200, 0, 1920, 0, 1200, 1440, 0, 0, 1);
    cv::Mat         distort  = (cv::Mat_<float>(1, 4) << 0, 0, 0, 0);
    Eigen::Affine3f pose;

    int tag_len = TagDetector(image, tag_coord, inrinsic, distort, pose);

    std::cout << tag_len << std::endl;
    std::cout << pose.matrix() << std::endl;

    return 0;
}
