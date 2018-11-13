#ifndef __COMMON_H_ 
#define __COMMON_H_
#include <vector>
#include <opencv2/core/core.hpp>
class common
{
public:
    static std::vector<cv::Point2i > CocoPairsRender;
    static std::vector<cv::Scalar > CocoColors;
};
std::vector<cv::Point2i > common::CocoPairsRender={
            cv::Point2i(1, 2), cv::Point2i(1, 5), cv::Point2i(2, 3), cv::Point2i(3, 4), cv::Point2i(5, 6), cv::Point2i(6, 7), cv::Point2i(1, 8), cv::Point2i(8, 9), cv::Point2i(9, 10), cv::Point2i(1, 11),
    cv::Point2i(11, 12), cv::Point2i(12, 13), cv::Point2i(1, 0), cv::Point2i(0, 14), cv::Point2i(14, 16), cv::Point2i(0, 15), cv::Point2i(15, 17), cv::Point2i(2, 16), cv::Point2i(5, 17)
    };
std::vector<cv::Scalar > common::CocoColors={cv::Scalar(255, 0, 0), cv::Scalar(255, 85, 0), cv::Scalar(255, 170, 0), cv::Scalar(255, 255, 0), cv::Scalar(170, 255, 0), cv::Scalar(85, 255, 0), cv::Scalar(0, 255, 0),
              cv::Scalar(0, 255, 85), cv::Scalar(0, 255, 170), cv::Scalar(0, 255, 255), cv::Scalar(0, 170, 255), cv::Scalar(0, 85, 255), cv::Scalar(0, 0, 255), cv::Scalar(85, 0, 255),
              cv::Scalar(170, 0, 255), cv::Scalar(255, 0, 255), cv::Scalar(255, 0, 170), cv::Scalar(255, 0, 85)};

    
#endif
