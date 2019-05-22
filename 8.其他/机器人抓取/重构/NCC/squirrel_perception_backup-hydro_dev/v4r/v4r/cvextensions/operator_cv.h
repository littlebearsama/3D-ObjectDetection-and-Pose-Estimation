/**
 * @file operator_cv.h
 * @author Markus Bader
 * @version 0.1
 * @brief
 *
 * @see
 **/

#ifndef OPERATOR_CV_H
#define OPERATOR_CV_H

#include <opencv/cv.h>
namespace cv
{

template<typename _T>
static inline const Point_<_T> operator * (const Mat_<_T> &M, const Point_<_T> &src)
{
    Point_<_T> des;
    des.x = M(0,0) * src.x + M(0,1) * src.y;
    des.y = M(1,0) * src.x + M(1,1) * src.y;
    if (M.cols > 2) {
        des.x += M(0,2);
        des.y += M(1,2);
    }
    return des;
}

template<typename _T>
static inline const Point3_<_T> operator * (const Mat_<_T> &M, const Point3_<_T> &src)
{
    Point3_<_T> des;
    des.x = M(0,0) * src.x + M(0,1) * src.y + M(0,2) * src.z;
    des.y = M(1,0) * src.x + M(1,1) * src.y + M(1,2) * src.z;
    des.z = M(2,0) * src.x + M(2,1) * src.y + M(2,2) * src.z;
    if (M.cols > 3) {
        des.x += M(0,3);
        des.y += M(1,3);
        des.z += M(2,3);
    }
    return des;
}
template <typename T2>
void copy(const cv::Mat &M, cv::Vec<T2,3> &vec) {
    switch (vec.type) {
    case CV_8U:
        vec[0] = M.at<uchar>(0, 0);
        vec[1] = M.at<uchar>(1, 0);
        vec[2] = M.at<uchar>(2, 0);
        break;
    case CV_8S:
        vec[0] = M.at<char>(0, 0);
        vec[1] = M.at<char>(1, 0);
        vec[2] = M.at<char>(2, 0);
        break;
    case CV_16U:
        vec[0] = M.at<uint16_t>(0, 0);
        vec[1] = M.at<uint16_t>(1, 0);
        vec[2] = M.at<uint16_t>(2, 0);
        break;
    case CV_16S:
        vec[0] = M.at<int16_t>(0, 0);
        vec[1] = M.at<int16_t>(1, 0);
        vec[2] = M.at<int16_t>(2, 0);
        break;
    case CV_32S:
        vec[0] = M.at<int32_t>(0, 0);
        vec[1] = M.at<int32_t>(1, 0);
        vec[2] = M.at<int32_t>(2, 0);
        break;
    case CV_32SC2:
        std::cerr << "Wrong format to set vec<T,3> CV_32SC2." << std::endl;
        break;
    case CV_32SC3:
        vec[0] = M.at<int32_t>(0, 0);
        vec[1] = M.at<int32_t>(1, 0);
        vec[2] = M.at<int32_t>(2, 0);
        break;
    case CV_32SC4:
        vec[0] = M.at<int32_t>(0, 0);
        vec[1] = M.at<int32_t>(1, 0);
        vec[2] = M.at<int32_t>(2, 0);
        break;
    case CV_32F:
        vec[0] = M.at<float>(0, 0);
        vec[1] = M.at<float>(1, 0);
        vec[2] = M.at<float>(2, 0);
        break;
    case CV_32FC2:
        std::cerr << "Wrong format to set vec<T,3> CV_32FC2." << std::endl;
        break;
    case CV_32FC3:
        vec[0] = M.at<float>(0, 0);
        vec[1] = M.at<float>(1, 0);
        vec[2] = M.at<float>(2, 0);
        break;
    case CV_32FC4:
        vec[0] = M.at<float>(0, 0);
        vec[1] = M.at<float>(1, 0);
        vec[2] = M.at<float>(2, 0);
        break;
    case CV_64F:
        vec[0] = M.at<double>(0, 0);
        vec[1] = M.at<double>(1, 0);
        vec[2] = M.at<double>(2, 0);
        break;
    case CV_64FC2:
        std::cerr << "Wrong format to set vec<T,3> CV_64FC2." << std::endl;
        break;
    case CV_64FC3:
        vec[0] = M.at<double>(0, 0);
        vec[1] = M.at<double>(1, 0);
        vec[2] = M.at<double>(2, 0);
        break;
    case CV_64FC4:
        vec[0] = M.at<double>(0, 0);
        vec[1] = M.at<double>(1, 0);
        vec[2] = M.at<double>(2, 0);
        break;
    default:
        std::cerr << "Unkonw format to set vec<T,3>" << std::endl;
    }
}
}

#endif //OPERATOR_CV_H
