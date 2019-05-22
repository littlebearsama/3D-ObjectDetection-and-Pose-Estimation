/**
 * @file print_cv.h
 * @author Markus Bader
 * @date Mo Jul 9 2007
 * @version 0.1
 * @brief
 *
 * @see
 **/

#ifndef PRINT_CV_H
#define PRINT_CV_H

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <opencv/cv.h>

#define CVPRINT( A ) cvPrint( A, #A"");
#define CVPRINT_REF( A ) cvPrint( &A, #A"");
#define TYPE_CORRECT_ENTRY(M, r, c) \
CV_MAT_TYPE(M.type()) == CV_64F ? M.at<double>(r, c) : \
CV_MAT_TYPE(M.type()) == CV_8U ? M.at<uchar>(r, c) : \
CV_MAT_TYPE(M.type()) == CV_8S ? M.at<char>(r, c) : \
CV_MAT_TYPE(M.type()) == CV_16U ? M.at<uint16_t>(r, c) : \
CV_MAT_TYPE(M.type()) == CV_16S ? M.at<int16_t>(r, c) : \
CV_MAT_TYPE(M.type()) == CV_32S ? M.at<int32_t>(r, c) : \
CV_MAT_TYPE(M.type()) == CV_32F ? M.at<float>(r, c) 

template <typename T>
inline std::ostream& operator << ( std::ostream &os, const cv::Size_<T> &r) {
    return os << std::setprecision (5) << std::fixed << "[ " << r.width << ", " << r.height << " ]";
};

template <typename T>
inline std::ostream& operator << ( std::ostream &os, const cv::Point3_<T> &r) {
    return os << std::setprecision (5) << std::fixed << "[ " << r.x << ", " << r.y << ", " << r.z << " ]";
};
template <typename T>
inline std::ostream& operator << ( std::ostream &os, const cv::Point_<T> &r) {
    return os << std::setprecision (5) << std::fixed << "[ " << r.x << ", " << r.y << " ]";
};

template <typename T, int cn>
inline std::ostream& operator << ( std::ostream &os, const cv::Vec<T, cn> &r) {
    os << "[ " << std::setprecision (5) << std::fixed << std::setw(10) << r.val[0];
    for (int i = 1; i < cn; i++) {
        os << ", " << std::setprecision (5) << std::fixed << std::setw(10) << r.val[i];
    }
    os << " ]";
    return os;
};

/**
* @brief output via stream
**/
inline std::ostream & operator<< ( std::ostream &os, const CvPoint &p ) {
    char pText[64];
    sprintf(pText, "[ %-12i, %-12i ]", p.x , p.y);
    return os << pText;
}

/**
* @brief output via stream
**/
inline std::ostream & operator<< ( std::ostream &os, const CvPoint2D64f &p ) {
    char pText[64];
    sprintf(pText, "[ %-12.4f, %-12.4f ]", p.x , p.y);
    return os << pText;
}

/**
* @brief output via stream
**/
inline std::ostream & operator<< ( std::ostream &os, const CvPoint3D64f &p ) {
    char pText[64];
    sprintf(pText, "[ %-12.4f, %-12.4f, %-12.4f ]", p.x , p.y, p.z);
    return os << pText;
}

/**
* @brief output via stream
**/
template <typename T>
inline std::ostream & operator<< ( std::ostream &os, const cv::Mat_<T> &M ) {
    os << "[";
    for ( int r = 0; r < M.rows; r++ ) {
      if(r != 0) os << " ";
      for ( int c = 0; c < M.cols; c++ ) {
         os << std::setprecision (5) << std::fixed << std::setw(10) << M(r, c);
         if(c < (M.cols -1 )) os << ", ";
      }
      if ( r < M.rows-1) os << ";\n";
    }
    os << "]";
    return os;
}

namespace V4R {
/**
* @brief prints a cv matrix in a nice format with an info text/ or in matlab format
* @param rMatrix
* @param pInfo
**/
inline void print ( const cv::Mat &rMatix, const char *pInfo = NULL) {
    int type = CV_MAT_TYPE(rMatix.type());
    if ( pInfo != NULL ) {
        printf ( "%s  [%i x %i] ***", pInfo, rMatix.rows, rMatix.cols );
        switch (type) {
        case CV_8U:
            printf ( " Type: CV_8U\n");
            break;
        case CV_8S:
            printf ( " Type: CV_8S\n");
            break;
        case CV_16U:
            printf ( " Type: CV_16U\n");
            break;
        case CV_16S:
            printf ( " Type: CV_16S\n");
            break;
        case CV_32S:
            printf ( " Type: CV_32S\n");
            break;
        case CV_32SC2:
            printf ( " Type: CV_32SC2\n");
            break;
        case CV_32SC3:
            printf ( " Type: CV_32SC3\n");
            break;
        case CV_32SC4:
            printf ( " Type: CV_32SC4\n");
            break;
        case CV_32F:
            printf ( " Type: CV_32F\n");
            break;
        case CV_32FC2:
            printf ( " Type: CV_32FC2\n");
            break;
        case CV_32FC3:
            printf ( " Type: CV_32FC3\n");
            break;
        case CV_32FC4:
            printf ( " Type: CV_32FC4\n");
            break;
        case CV_64F:
            printf ( " Type: CV_64F\n");
            break;
        case CV_64FC2:
            printf ( " Type: CV_64FC2\n");
            break;
        case CV_64FC3:
            printf ( " Type: CV_64FC3\n");
            break;
        case CV_64FC4:
            printf ( " Type: CV_64FC4\n");
            break;
        default:
            printf ( " Type: NA\n");
        }

    }
    
    printf( "[");
    for ( int row = 0; row < rMatix.rows; row++ ) {
        for ( int col = 0; col < rMatix.cols; col++ ) {
            switch (type) {
            case CV_8U:
                printf ( " %-12i",   rMatix.at<uchar>(row, col) );
                break;
            case CV_8S:
                printf ( " %-12i",   rMatix.at<char>(row, col) );
                break;
            case CV_16U:
                printf ( " %-12i",   rMatix.at<uint16_t>(row, col) );
                break;
            case CV_16S:
                printf ( " %-12i",   rMatix.at<int16_t>(row, col) );
                break;
            case CV_32S:
                printf ( " %-12i",   rMatix.at<int32_t>(row, col) );
                break;
            case CV_32SC2:
                printf ( " %-12i",   rMatix.at<int32_t>(row, col*2) );
                printf ( " %-12i",   rMatix.at<int32_t>(row, col*2+1) );
                break;
            case CV_32SC3:
                printf ( " %-12i",   rMatix.at<int32_t>(row, col*2) );
                printf ( " %-12i",   rMatix.at<int32_t>(row, col*2+1));
                printf ( " %-12i",   rMatix.at<int32_t>(row, col*2+2));
                break;
            case CV_32SC4:
                printf ( " %-12i",   rMatix.at<int32_t>(row, col*2) );
                printf ( " %-12i",   rMatix.at<int32_t>(row, col*2+1));
                printf ( " %-12i",   rMatix.at<int32_t>(row, col*2+2));
                printf ( " %-12i",   rMatix.at<int32_t>(row, col*2+3));
                break;
            case CV_32F:
                printf ( " %-12.4f",   rMatix.at<float>(row, col) );
                break;
            case CV_32FC2:
                printf ( " %-12.4f",   rMatix.at<float>(row, col*2) );
                printf ( " %-12.4f",   rMatix.at<float>(row, col*2+1) );
                break;
            case CV_32FC3:
                printf ( " %-12.4f",   rMatix.at<float>(row, col*2) );
                printf ( " %-12.4f",   rMatix.at<float>(row, col*2+1) );
                printf ( " %-12.4f",   rMatix.at<float>(row, col*2+2) );
                break;
            case CV_32FC4:
                printf ( " %-12.4f",   rMatix.at<float>(row, col*2) );
                printf ( " %-12.4f",   rMatix.at<float>(row, col*2+1) );
                printf ( " %-12.4f",   rMatix.at<float>(row, col*2+2) );
                printf ( " %-12.4f",   rMatix.at<float>(row, col*2+3) );
                break;
            case CV_64F:
                printf ( " %-12.4f",   rMatix.at<double>(row, col) );
                break;
            case CV_64FC2:
                printf ( " %-12.4f",   rMatix.at<double>(row, col*2) );
                printf ( " %-12.4f",   rMatix.at<double>(row, col*2+1) );
                break;
            case CV_64FC3:
                printf ( " %-12.4f",   rMatix.at<double>(row, col*2) );
                printf ( " %-12.4f",   rMatix.at<double>(row, col*2+1) );
                printf ( " %-12.4f",   rMatix.at<double>(row, col*2+2) );
                break;
            case CV_64FC4:
                printf ( " %-12.4f",   rMatix.at<double>(row, col*2) );
                printf ( " %-12.4f",   rMatix.at<double>(row, col*2+1) );
                printf ( " %-12.4f",   rMatix.at<double>(row, col*2+2) );
                printf ( " %-12.4f",   rMatix.at<double>(row, col*2+3) );
                break;
            default:
                printf ( " Type: NA\n");
            }
        }
        if ( row < rMatix.rows-1) printf ( ";\n");
        else  printf( "]\n");
    }
}

/**
* @brief prints a cv matrix in a nice format with an info text/ or in matlab format
* @param pr
* @param pInfo
**/
template <typename T, int cn>
inline void print ( const cv::Vec<T, cn> &r, const char *pInfo = NULL) {
    if ( pInfo != NULL ) {
        printf ( "%s = ", pInfo);
    } 
    printf ( "[%-12.4f", r[0]  );
    for(int i = 1; i < cn-1; i++) printf ( " %-12.4f", r[i]  );
    printf ( " %-12.4f]\n", r[cn-1]  );
}

/**
* @brief prints a cv point in a nice format with an info text/ or in matlab format
* @param Point2i
* @param pInfo
**/
inline void print ( const cv::Point_<int> &p, const char *pInfo = NULL) {
    if ( pInfo != NULL ) {
        printf ( "%s = ", pInfo);
    } 
    printf("[ %-12i, %-12i ]", p.x , p.y);
}
/**
* @brief prints a cv point in a nice format with an info text/ or in matlab format
* @param Point3i
* @param pInfo
**/
inline void print ( const cv::Point3_<int> &p, const char *pInfo = NULL) {
    if ( pInfo != NULL ) {
        printf ( "%s = ", pInfo);
    } 
    printf("[ %-12i, %-12i, %-12i ]", p.x , p.y, p.z);
}

/**
* @brief prints a cv point in a nice format with an info text/ or in matlab format
* @param Point2f
* @param pInfo
**/
inline void print ( const cv::Point_<float> &p, const char *pInfo = NULL) {
    if ( pInfo != NULL ) {
        printf ( "%s = ", pInfo);
    } 
    printf("[ %-12.4f, %-12.4f ]", p.x , p.y);
}
/**
* @brief prints a cv point in a nice format with an info text/ or in matlab format
* @param Point3f
* @param pInfo
**/
inline void print ( const cv::Point3_<float> &p, const char *pInfo = NULL) {
    if ( pInfo != NULL ) {
        printf ( "%s = ", pInfo);
    } 
    printf("[ %-12.4f, %-12.4f, %-12.4f ]", p.x , p.y, p.z);
}
/**
* @brief prints a cv point in a nice format with an info text/ or in matlab format
* @param Point2d
* @param pInfo
**/
inline void print ( const cv::Point_<double> &p, const char *pInfo = NULL) {
    if ( pInfo != NULL ) {
        printf ( "%s = ", pInfo);
    } 
    printf("[ %-12.4f, %-12.4f ]\n", p.x , p.y);
}

/**
* @brief prints a cv point in a nice format with an info text/ or in matlab format
* @param Point3d
* @param pInfo
**/
inline void print ( const cv::Point3_<double> &p, const char *pInfo = NULL) {
    if ( pInfo != NULL ) {
        printf ( "%s = ", pInfo);
    } 
    printf("[ %-12.4f, %-12.4f, %-12.4f  ]\n", p.x , p.y, p.z);
}

/**
* @brief prints a cv point in a nice format with an info text/ or in matlab format
* @param rPoint
* @param pInfo
**/
inline void print ( const CvPoint3D64f &rPoint, const char *pInfo = NULL) {
    if ( pInfo != NULL ) {
        printf ( "%s = ", pInfo);
    } 
    printf ( "[%-12.4f", rPoint.x  );
    printf ( " %-12.4f", rPoint.y  );
    printf ( " %-12.4f]\n", rPoint.z  );
}

/**
* @brief prints a cv point in a nice format with an info text/ or in matlab format
* @param rPoint
* @param pInfo
**/
inline void print ( const CvPoint2D64f &rPoint, const char *pInfo = NULL ) {
    if ( pInfo != NULL ) printf ( "%s = ", pInfo);
    printf ( "[%-12.4f", rPoint.x  );
    printf ( " %-12.4f]", rPoint.y  );
    printf ( "\n" );
}

}

#endif //PRINT_CV_H
