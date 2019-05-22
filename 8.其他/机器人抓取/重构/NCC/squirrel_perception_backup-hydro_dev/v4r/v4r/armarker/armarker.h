/*
Copyright (C) 2010 Markus Bader

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

/**
 * @file armarker.h
 * @author Markus Bader
 * @brief  Class to abstract the artookit marker detection. \n
 * The detection is now easy to use with the opencv
 */

#ifndef ARMARKER_H
#define ARMARKER_H
#include <vector>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/concept_check.hpp>

namespace V4R {

/**
* @class MarkerBase
* @author Markus Bader
* @brief More or less the same as the ARMarkerInfo struct\n
* pos is realy the center computed by the vertex corners
**/
struct Marker {
    int     id;/// marker identitied number
    int     area; /// area number of pixels in the labeled region
    double  cf; /// confidence value (probability to be a marker)
    double  pos[2]; /// center of marker computed by the vertex corners
    double  line[6][3]; /// line equations for four side of the marker and two diagonal lines
    double  vertex[4][2]; /// edge points of the marker on the source image
    double  poseT[3]; /// pose translation of the marker in 3D [x, y, z]
    double  poseR[3]; /// pose rotation of the marker in 3D [wx, wy, wz]
    template <typename T>
    void getCorners(std::vector<cv::Point_<T> > &corners, bool clearFirst = false){
      if(clearFirst) corners.clear();
      corners.push_back(cv::Point_<T>(vertex[0][0], vertex[0][1]));
      corners.push_back(cv::Point_<T>(vertex[1][0], vertex[1][1]));
      corners.push_back(cv::Point_<T>(vertex[2][0], vertex[2][1]));
      corners.push_back(cv::Point_<T>(vertex[3][0], vertex[3][1]));
    }
};


/**
* @class MarkerPattern
* @author Markus Bader
* @brief class to discribe patterns\n
**/
class MarkerPattern {
public:
    ///constuctor
    MarkerPattern() {};
    ///destuctor
    ~MarkerPattern() {};
    /// copy constuctor
    MarkerPattern(const MarkerPattern &r) : mName(r.mName) , mPatternFile(r.mPatternFile), mSize(r.mSize), mPose(r.mPose), mGroup(r.mGroup){};
    /// constuctor
    MarkerPattern(const std::string &name, const std::string &patternFile, const cv::Size_<double> &size, const cv::Vec<double, 6> &pose, std::string group = "")
            : mName(name) , mPatternFile(patternFile), mSize(size), mPose(pose), mGroup(group) {};
    /**
     * Loads patterns
     * @param patternFile in yml format like
     * @code
     * %YAML:1.0
     * marker: [ "Aer", "Ber"]
     * hiro: !!marker
     *    file: "/tmp/hiro.patt"
     *    size: [ 0.1., 0.1]
     *    pose: [ 1., 0., 0., 0., 0., 0. ]
     *    group: "my_group" <!-- optional -->
     * Aer: !!marker
     *    file: "/tmp/1.patt"
     *    size: [ 0.1., 0.1]
     *    pose: [ 0., 0., 0., 0., 0., 0. ]
     *    group: "my_group" <!-- optional -->
     * Ber: !!marker
     *    file: "/tmp/2.patt"
     *    size: [ 0.1, 0.1]
     *    pose: [ 1., 0., 1., 0., 0., 0. ]
     *\endcode
     * @return vector with pattern information
     **/
    static const std::vector<MarkerPattern> loadList(const std::string &patternList) ;
    /// returns the pattern name
    std::string name() {
        return mName;
    };
    /// returns related pattern file name
    std::string patternFile() {
        return mPatternFile;
    };
    /// returns pattern size in meter
    cv::Size_<double> size() {
        return mSize;
    };
    /// returns the group id
    std::string group() {
        return mGroup;
    };
    /// returns pattern pose [x, y, z, wx, wy, wz]
    cv::Vec<double, 6> pose() {
        return mPose;
    };
    int getCornersAbs(std::vector<cv::Point3d> &points, bool clearFirst = false);
    void getCornersRel(std::vector<cv::Point3d> &points, bool clearFirst = false);
private:
    std::string mName; /// name of the pattern
    std::string mPatternFile;  /// pattern file
    cv::Size_<double> mSize;  /// size of the pattern in meters
    cv::Vec<double, 6> mPose;  /// pose of the pattern
    std::string mGroup; /// if empty it belongs to no group
};



/**
* @class MarkerHdl
* @author Markus Bader
* @brief Marker Handler\n
**/
class MarkerHdl {
public:
    /**
     * Constructor
     **/
    MarkerHdl();
    /**
     * Constructor
     **/
    MarkerHdl(Marker *pMarker);

    /**
     * set
     * @param pMarker
     **/
    void set(Marker *pMarker);
    
    /**
     * set
     * @param pMarker
     **/
    void set(const std::vector< MarkerPattern > &patterns);
    /**
     * set
     * @param pImg to draw
     **/
    void set(IplImage *pImg);
    /**
     * set
     * @param img to draw
     **/
    void set(cv::Mat &img);
    /**
     * get
     * @return Marker
     **/
    Marker *get();
    /**
     * marker name based on the pattern info
     * @return name
     * @pre set(const std::vector< MarkerPattern > &patterns)
     **/
    std::string name();
    /**
     * marker group based on the pattern info
     * @return group name
     * @pre set(const std::vector< MarkerPattern > &patterns)
     **/
    std::string group();
    /**
     * refence to Marker
     * @return Marker
     **/
    const Marker& operator()();
    /**
     * Copies a ARMarkerInfo struct to the current class
     * @param pARMarkerInfo pointer to an ARMarkerInfo struct
     **/
    void setARMarkerInfo(void *pARMarkerInfo);
    /**
     * Sets the undistor parameters
     * @param cameraMatrix 3x3
     * @param distCoeffs 4x1 or 5x1
     * @post undistort
     **/
    void setUndistortParameter(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs);
    /**
     * Compute pose \n
     * fills the posT and posR values
     * @param markerSize in [mm]
     * @post setUndistortParameter
     **/
    void findPose(double markerSize, const cv::Mat_<double> &M);


    /**
     * undistores marker
     * @pre setUndistortParameter
     **/
    void undistor();
    /// A other draw function
    void drawCenter(const CvScalar &color, int size = 3);
    /// A other draw function
    void drawLines(const CvScalar &color, bool bSegmentsOnly = true);
    /// A other draw function
    void drawVertex(const CvScalar &color, int size = 3);
    /// A other draw function
    void drawId(const CvScalar &color, double scale = 1., int fontFace = cv::FONT_HERSHEY_PLAIN);
    /// A other draw function
    void drawGroup(const CvScalar &color, double scale = 1., int fontFace = cv::FONT_HERSHEY_PLAIN);
    /// A other draw function
    void drawVertexId (const CvScalar &color, double scale = 0.5, int fontFace = cv::FONT_HERSHEY_PLAIN);
    /// A other draw function
    void drawPosT(const CvScalar &color, double scale = 0.5, int fontFace = cv::FONT_HERSHEY_PLAIN);
    /// A other draw function
    void drawName(const CvScalar &color, double scale = 1., int fontFace = cv::FONT_HERSHEY_PLAIN);
    /// A other draw function
    void drawConficence (const CvScalar &color, double scale = 0.5, int fontFace = cv::FONT_HERSHEY_PLAIN);
    /**
     * undistors a vector of marker
     * @param rMarker
     * @param cameraMatrix
     * @param distCoeffs
     **/
    static void undistor(std::vector<Marker> &rMarker, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs);
private:
    Marker *mpMarker;
    cv::Mat mImg;
    cv::Mat_<double> mCameraMatrix;
    cv::Mat_<double> mDistCoeffs;
    std::vector< MarkerPattern > mPatterns;
};
/**
 * @class ARMarkerDetection
 * @author Markus Bader
 **/
class MarkerDetection {
public:

    /**
     * Constructor
     * @param imgWidht
     * @param imgHeight
     **/
    MarkerDetection();
    /**
     * Destructor
     **/
    ~MarkerDetection();
    /**
     * Detects AR marker in the image provided
     * @param pImg
     * @param thresh
     * @return vector with detected marker
     * @pre loadMarker and loadMarkerList
     **/
    const std::vector<V4R::Marker>  &detectMarker(const IplImage *pImg, int thresh = 100);

    /**
     * Detects AR marker in the image provided
     * @param img
     * @param thresh
     * @return vector with detected marker
     * @pre loadMarker and loadMarkerList
     **/
    const std::vector<V4R::Marker>  &detectMarker(const cv::Mat &img, int thresh = 100);
    
    /**
     * optimization using the opencv cvFindCornerSubPix <\br>
     * but it makes only sence on not calibrated images
     * @param img 
     * @param winSize
     * @return vector with detected optimized marker
     * @ToDo finish the implementation
     * @pre detectMarker
     **/
    const std::vector<V4R::Marker>  &subPixOptimization(const cv::Mat &img, const cv::Size winSize = cv::Size(11,11));
    /**
     * loads a list of marker
     * @param rFile
     * @code
     * %YAML:1.0
     * marker: [ "Aer", "Ber"]
     * hiro: !!marker
     *    file: "/tmp/hiro.patt"
     *    size: [ 0.1., 0.1]
     *    pose: [ 1., 0., 0., 0., 0., 0. ]
     *    group: "my_group" <!-- optional -->
     * Aer: !!marker
     *    file: "/tmp/1.patt"
     *    size: [ 0.1., 0.1]
     *    pose: [ 0., 0., 0., 0., 0., 0. ]
     *    group: "my_group" <!-- optional -->
     * Ber: !!marker
     *    file: "/tmp/2.patt"
     *    size: [ 0.1, 0.1]
     *    pose: [ 1., 0., 1., 0., 0., 0. ]
     *\endcode
     * @see ARMarkerPattern::loadList
     * @post ARMarkerDetection::detectMarker
     **/
    int loadMarkerList(const std::string &rFile);
    /**
     * loads a marker file for detection
     * @param rFile
     * @param size of the marker / one side length [mm]
     * @return id of the marker
     * @post ARMarkerDetection::detectMarker
     **/
    int loadMarker(const std::string &rFile, double size = 1);

    /**
    * @brief computes the camera pose based on a selected group of marker
    * @param cameraMatrix 3x3
    * @param distCoeffs 4x1 
    * @param rvec computed rotation 
    * @param tvec computed translation
    * @param group_conficence confidence of the detected group
    * @param group group of markers used on an empty string it will use all markers
    * @param min_convidence minimal confidence which a marker must have to be part of a group
    * @return zero on success
    **/
    int computePose(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, cv::Mat_<double> &rvec, cv::Mat_<double> &tvec, double &group_conficence, std::string group = "", double min_convidence = 0.5);
    
    /**
    * @brief computes the marker poses
    * @param cameraMatrix 3x3
    * @param distCoeffs 4x1 
    * @return the detected marker
    **/
    const std::vector<V4R::Marker>  &computeMarkerLocation (const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs );

    /**
     * pattern info
     **/
    const std::vector<MarkerPattern> &getPatterns() const;
    
protected:

    void *mpARCParam;
    unsigned int mFrameCount;
    std::vector<Marker> mMarker;
    std::vector<MarkerPattern> mMarkerPatterns;
    void init(int imgWidht, int imgHeight);
};
};
#endif // ARMARKER_H
// kate: indent-mode cstyle; space-indent on; indent-width 0;
