/*////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2013, Jonathan Balzer
//
// All rights reserved.
//
// This file is part of the R4R library.
//
// The R4R library is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// The R4R library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with the R4R library. If not, see <http://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////*/

#ifndef _V4R_CALIBRATION_DSENSOR_H
#define _V4R_CALIBRATION_DSENSOR_H

#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include "dccam.h"
#include <map>

class CDepthColorSensor {

public:

    //! Show which PrimeSense modules are available.
    static std::map<int,std::string> ShowAvailableSensors();

    //! Constructor.
    CDepthColorSensor();

    //! Destructor.
    virtual ~CDepthColorSensor() { CloseDevice(); openni::OpenNI::shutdown(); }

    //! Parametrized constructor.
    CDepthColorSensor(const CCam& rgb, const CDepthCam& depth);
    CDepthColorSensor(const CDepthColorCam& cam);

    //! Open a PrimeSense device by number.
    bool OpenDevice(int i, int stream=openni::SENSOR_COLOR);

    //! Closes the active device and all streams associated with it.
    bool CloseDevice();

    //! Get RGB image.
    cv::Mat GetRGB();

    //! Get IR image.
    cv::Mat GetIR();

    //! Get a depth image.
    cv::Mat GetDisparity();

    //! Check whether everything is still running smoothly.
    bool IsSane();


    //! Start dumping streams into a ONI file.
    bool StartRecording(const char* filename);

    //! Stop file dump.
    bool StopRecording();


    //! Access to camera.
    CDepthColorCam& GetCam() { return m_cam; }



private:

    CDepthColorCam m_cam;

    openni::Device m_device;                                   //! OpenNI device
    openni::VideoStream m_rgb_stream;                          //! OpenNI image stream
    openni::VideoStream m_depth_stream;                        //! OpenNI depth stream
    openni::VideoStream m_ir_stream;                        //! OpenNI depth stream
    openni::Recorder m_recorder;                               //! a recorder


    // protect from copying
    CDepthColorSensor(const CDepthColorSensor& sensor);
    CDepthColorSensor operator=(const CDepthColorSensor& sensor);

};


#endif // DSENSOR_H
