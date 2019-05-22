#include "dsensor.h"

using namespace openni;
using namespace std;
using namespace cv;

map<int,string> CDepthColorSensor::ShowAvailableSensors() {

  map<int,string> result;

  OpenNI::initialize();

  Array<DeviceInfo> devicelist;
  OpenNI::enumerateDevices(&devicelist);

  for(int i=0; i<devicelist.getSize(); i++) {

    result.insert(pair<int,string>(i,devicelist[i].getName()));

    cout << "Device no: " << i << endl;
    cout << "Name: " << devicelist[i].getName() << endl;
    cout << "URI: " << devicelist[i].getUri() << endl;
    cout << "Vendor: " << devicelist[i].getVendor() << endl;
    cout << "USB Product ID: " << devicelist[i].getUsbProductId() << endl;
    cout << "USB Vendor ID: " << devicelist[i].getUsbVendorId() << endl;

    Device dev;
    dev.open(devicelist[i].getUri());

    const Array<VideoMode>& modes = dev.getSensorInfo(SENSOR_COLOR)->getSupportedVideoModes();

    for(size_t j=0; j<modes.getSize(); j++) {

      cout << "Pixel format:";

      switch(modes[j].getPixelFormat()) {

      case PIXEL_FORMAT_RGB888:
        cout << "RGB888" << endl;
        break;
      case PIXEL_FORMAT_YUV422:
        cout << "YUV422" << endl;
        break;
      case PIXEL_FORMAT_GRAY8:
        cout << "GRAY8" << endl;
        break;
      case PIXEL_FORMAT_GRAY16:
        cout << "GRAY16" << endl;
        break;
      case PIXEL_FORMAT_JPEG:
        cout << "JPEG" << endl;
        break;
      default:
        break;

      }

      cout << "FPS: " << modes[j].getFps() << endl;
      cout << "Resolution: " << modes[j].getResolutionX() << "x" << modes[j].getResolutionY() << endl;
      cout << "---" << endl;

    }

    dev.close();


  }

  OpenNI::shutdown();

  return result;

}

CDepthColorSensor::CDepthColorSensor():
  m_device(),
  m_cam(),
  m_rgb_stream(),
  m_depth_stream(),
  m_recorder()
{

  OpenNI::initialize();


}

CDepthColorSensor::CDepthColorSensor(const CCam& rgb, const CDepthCam& depth):
  m_device(),
  m_cam(rgb, depth),
  m_rgb_stream(),
  m_depth_stream(),
  m_recorder()
{

  OpenNI::initialize();

  // attach streams to recorder (maybe do this later)
  m_recorder.attach(m_rgb_stream);
  m_recorder.attach(m_depth_stream);

}

CDepthColorSensor::CDepthColorSensor(const CDepthColorCam& cam):
  m_device(),
  m_cam(cam),
  m_rgb_stream(),
  m_depth_stream(),
  m_recorder()
{

  OpenNI::initialize();

  // attach streams to recorder (maybe do this later)
  m_recorder.attach(m_rgb_stream);
  m_recorder.attach(m_depth_stream);

}

bool CDepthColorSensor::OpenDevice(int i, int stream) {

  Array<DeviceInfo> devicelist;
  OpenNI::enumerateDevices(&devicelist);

  if(i>=devicelist.getSize()) {
    cout << "Device no. " << i << " not found." << endl;
    return 1;

  }

  // open device
  if(m_device.open(devicelist[i].getUri())!=STATUS_OK) {

    cout << "Couldn't open device: " << OpenNI::getExtendedError() << endl;
    return 1;

  }

  // set registration property, we will do this manually
  m_device.setImageRegistrationMode(IMAGE_REGISTRATION_OFF);

  // open streams
  if(stream==SENSOR_COLOR){
    if (m_device.getSensorInfo(SENSOR_COLOR) != NULL) {

      if(m_rgb_stream.create(m_device, SENSOR_COLOR)!=STATUS_OK)
        return 1;

    }
    else
      return 1;

    // rgb settings
    VideoMode mode(m_rgb_stream.getVideoMode());
    mode.setResolution(m_cam.m_rgb_cam.m_size[0],m_cam.m_rgb_cam.m_size[1]);
    m_rgb_stream.setVideoMode(mode);
    m_rgb_stream.setMirroringEnabled(false);

    CameraSettings* cam = m_rgb_stream.getCameraSettings();
    cam->setAutoExposureEnabled(false);
    cam->setAutoWhiteBalanceEnabled(false);

    if(m_rgb_stream.start()!= STATUS_OK) {

      cout << "Couldn't start RGB stream: " << OpenNI::getExtendedError() << endl;
      return 1;

    }

  }else{
    if (m_device.getSensorInfo(SENSOR_IR) != NULL) {

      if(m_ir_stream.create(m_device, SENSOR_IR)!=STATUS_OK)
        return 1;

    }
    else
      return 1;

    // rgb settings
    VideoMode mode(m_ir_stream.getVideoMode());
    mode.setResolution(m_cam.m_rgb_cam.m_size[0],m_cam.m_rgb_cam.m_size[1]);
    m_ir_stream.setVideoMode(mode);
    m_ir_stream.setMirroringEnabled(false);

    if(m_ir_stream.start()!= STATUS_OK) {

      cout << "Couldn't start IR stream: " << OpenNI::getExtendedError() << endl;
      return 1;

    }
  }

  if (m_device.getSensorInfo(SENSOR_DEPTH) != NULL) {

    if(m_depth_stream.create(m_device, SENSOR_DEPTH)!=STATUS_OK)
      return 1;

  }
  else
    return 1;


  // depth settings
  VideoMode dmode(m_depth_stream.getVideoMode());
  dmode.setResolution(m_cam.m_depth_cam.m_size[0],m_cam.m_depth_cam.m_size[1]);
  dmode.setPixelFormat(PIXEL_FORMAT_SHIFT_9_2);
  m_depth_stream.setVideoMode(dmode);
  m_depth_stream.setMirroringEnabled(false);

  if(m_depth_stream.start()!= STATUS_OK) {

    cout << "Couldn't start depth stream: " << OpenNI::getExtendedError() << endl;
    return 1;

  }



  return 0;

}

bool CDepthColorSensor::CloseDevice() {

  if(m_depth_stream.isValid()) {

    m_depth_stream.stop();
    m_depth_stream.destroy();

  }

  if(m_rgb_stream.isValid()) {

    m_rgb_stream.stop();
    m_rgb_stream.destroy();

  }

  if(m_ir_stream.isValid()) {

    m_ir_stream.stop();
    m_ir_stream.destroy();

  }

  if(m_device.isValid())
    m_device.close();

  return 0;

}

Mat CDepthColorSensor::GetRGB() {

  VideoFrameRef frame;

  if(m_rgb_stream.readFrame(&frame)!=STATUS_OK)
    return Mat::zeros(m_cam.m_rgb_cam.m_size[1],m_cam.m_rgb_cam.m_size[0],CV_8UC3);

  return Mat(frame.getHeight(),frame.getWidth(),CV_8UC3,(unsigned char*)frame.getData());

}

Mat CDepthColorSensor::GetIR() {

  VideoFrameRef frame;

  if(m_ir_stream.readFrame(&frame)!=STATUS_OK)
    return Mat::zeros(m_cam.m_rgb_cam.m_size[1],m_cam.m_rgb_cam.m_size[0],CV_8UC1);

  return Mat(frame.getHeight(),frame.getWidth(),CV_8UC1,(unsigned char*)frame.getData());

}

Mat CDepthColorSensor::GetDisparity() {

  VideoFrameRef frame;

  if(m_depth_stream.readFrame(&frame)!=STATUS_OK)
    return Mat::zeros(m_cam.m_depth_cam.m_size[1],m_cam.m_depth_cam.m_size[0],CV_16UC1);

  return Mat(frame.getHeight(),frame.getWidth(),CV_16UC1,(unsigned short*)frame.getData());

}

bool CDepthColorSensor::IsSane() {

  return m_depth_stream.isValid() && m_rgb_stream.isValid() && m_device.isValid();
}




bool CDepthColorSensor::StartRecording(const char* filename) {

  if(!IsSane()) {

    cout << "ERROR: Could not open device." << endl;
    return 1;

  }

  m_recorder.create(filename);

  // attach streams to recorder (maybe do this later)
  m_recorder.attach(m_rgb_stream);
  m_recorder.attach(m_depth_stream);

  if(!m_recorder.isValid()) {

    cout << "Recorder not valid." << endl;
    return 1;

  }

  m_recorder.start();

  if(!m_recorder.isValid()) {

    cout << "Could not start capture." << endl;
    return 1;

  }

  return 0;

}

bool CDepthColorSensor::StopRecording() {

  m_recorder.stop();
  m_recorder.destroy();

  return 0;

}


