/*
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (c) 2011, Thomas Mörwald
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author thomas.moerwald
 *
 */

#ifndef TGTHREAD_TOMGINE_THREAD_H
#define TGTHREAD_TOMGINE_THREAD_H

#include <opencv2/core/core.hpp>
#include <vector>
#include <list>
#include <string>
#include <semaphore.h>

#include "tgTomGine.h"

namespace TomGine {

/** @brief RGB-value of point clouds, accessable as float or long value. */
typedef union
{
  struct
  {
    unsigned char Blue; // Blue channel
    unsigned char Green; // Green channel
    unsigned char Red; // Red channel
    unsigned char Alpha; // Alpha channel
  };
  float float_value;
  long long_value;
} tgRGBValue;

class tgEventListener
{
public:
  virtual void EventFunction(TomGine::Event) {}
};

/** @brief Running TomGine rendering engine in an enclosed thread. No OpenGL commands are allowed outside of this thread. */
class tgTomGineThread
{
  enum ClearCommand
  {
    GL_CLEAR_NONE,
    GL_CLEAR_ALL,
    GL_CLEAR_MODELS_2D,
    GL_CLEAR_MODELS_3D,
    GL_CLEAR_MODEL_POINTERS,
    GL_CLEAR_POINTCLOUDS,
    GL_CLEAR_POINTS_2D,
    GL_CLEAR_POINTS_3D,
    GL_CLEAR_LINES_2D,
    GL_CLEAR_LINES_3D,
    GL_CLEAR_LABELS,
    GL_CLEAR_CAMERAS,
  };

  struct DrawFlags
  {
    bool image;
    bool cameras;
    bool pointclouds;
    bool lines;
    bool points;
    bool labels;
    bool models;
    bool normals;
    bool textured;
  };

public:
  DrawFlags df;
  ClearCommand m_clearcommand;
  int m_clearindex;

protected:

  // ------------ DATA ------------
  int m_width, m_height; // window width and height
  float m_depth_min, m_depth_max; // min and max depth for TomGine
  bool m_bfc; // enable, disable back-face-culling
  bool m_stopTomGineThread; // stop the tomeGine thread
  bool m_renderingStopped;
  bool m_eventsStopped;
  bool m_snapshot;
  bool m_record;


  bool m_loadImage;
  bool m_showCoordinateFrame; // show a coordinate frame
  bool m_camChanged;
//  bool m_tgCamChanged;
  bool m_inputSpeedChanged;
  bool m_rotCenterChanged;
  bool m_waitingForEvents;
  bool m_listenForEvents;
  bool m_waitForZBuffer;

  unsigned m_listenEventListSize;
  bool m_eventCallback;
  tgEventListener* m_eventListner;

  TomGine::tgEngine *m_engine;
  std::string m_windowname;
  std::list<Event> m_eventlist;
  std::list<Event> m_waitingeventlist;
  std::list<Event> m_listenEventList;

  cv::Mat m_img; // image
  cv::Mat m_intrinsic; // intrinsic parameters of the camera
  cv::Mat m_extR, m_extT; // extrinsic camera
  cv::Mat1f m_zBuffer;
//  TomGine::tgCamera m_tgCamera;
  TomGine::vec3 m_rotCenter; // rotation center
  TomGine::vec3 m_inputSpeeds;
  TomGine::vec3 m_clearColor;
  std::string m_snapshotFile;
  std::string m_recorderPath;
  unsigned m_recorderFrame;
  unsigned m_maxFPS;
  unsigned m_numScreenShot;
  bool m_FPS;
  float m_fTime;
  float m_normal_length;

  // Cameras
  std::vector<tgCamera> m_cameras;

  // Point2D
  std::vector<TomGine::tgColorPoint> m_points2D;
  std::vector<float> m_pointSize2D; // size of each 2D point

  // Point3D
  std::vector<TomGine::tgColorPoint> m_points3D;
  std::vector<float> m_pointSize3D; // size of each 3D point

  // Line2D
  std::vector<TomGine::tgLine> m_lines2D; // lines in 2D
  std::vector<TomGine::vec3> m_lineCols2D; // color of 2D lines
  std::vector<float> m_lineWidth2D; // probability of 2D lines

  // Line3D
  std::vector<TomGine::tgLine> m_lines3D; // lines in 3D
  std::vector<TomGine::vec3> m_lineCols3D; // color of 3D lines
  std::vector<float> m_lineWidth3D; // width of 3D lines

  // Labels
  std::vector<TomGine::tgLabel2D> m_labels2D;
  std::vector<TomGine::tgLabel3D> m_labels3D;

  std::vector<TomGine::tgTextureModel*> m_models2D;
  std::vector<TomGine::tgTextureModel*> m_models3D;
  std::vector<TomGine::tgModel*> m_modelpointers;
  std::vector<TomGine::tgModel*> m_pointclouds;

  // ------------ thread locks ------------
  pthread_t thread_gl; // tomGine thread_gl
  pthread_t thread_event; // thread for event handling (keyboard, mouse, resize, etc ...)
  pthread_mutex_t dataMutex; // mutex for data
  pthread_mutex_t eventMutex; // mutex for data
  sem_t renderSem;
  sem_t renderFinishedSem;
  sem_t clearFinishedSem;
  sem_t snapshotSem;

  //-------------- drawing --------------
  void GL_Record(TomGine::tgEngine *render);
  void GL_Snapshot(const std::string &filename);
  void GL_ZBuffer(cv::Mat1f &z, TomGine::tgEngine *render);
  void GL_Update(TomGine::tgEngine *render);
  virtual void GL_Draw(TomGine::tgEngine *render);

  virtual void GL_DrawCameras();
  virtual void GL_DrawPointCloud();
  virtual void GL_DrawPoints2D();
  virtual void GL_DrawPoints3D();
  virtual void GL_DrawLines2D();
  virtual void GL_DrawLines3D();
  virtual void GL_DrawLabels(TomGine::tgEngine *render);
  virtual void GL_DrawModels2D();
  virtual void GL_DrawModels3D(bool textured = true);

  //  void GL_SyncNurbsData();
  //  void GL_DrawNurbs();

  void GL_Clear();

  //-------------- threads --------------
  friend void* ThreadDrawing(void* c);
  friend void* ThreadEventHandling(void* c);

  //-------------- input --------------
  bool KeyHandler(Event &event);

  void init();

public:
  /** @brief Initialize class and start threads. */
  tgTomGineThread(int w, int h, std::string windowname = std::string("TomGine"), bool bfc = false,
      float depth_min = 0.01, float depth_max = 10.0);
  /** @brief Wait for all threads to finish, then destroy. */
  virtual ~tgTomGineThread();

  void PrintUsage();

  /** @brief Blocks and waits for a specific event.
   *  @param type Event type to wait for (e.g. TMGL_Release, TMGL_Press, TMGL_Motion)
   *  @param input Event input to wait for (e.g. TMGL_Button1, TMGL_a, TMGL_Escape) */
  TomGine::Event WaitForEvent(Type type, Input input);
  /** @brief Starts listening for events that can be queried using GetEventQueue(...)
   *  @param max_events maximum number of events queued while listening */
  void StartEventListener(unsigned max_events);
  void RegisterEventListener(tgEventListener* listener);
  /** @brief Stops listening for events */
  void StopEventListener();
  /** @brief Get current event queue recorded by event listener */
  void GetEventQueue(std::list<Event> &events, bool waiting=false);

  bool SelectModels(int x, int y, std::vector<int> &ids);
  bool SelectModels(int x, int y, const std::vector<int> &ids, int &id, double &z_min);
  bool SelectModel(int x, int y, int &id);

  /** @brief Returns the current camera */
  TomGine::tgCamera GetCamera();

  /** @brief Set the Background color. */
  void SetClearColor(float r, float g, float b);
  /** @brief Set the Background gray level. */
  void SetClearColor(float gray);
  /** @brief Set the intrinsic parameter of the TomGine virtual camera.
   * @see cv::calibrateCamera */
  void SetCamera(cv::Mat &_intrinsic);
  /** @brief Set extrinsic parameter of TomGine virtual camera (pose). */
  void SetCamera(cv::Mat &R, cv::Mat &t);
  /** @brief Set camera (OpenGL coordinates). */
  void SetCamera(const TomGine::tgCamera &cam);
  /** @brief Set default camera. */
  void SetCameraDefault();

  void LookAt(const vec3 &p);
  /** @brief Set point to rotate the camera about using the mouse. */
  void SetRotationCenter(const cv::Vec3d &_rotCenter);
  /** @brief Set point to rotate the camera about using the mouse. */
  void SetRotationCenter(float x, float y, float z);
  /** @brief Set point to rotate the camera about using the mouse. */
  void SetRotationCenter(const TomGine::vec3 &cor);
  /** @brief Enable/Disable drawing of coordinate frame. */
  void SetCoordinateFrame();
  /** @brief Set a background image. */
  void SetImage(unsigned char* data, unsigned width, unsigned height);
  void SetImage(const cv::Mat &_img, bool show = true, bool background = true);
  /** @brief Get the background image */
  bool GetImage(cv::Mat &_img);
  /** @brief Set speed of camera movement using the mouse (default 1.0,1.0,1.0). */
  void SetInputSpeeds(float rotation = 1.0f, float translation = 1.0f, float zoom = 1.0f);
  void GetInputSpeeds(float &rotation, float &translation, float &zoom);
  /** @brief Set the length of normals */
  void SetNormalLength (float n);
  /** @brief Set the frame rate of the render engine
   *  @param max_fps The maximum frame rate. If set to 0 the render engine will only update on changes and on
   *  the function call Update() */
  void SetFrameRate(unsigned max_fps);

  /** @brief    Add a camera to the scene
   *  @param    cam the camera to add   */
  int AddCamera(const tgCamera& cam);
  /** @brief	Add a point in 2D to the scene.
   *  @param	x,y		position of the point.
   *  @param	r,g,b	color of the point.
   *  @param	size	size of the point in pixel. */
  int AddPoint2D(float x, float y, uchar r = 255, uchar g = 0, uchar b = 0, float size = 1.0f);
  /** @brief	Add a point in 3D to the scene.
   *  @param	x,y,z	position of the point.
   *  @param	r,g,b	color of the point.
   *  @param	size	size of the point in pixel. */
  int AddPoint3D(float x, float y, float z, uchar r = 255, uchar g = 0, uchar b = 0,
      float size = 1.0f);
  /** @brief  Add a line in 2D to the scene.
   *  @param  x1,y1  Start point of the line.
   *  @param  x2,y2  End point of the line.
   *  @param  r,g,b   Color of the line.
   *  @param  width   Width of the line in pixel.  */
  int AddLine2D(float x1, float y1, float x2, float y2, uchar r = 255, uchar g = 0, uchar b = 0,
      float width = 1.0f);
  /** @brief	Add a line in 3D to the scene.
   *  @param	x1,y1,z1	Start point of the line.
   *  @param 	x2,y2,z2	End point of the line.
   *  @param	r,g,b		Color of the line.
   *  @param	width		Width of the line in pixel.	 */
  int AddLine3D(float x1, float y1, float z1, float x2, float y2, float z2, uchar r = 255,
      uchar g = 0, uchar b = 0, float width = 1.0f);
  /** @brief	Add a line in 3D to the scene.
   *  @param	x1,y1,z1	Start point of the line.
   *  @param 	x2,y2,z2	End point of the line.
   *  @param	r,g,b		Color of the line.
   *  @param	width		Width of the line in pixel.	 */
  int AddLine3D(const TomGine::tgLine& line, uchar r = 255, uchar g = 0, uchar b = 0, float width = 1.0f);
  /** @brief Add a label in 2D to the scene.
   *  @param text Character string of the label
   *  @param size The font size in points
   *  @param x,y The 2d position in pixels
   *  @param r,g,b The color of the label*/
  int AddLabel2D(std::string text, int size, int x, int y, float r = 1.0f, float g = 1.0f,
      float b = 1.0f);
  /** @brief Add a label in 3D to the scene.
   *  @param text Character string of the label
   *  @param size The font size in points
   *  @param pos The 3d position
   *  @param r,g,b The color of the label*/
  int AddLabel3D(std::string text, int size, vec3 pos, float r = 1.0f, float g = 1.0f,
      float b = 1.0f);
  /** @brief Add a label in 3D to the scene.
   *  @param text Character string of the label
   *  @param size The font size in points
   *  @param x,y,z The 3d position*/
  int AddLabel3D(std::string text, int size, double x, double y, double z);
  /** @brief Adds a copy of the model to the scene and calls the tgModel::Draw() function for rendering
   *  @param model The model to be added */
  int AddModel2D(const TomGine::tgTextureModel &model);
  /** @brief Adds a copy of the model to the scene and calls the tgModel::Draw() function for rendering
   *  @param model The model to be added */
  int AddModel(const TomGine::tgTextureModel &model);
  int AddModel3D(const TomGine::tgTextureModel &model);
  /** @brief	Adds a tgModel as pointer to the scene and calls the tgModel::Draw() function when rendered.
   *  @brief	Attention: Creation and destruction of model is in control of the user.
   *  Destruction of the model at a point where the OpenGL rendering thread (ThreadDrawing) is
   *  still running leads to segmentation faults. \n
   *  This function is meant to provide the user with an interface to use OpenGL commands
   *  within an derived class of tgModel, overwriting the virtual void tgModel::Draw().
   *  @param model	The pointer to a tgModel.
   *  @return	Unique id of the model added.*/
  int AddModel(TomGine::tgModel *model);
  /** @brief Adds a colored point cloud to the scene.
   *  @param cloud Cloud of points in OpenCV vector format. 4th entry is float-encoded RGBA color.
   *  @return Unique id of the point cloud added. */
  int AddPointCloud(cv::Mat_<cv::Vec4f> cloud, float pointsize=1.0);
  /** @brief Adds a colored point cloud to the scene.
   *  @param cloud Vector of points in OpenCV vector format. 4th entry is float-encoded RGBA color.
   *  @return Unique id of the point cloud added. */
  int AddPointCloud(const std::vector<cv::Vec4f> &cloud, float pointsize=1.0);
  /** @brief Adds a point cloud to the scene.
   *  @param cloud Vector of points in OpenCV vector format. 
   *  @return Unique id of the point cloud added. */
  int AddPointCloud(const cv::Mat_<cv::Vec3f> &cloud, uchar r, uchar g, uchar b, float pointsize);
  /** @brief Adds a point cloud to the scene.
   *  @param cloud Vector of points in OpenCV vector format. 
   *  @param image colour values of the cloud
   *  @return Unique id of the point cloud added. */
  int AddPointCloud(const cv::Mat_<cv::Vec3f> &cloud, const cv::Mat_<cv::Vec3b> &image, float pointsize);
  /** @brief  Set a point in 2D to the scene.
   *  @param  x,y   position of the point. */
  void SetPoint2D(int id, float x, float y);
  /** @brief  Set a point in 3D to the scene.
   *  @param  x,y,z   position of the point. */
  void SetPoint3D(int id, float x, float y, float z);
  /** @brief Sets a label in 2D to the scene.
   *  @param text Character string of the label
   *  @param size The font size in points
   *  @param x,y The 2d position in pixels
   *  @param r,g,b The color of the label*/
  void SetLabel2D(int id, std::string text, int size, int x, int y, float r = 1.0f, float g = 1.0f,
      float b = 1.0f);
  /** @brief Adds a copy of the model to the scene and calls the tgModel::Draw() function for rendering
   *  @param id The id of the model returned by AddModel().
   *  @param model The model to be added */
  void SetModel2D(int id, const TomGine::tgTextureModel &model);
  void SetModel3D(int id, const TomGine::tgTextureModel &model);
  void SetModelPose(int id, const TomGine::tgPose &pose);
  void SetModelColor(int id, uchar r, uchar g, uchar b);
  /** @brief  Sets the model pointer added with AddModel() to the model pointer specified.
   *  @param	id The id of the model returned by AddModel().
   *  @param  model The new model pointer. */
  void SetModel(int id, TomGine::tgModel *model);
  /** @brief	Sets the point cloud added with AddNurbsSurface().
   *  @param	id		The id of the point cloud.
   *  @param	pcl		The new point cloud.	 */
  void SetPointCloud(int id, const TomGine::tgModel &pcl);
  /** @brief	Sets the point cloud added with AddNurbsSurface().
   *  @param	id		The id of the point cloud.
   *  @param	pcl		The new point cloud.	 */
  void SetPointCloud(int id, cv::Mat_<cv::Vec4f> cloud);


  void RemoveAllModel3DFrom(int id);

  /** @brief	Trigger rendering thread to update data and draw it to screen. */
  void Update();
  /** @brief	Indicates if threads have stopped (finished). */
  bool Stopped();
  /** @brief	Takes a snapshot and saves it to file. */
  void Snapshot(const char* filename);
  /** @brief  Starts recording image sequences.
   *  @param  path  Path to folder where the images are stored
   *  @param  fps   Frame rate of recording */
  void StartRecording(std::string path, unsigned fps);
  /** @brief  Stops recording image sequences. */
  void StopRecording();

  void GetDepthBuffer(cv::Mat1f &z, TomGine::tgCamera &cam);

  /** @brief	Clear content of the scene. */
  virtual void Clear();
  /** @brief    Clear pointclouds of the scene. */
  virtual void ClearPointClouds();
  /** @brief    Clear cameras of the scene. */
  virtual void ClearCameras();
  /** @brief  Clear the points in the scene added with AddPoint2D(). (not point-clouds). */
  virtual void ClearPoints2D();
  /** @brief  Clear the points in the scene added with AddPoint3D(). (not point-clouds). */
  virtual void ClearPoints3D();
  /** @brief	Clear labels2D and labels3D in the scene. */
  virtual void ClearLabels();
  /** @brief  Clear the lines in the scene. */
  virtual void ClearLines2D();
  /** @brief	Clear the lines in the scene. */
  virtual void ClearLines3D();
  /** @brief Clear the models in the scene */
  virtual void ClearModels();
  virtual void ClearModels2D();
  virtual void ClearModels3D();
  virtual void ClearModels3DFrom(int id);
  virtual void ClearModelPointers();

  void DragDropModels(float speed);

};

}

#endif

