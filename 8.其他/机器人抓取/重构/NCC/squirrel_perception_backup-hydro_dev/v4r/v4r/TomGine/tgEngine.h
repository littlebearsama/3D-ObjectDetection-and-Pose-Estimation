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

#ifndef TG_ENGINE
#define TG_ENGINE

#include "headers.h"

#include <opencv2/core/core.hpp>

#include "GLWindow.h"
#include "tgCamera.h"
#include "tgLight.h"
#include "tgRenderModel.h"
#include "tgPose.h"
#include "tgTimer.h"
#include "tgMathlib.h"
#include "tgTexture.h"
#include "tgFont.h"

namespace TomGine {

/** @brief Render engine, OpenGL viewer. Manages camera movement, mouse- and keyboard-input, fonts and visualisation.
 *  This is no scene-graph. */
class tgEngine
{
protected:
  unsigned m_width;
  unsigned m_height;
  float m_far;
  float m_near;

  float m_input_rotation_speed;
  float m_input_translation_speed;
  float m_input_zoom_speed;

  tgTexture2D* m_image;

  tgCamera m_cam[6];
  tgCamera m_camera;
  tgCamera m_cam_ortho;

  tgLight m_light0;
  tgTimer m_timer;
  vec3 m_cor; ///< Center of Rotation

  int m_mouse_pos[2];

  float m_frametime;
  bool m_bfc;
  bool m_button_left, m_button_middle, m_button_right;
  bool m_wireframe;
  bool m_smoothshading;
  bool m_image_show;
  bool m_image_flip;
  bool m_activate2D;
  bool m_stereo;
  bool m_light0_moving;

  //  bool m_idle;
  //  unsigned m_interval_us;
  //  pthread_mutex_t dataMutex;
  //  pthread_t thread_idle;
  //  friend void* ThreadIdle(void* c);

public:
  /** @brief Initialising render engine.
   * Warning: OpenGL commands can only be used after initialisation and are not effective before.
   * @param width		Width of rendering window in pixels
   * @param height 	Height of rendering window in pixels
   * @param far 		Far clipping plane (distance above which objects are cut of).
   * @param near 		Near clipping plane (distance below which objects are cut of).
   * @param name 		Caption of Window.
   * @param bfc 		Enable / Disable back face culling (render back face of polygons or not).
   * @param threaded 	true if 'm_window' is threaded, false if not (please see GLWindow for further information). */
  tgEngine(unsigned width = 640, unsigned height = 480, float far = 10.0f, float near = 0.01f, const char* name = "TomGine",
           bool bfc = false, bool threaded = false, bool stereo = false);

  /** @brief Destroy render engine.
   * Warning: OpenGL commands can only be used before destruction and are not effective afterwards. */
  ~tgEngine();

  GLWindow* m_window; ///< GLX window embedding the OpenGL context used (see GLWindow() for more information).
  //  tgShader *m_NurbsSurfaceShader;  ///< shader for drawing NURBS


  /** @brief Welcome message */
  void Welcome();

  /** @brief Swaps background buffer to screen and resets rendering settings.
   * @param time Time in seconds since last Update() call  */
  void Update(float &fTime);
  /** @brief Swaps background buffer to screen and resets rendering settings. */
  void Update();

  /** @brief Apply keyboard and mouse input. (i.e. rotate, translate, zoom, draw wireframe, change cam, ...) */
  bool ProcessEvents();
  /** @brief Get list of events waiting in event que of this window. */
  bool GetEventList(std::vector<Event> &eventlist);
  /** @brief Wait for event in event que of this window (blocking). */
  void WaitForEvent(Event &event);
  /** @brief Unblocks function WaitForEvent(...). */
  void UnWaitForEvent();

  /** @brief Draw background image. */
  void DrawBackgroundImage(tgTexture2D *image = NULL, bool flip = false);

  //  /** @brief Draw foreground image. */
  //  void DrawForegroundImage();

  /** @brief Handles keyboard and mouse input applied to this window */
  bool InputControl(Event &event);

  /**	@brief Draws a simple coordinate frame */
  void DrawCoordinates(float linelength = 1.0f, float linewidth = 1.0f);

  /** @brief Display frames-per-second. */
  void DrawFPS(float interval = 0.5f);

  /** @brief Sets camera of rendering engine (including internal and external camera parameters) */
  void SetCamera(tgCamera cam);
  /** @brief Sets camera of rendering engine (including internal and external camera parameters)
   *  @param intrinsic	Intrinsic camera matrix (projective transformation from world in image space).
   *  @param width		Width of camera image in pixel
   *  @param height		Height of camera image in pixel
   *  @param R,T			Extrinsic camera parameters (pose of camera) */
  void SetCamera(cv::Mat &intrinsic, unsigned &width, unsigned &height, cv::Mat &R, cv::Mat &T);
  /** @brief Update all cameras relative to 'cam'. */
  void UpdateCameraViews(tgCamera cam);

  /**	@brief Sets center of rotation */
  void SetCenterOfRotation(float x, float y, float z);

  /** @brief Activate left drawing buffer (for stereo only) */
  void ActivateLeft();
  /** @brief Activate right drawing buffer (for stereo only) */
  void ActivateRight();

  /** @brief Activates 3D rendering mode; standard after Update() */
  void Activate3D();

  /** @brief Activates 2D rendering moder */
  void Activate2D();

  /** @brief Swaps frame buffer to screen (called by Update() aswell)*/
  void Swap();
  void Swap(float &fTime);

  /** @brief Load a background image to display
   *  @param image_data	Image pixel data according to 'width', 'height' and 'format'
   *  @param width,hight	Dimensions of image
   *  @param format		Format of image data (GL_RGB, GL_BGR, GL_RGBA, GL_LUMINANCE, ... see glTexImage2D in OpenGL spec.)	 */
  void LoadBackgroundImage(unsigned char* image_data, int width, int height, GLenum format = GL_RGB, bool flip = false);

  /** @brief Remove background image. */
  void UnloadBackgroundImage();

  /**	@brief Returns the actual position of the camera with respect to the coordinate frame */
  vec3 GetCameraPosition()
  {
    return m_camera.GetPos();
  }

  /** @brief get a copy of the current camera of the engine */
  tgCamera GetCamera()
  {
    return m_camera;
  }

  /** @brief get state of wireframe drawing mode (wireframe mode on/off) */
  bool GetWireframeMode()
  {
    return m_wireframe;
  }

  void PrintText(std::string text, float x, float y, float r = 1.0f, float g = 1.0f, float b = 1.0f);

  /** @brief Print a text in 3D world coordinates.
   *  @param text	Text to print.
   *  @param pos	3D position of the text.
   *  @param size	Size of the font.	 */
  void PrintText3D(std::string text, vec3 pos, int size = 16, float r = 1.0, float g = 1.0, float b = 1.0, float a = 1.0);

  /** @brief Print a text at a 2D image coordinates.
   *  @param text	Text to print.
   *  @param pos	2D position of the text.
   *  @param size	Size of the font.	 */
  //	void PrintText2D(std::string text, vec2 pos, int size=16);
  // USE g_font->Print() instead

  /** @brief Set input speed for mouse control (rotation). */
  void SetInputRotationSpeed(float v)
  {
    m_input_rotation_speed = v;
  }
  /** @brief Set input speed for mouse control (translation). */
  void SetInputTranslationSpeed(float v)
  {
    m_input_translation_speed = v;
  }
  /** @brief Set input speed for mouse control (zoom). */
  void SetInputZoomSpeed(float v)
  {
    m_input_zoom_speed = v;
  }
  /** @brief Set input speed for mouse control (rotation, translation, zoom). */
  void SetSpeeds(float rotation, float translation, float zoom)
  {
    m_input_rotation_speed = rotation;
    m_input_translation_speed = translation;
    m_input_zoom_speed = zoom;
  }

  /** @brief Set default scene lighting
   *  @param light  the light configuration
   *  @param moving (true == attached to camera; false == attached to scene) */
  inline void SetLight0(const tgLight& light, bool moving)
  {
    m_light0 = light;
    m_light0_moving = moving;
  }

  inline tgLight GetLight0()
  {
    return m_light0;
  }

  /** @brief Get point cloud of current view in world coordinates. */
  void GetPointCloud(cv::Mat_<cv::Vec4f> &points, bool world_coords);
  void GetPointCloud(std::vector<vec4> &points, bool organized = false, bool world_coords = true);

  /** @brief Get 3D point of nearest surface at image position (x,y) */
  vec3 Get3DPointFrom2D(int x, int y);

};

} // namespace TomGine

#endif
