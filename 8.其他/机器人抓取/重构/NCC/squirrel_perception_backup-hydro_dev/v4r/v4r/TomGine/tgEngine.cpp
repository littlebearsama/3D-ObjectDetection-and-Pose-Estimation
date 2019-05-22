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

#include "tgEngine.h"
#include "tgError.h"
#include <stdexcept>

using namespace TomGine;

tgEngine::tgEngine(unsigned width, unsigned height, float far, float near, const char* name, bool bfc, bool threaded,
                   bool stereo)
{
  m_window = new GLWindow(width, height, name, threaded, stereo);
  g_font->SetGLXFontBase(m_window->GetFontBase());

  m_width = width;
  m_height = height;
  m_far = far;
  m_near = near;
  m_bfc = bfc;
  m_stereo = stereo;

  m_input_rotation_speed = 1.0f;
  m_input_translation_speed = 1.0f;
  m_input_zoom_speed = 1.0f;

  m_cor = vec3(0.0, 0.0, 0.0);

  m_button_left = false;
  m_button_middle = false;
  m_button_right = false;

  m_wireframe = false;
  m_smoothshading = false;
  m_image_show = false;
  m_activate2D = false;
  m_light0_moving = true;

  m_mouse_pos[0] = 0;
  m_mouse_pos[1] = 0;

  m_light0.Activate();

  float da = 0.25f * (m_far - m_near);
  // Setup 3D camera
  m_camera.Set(vec3(da, da, da), // Position of camera
      vec3(0, 0, 0), // Point where camera looks at
      vec3(0, 1, 0), // UP-Vector of Camera
      45, m_width, m_height, // field of view in degree in y, image width, image height
      m_near, m_far, // near clipping plane, far clipping plane
      tgCamera::GL_PERSPECTIVE); // Perspective camera
  UpdateCameraViews(m_camera);

  // Setup 2D camera
  m_cam_ortho.Set(vec3(0, 0, 1), vec3(0, 0, 0), vec3(0, 1, 0), 45, m_width, m_height, 0.1f, 2.0f, tgCamera::GL_ORTHO);

  m_image = 0;
  // 	Welcome();

  // OpenGL settings
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_DEPTH_TEST);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glShadeModel(GL_SMOOTH);
  glDisable(GL_TEXTURE_2D);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);
}

tgEngine::~tgEngine()
{
  if (m_window)
    delete (m_window);
  //  if (m_NurbsSurfaceShader)
  //    delete (m_NurbsSurfaceShader);
  if (m_image)
    delete (m_image);
}

void tgEngine::Welcome()
{
  printf("\n   ***   TomGine Render Engine   ***\n\n");
  printf("rotate: left mouse button\n");
  printf("slide:  right mouse button\n");
  printf("zoom:   mouse wheel\n");
  printf("[f]:    toggle shading mode\n");
  printf("[w]:    draw wireframed\n");
  printf("[esc]:	quit\n");
  printf("\n");
}

void tgEngine::Update()
{
  float fTime;
  Update(fTime);
}

void tgEngine::Update(float &fTime)
{
  // swap to screen and clear render buffer
  Swap(fTime);

  Activate3D();

  if (m_light0_moving)
  {
    vec3 cam_p = m_camera.GetPos();
    m_light0.position = vec4(cam_p.x, cam_p.y, cam_p.z, 1.0f);
  }
  m_light0.Activate();
}

bool tgEngine::ProcessEvents()
{
  Event event;
  bool run = true;
  std::vector<Event> eventlist;
  while (m_window->GetEvent(event))
  {
    eventlist.push_back(event);
    run = run && InputControl(event);
  }
  return run;
}

bool tgEngine::GetEventList(std::vector<Event> &eventlist)
{
  Event event;
  bool run = true;
  eventlist.clear();
  while (m_window->GetEvent(event))
  {
    eventlist.push_back(event);
    run = run && InputControl(event);
  }
  return run;
}

void tgEngine::WaitForEvent(Event &event)
{
  m_window->GetEventBlocking(event);
}

void tgEngine::UnWaitForEvent()
{
  m_window->UnBlockGetEvent();
}

bool tgEngine::InputControl(Event &event)
{
  int x_rel = 0;
  int y_rel = 0;

  switch (event.type) {

  // *********************************************************
  //			KeyCode:		/usr/include/X11/keysymdef.h
  case TMGL_Quit:
    return false;
    break;
  case TMGL_Press:
    // 			printf("event.key.keysym: %x\n", event.key.keysym);
    //    printf("[tgEngine::InputControl] Pressed: %d\n", event.input);
    switch (event.input) {
    case TMGL_Escape:
      return false;
      break;
    case TMGL_w:
      if (m_wireframe)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      else
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      m_wireframe = !m_wireframe;
      break;
    case TMGL_KP_0:
    case TMGL_z:
      m_camera = m_cam[0];
      break;
    case TMGL_KP_7:
      m_camera = m_cam[1];
      break;
    case TMGL_KP_6:
      m_camera = m_cam[2];
      break;
    case TMGL_KP_4:
      m_camera = m_cam[3];
      break;
    case TMGL_KP_2:
      m_camera = m_cam[4];
      break;
    case TMGL_KP_8:
      m_camera = m_cam[5];
      break;

    case TMGL_Button1:
      m_button_left = true;
      break;
    case TMGL_Button2:
      m_button_middle = true;
      break;
    case TMGL_Button3:
      m_button_right = true;
      break;
    case TMGL_Button4:
      m_camera.TranslateF(0.01f * (m_far - m_near) * m_input_zoom_speed);
      break;
    case TMGL_Button5:
      m_camera.TranslateF(-0.01f * (m_far - m_near) * m_input_zoom_speed);
      break;
    default:
      break;
    }
    break;

    // *********************************************************
  case TMGL_Release:
    switch (event.input) {
    case TMGL_Button1:
      m_button_left = false;
      break;
    case TMGL_Button2:
      m_button_middle = false;
      break;
    case TMGL_Button3:
      m_button_right = false;
      break;
    default:
      break;
    }
    break;

    // *********************************************************
  case TMGL_Motion:
    x_rel = event.motion.x - m_mouse_pos[0];
    y_rel = event.motion.y - m_mouse_pos[1];

    if (m_button_left)
    {
      m_camera.Orbit(m_cor, m_camera.GetU(), -0.05f * x_rel * m_input_rotation_speed);
      m_camera.Orbit(m_cor, m_camera.GetS(), -0.05f * y_rel * m_input_rotation_speed);
    } else if (m_button_right)
    {
      m_camera.TranslateS(-0.0005f * (m_far - m_near) * x_rel * m_input_translation_speed);
      m_camera.TranslateU(0.0005f * (m_far - m_near) * y_rel * m_input_translation_speed);
    } else if (m_button_middle)
    {
      m_camera.TranslateF(0.001f * (m_far - m_near) * x_rel * m_input_zoom_speed);
      m_camera.TranslateF(0.001f * (m_far - m_near) * y_rel * m_input_zoom_speed);
    }

    m_mouse_pos[0] = event.motion.x;
    m_mouse_pos[1] = event.motion.y;

    break;

    // *********************************************************
  case TMGL_Expose:
    m_width = event.expose.width;
    m_height = event.expose.height;
    m_camera.SetViewport(m_width, m_height);
    m_cam[0].SetViewport(m_width, m_height);
    m_cam[1].SetViewport(m_width, m_height);
    m_cam[2].SetViewport(m_width, m_height);
    m_cam[3].SetViewport(m_width, m_height);
    m_cam[4].SetViewport(m_width, m_height);
    m_cam[5].SetViewport(m_width, m_height);
    m_cam_ortho.Set(vec3(0, 0, 1), vec3(0, 0, 0), vec3(0, 1, 0), 45, m_width, m_height, 0.1f, 2.0f, tgCamera::GL_ORTHO);
    break;

    // *********************************************************
    // 		case ClientMessage:
    // 			if(event.clientmessage.stop)
    // 				return false;
    // 			break;

    // *********************************************************
  default:
    break;

  } // switch(event.type)

  return true;
}

void tgEngine::DrawCoordinates(float linelength, float linewidth)
{
  tgPose p;
  p.DrawCoordinates(linelength, linewidth);
}

void tgEngine::DrawFPS(float interval)
{
  static float fps = 0.0f;
  static float dt = 0.0f;
  dt += m_frametime;

  if (dt > interval)
  {
    dt = 0.0f;
    fps = 1.0f / m_frametime;
  }

  bool activate2D = m_activate2D;
  Activate2D();
  char charbuffer[16];
  sprintf(charbuffer, "%d", (int) (fps));
  g_font->Print(charbuffer, 20, 10, 10);
  if (!activate2D)
    Activate3D();
}

void tgEngine::SetCamera(tgCamera cam)
{
  m_width = cam.GetWidth();
  m_height = cam.GetHeight();
  m_far = cam.GetZFar();
  m_near = cam.GetZNear();
  m_camera = cam;
  UpdateCameraViews(cam);
}

void tgEngine::UpdateCameraViews(tgCamera cam)
{
  m_cam[5] = m_cam[4] = m_cam[3] = m_cam[2] = m_cam[1] = m_cam[0] = cam;
  m_cam[1].Orbit(m_cor, m_cam[1].GetU(), M_PI);
  m_cam[2].Orbit(m_cor, m_cam[2].GetU(), M_PI * 0.5);
  m_cam[3].Orbit(m_cor, m_cam[3].GetU(), -M_PI * 0.5);
  m_cam[4].Orbit(m_cor, m_cam[4].GetS(), M_PI * 0.5);
  m_cam[5].Orbit(m_cor, m_cam[5].GetS(), -M_PI * 0.5);
}

void tgEngine::SetCamera(cv::Mat &intrinsic, unsigned &width, unsigned &height, cv::Mat &R, cv::Mat &T)
{
  TomGine::tgCamera::Parameter param;

  if (intrinsic.empty())
  {
    printf("[tgEngine::SetCameraIntrinsic] Warning, argument 'intrinsic' not valid (empty).\n");
    return;
  }

  if (intrinsic.rows < 3 || intrinsic.cols < 3)
  {
    printf("[tgEngine::SetCameraIntrinsic] Warning, argument 'intrinsic' not valid (size).\n");
    return;
  }

  if (R.empty() || T.empty())
  {
    printf("[tgEngine::SetCameraPose] Warning, arguments 'R, T' not valid (empty).\n");
    return;
  }

  if (R.rows < 3 || R.cols < 3)
  {
    printf("[tgEngine::SetCameraPose] Warning, argument 'R' not valid (size).\n");
    return;
  }

  if (T.rows < 3 && T.cols < 3)
  {
    printf("[tgEngine::SetCameraPose] Warning, argument 'T' not valid (size).\n");
    return;
  }

  bool activate2D = m_activate2D;
  param.width = width;
  param.height = height;
  param.zFar = m_camera.GetZFar();
  param.zNear = m_camera.GetZNear();

  // Instrinsic parameters:
  // entries of the camera matrix
  cv::Mat intrinsic32 = intrinsic;
  if (intrinsic.type() != CV_32F)
    intrinsic.convertTo(intrinsic32, CV_32F);
  float *d = intrinsic32.ptr<float> (0);
  param.fx = d[0];
  param.fy = d[4];
  param.cx = d[2];
  param.cy = d[5];

  // radial distortion parameters
  param.k1 = 0.0;
  param.k2 = 0.0;
  param.k3 = 0.0;
  // tangential distortion parameters
  param.p1 = 0.0;
  param.p2 = 0.0;

  cv::Mat R32 = R;
  if (R.type() != CV_32F)
    R.convertTo(R32, CV_32F);
  param.rot = mat3(R32.ptr<float> (0));

  cv::Mat T32 = T;
  if (T.type() != CV_32F)
    T.convertTo(T32, CV_32F);
  param.pos = vec3(T32.ptr<float> (0));

  m_camera.Set(param);
  UpdateCameraViews(m_camera);
  if (activate2D)
    Activate2D();
  else
    Activate3D();
}

void tgEngine::SetCenterOfRotation(float x, float y, float z)
{
  m_cor = vec3(x, y, z);
}

void tgEngine::ActivateLeft()
{
  if (m_stereo)
    glDrawBuffer(GL_BACK_LEFT);
}

void tgEngine::ActivateRight()
{
  if (m_stereo)
    glDrawBuffer(GL_BACK_RIGHT);
}

void tgEngine::Activate3D()
{
  if (m_bfc)
    glEnable(GL_CULL_FACE);
  else
    glDisable(GL_CULL_FACE);
  m_camera.ApplyTransform();
  m_camera.Activate();
  glEnable(GL_LIGHTING);
  m_activate2D = false;

  m_light0.Activate();
}

void tgEngine::Activate2D()
{
  glDisable(GL_CULL_FACE);
  m_cam_ortho.Activate();
  glDisable(GL_LIGHTING);
  m_activate2D = true;
}

void tgEngine::Swap()
{
  float fTime;
  Swap(fTime);
}

void tgEngine::Swap(float &fTime)
{
  // update frametime
  fTime = m_frametime = (float) m_timer.Update();
  m_window->Update();
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
}

void tgEngine::LoadBackgroundImage(unsigned char* image_data, int width, int height, GLenum format, bool flip)
{

  if (!m_image)
    m_image = new tgTexture2D();

  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
  m_image->Load(image_data, width, height, GL_RGB, format, GL_UNSIGNED_BYTE);
  m_image_show = true;
  m_image_flip = flip;
}

void tgEngine::DrawBackgroundImage(tgTexture2D *image, bool flip)
{
  if (image == NULL)
    image = m_image;

  if (image)
  {
    float alpha = 1.0f;
    float w = (float) m_width;
    float h = (float) m_height;
    bool activate2D = m_activate2D;
    Activate2D();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    if (alpha < 1.0f)
      glEnable(GL_BLEND);

    glDepthMask(0);
    glColor4f(1.0f, 1.0f, 1.0f, alpha);
    image->Bind();
    if (flip)
    {
      glBegin(GL_QUADS);
      glTexCoord2f(0.0f, 1.0f);
      glVertex3f(0., 0., 0.0f);
      glTexCoord2f(1.0f, 1.0f);
      glVertex3f(w, 0., 0.0f);
      glTexCoord2f(1.0f, 0.0f);
      glVertex3f(w, h, 0.0f);
      glTexCoord2f(0.0f, 0.0f);
      glVertex3f(0., h, 0.0f);
      glEnd();
    } else
    {
      glBegin(GL_QUADS);
      glTexCoord2f(0.0f, 0.0f);
      glVertex3f(0., 0., 0.0f);
      glTexCoord2f(1.0f, 0.0f);
      glVertex3f(w, 0., 0.0f);
      glTexCoord2f(1.0f, 1.0f);
      glVertex3f(w, h, 0.0f);
      glTexCoord2f(0.0f, 1.0f);
      glVertex3f(0., h, 0.0f);
      glEnd();
    }
    glDisable(GL_TEXTURE_2D);
    glDepthMask(1);
    glDisable(GL_BLEND);

    if (m_wireframe)
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    if (!activate2D)
      Activate3D();
  }
}

//void tgEngine::DrawForegroundImage()
//{
//  if (m_image)
//  {
//    float alpha = 1.0f;
//    float w = (float) m_width;
//    float h = (float) m_height;
//    bool activate2D = m_activate2D;
//    Activate2D();
//    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//
//    if (alpha < 1.0f)
//      glEnable(GL_BLEND);
//
//    glDisable(GL_DEPTH_TEST);
//    glColor4f(1.0f, 1.0f, 1.0f, alpha);
//    m_image->Bind();
//    if (m_image_flip)
//    {
//      glBegin(GL_QUADS);
//      glTexCoord2f(0.0f, 1.0f);
//      glVertex3f(0., 0., 0.0f);
//      glTexCoord2f(1.0f, 1.0f);
//      glVertex3f(w, 0., 0.0f);
//      glTexCoord2f(1.0f, 0.0f);
//      glVertex3f(w, h, 0.0f);
//      glTexCoord2f(0.0f, 0.0f);
//      glVertex3f(0., h, 0.0f);
//      glEnd();
//    } else
//    {
//      glBegin(GL_QUADS);
//      glTexCoord2f(0.0f, 0.0f);
//      glVertex3f(0., 0., 0.0f);
//      glTexCoord2f(1.0f, 0.0f);
//      glVertex3f(w, 0., 0.0f);
//      glTexCoord2f(1.0f, 1.0f);
//      glVertex3f(w, h, 0.0f);
//      glTexCoord2f(0.0f, 1.0f);
//      glVertex3f(0., h, 0.0f);
//      glEnd();
//    }
//    glDisable(GL_TEXTURE_2D);
//    glEnable(GL_DEPTH_TEST);
//    glDisable(GL_BLEND);
//
//    if (m_wireframe)
//      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//
//    if (!activate2D)
//      Activate3D();
//  }
//}

void tgEngine::UnloadBackgroundImage()
{
  m_image_show = false;
  if (m_image)
    delete (m_image);
  m_image = 0;
}

void tgEngine::PrintText(std::string text, float x, float y, float r, float g, float b)
{
  Activate2D();
  g_font->Print(text.c_str(), 15, x, y, r, g, b, 0.0f);
}

void tgEngine::PrintText3D(std::string text, vec3 pos, int size, float r, float g, float b, float a)
{
  vec2 vPos = m_camera.ToImageSpace(pos);
  bool activate2D = m_activate2D;
  Activate2D();
  g_font->Print(text.c_str(), size, vPos.x, vPos.y, r, g, b, a);
  if (!activate2D)
    Activate3D();
}

void tgEngine::GetPointCloud(cv::Mat_<cv::Vec4f> &points, bool world_coords)
{
  // get values of rendering pipeline
  mat4 proj, model;
  GLint view[4];
  glGetFloatv(GL_PROJECTION_MATRIX, proj);
  glGetFloatv(GL_MODELVIEW_MATRIX, model);
  glGetIntegerv(GL_VIEWPORT, view);

  points.create(view[3], view[2]);
  cv::Vec4f cvp;
  cvp[0] = NAN;
  cvp[1] = NAN;
  cvp[2] = NAN;
  cvp[3] = NAN;
  points.setTo(cvp);

  float depth_range[2];
  glGetFloatv(GL_DEPTH_RANGE, depth_range);

  mat3 Rt(model.transpose());
  vec3 t(model[12], model[13], model[14]);

  const float &fx = proj[0];
  const float &fy = proj[5];
  const float &cx = proj[8];
  const float &cy = proj[9];
  const float &z1 = proj[10];
  const float &z2 = proj[14];
  const float &dNear = depth_range[0];
  const float &dFar = depth_range[1];
  const int &width = view[2];
  const int &height = view[3];

  float zFar = z2 / (z1 + 1.0f);
  float zNear = z2 * zFar / (z2 - 2.0f * zFar);

  // grab depth buffer from OpenGL
  float *depthbuffer = (float*) malloc(sizeof(float) * width * height);
  glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, depthbuffer);

  RGBValue *colorbuffer = (RGBValue*) malloc(sizeof(RGBValue) * width * height);
  glReadPixels(0, 0, width, height, GL_BGRA, GL_UNSIGNED_BYTE, colorbuffer);

  // parse depth values and create 3D points in world coordinates
  for (int j = 0; j < height; j++)
  {
    for (int i = 0; i < width; i++)
    {
      int winY = j;
      if (world_coords)
        winY = height - j - 1; // inverse y coordinates

      int idx = (height - j - 1) * width + i; // depthbuffer index
      const float &z_b = depthbuffer[idx];
      const RGBValue &col = colorbuffer[idx];

      if (z_b > dNear && z_b < dFar) // depth range check
      {

        float z_n = 2.0 * z_b - 1.0; // normalized depth

        // transform to camera coordinates
        float z = 2.0 * zNear * zFar / (zFar + zNear - z_n * (zFar - zNear));
        float x = (2.0 * float(i) / width - 1.0f + cx) * z / fx;
        float y = (2.0 * float(winY) / height - 1.0f + cy) * z / fy;

        vec4 p(x, y, z, 1);
        if (world_coords) // transform to world coordinates
        {
          p.z = -p.z;
          p = Rt * (p - t);
        }
        p.w = col.float_value;

        cvp[0] = p.x;
        cvp[1] = p.y;
        cvp[2] = p.z;
        cvp[3] = p.w;

        points.at<cv::Vec4f>(j, i) = cvp;

      }
    }
  }
  free(depthbuffer);
  free(colorbuffer);
}

void tgEngine::GetPointCloud(std::vector<vec4> &points, bool organized, bool world_coords)
{
  points.clear();

  // get values of rendering pipeline
  mat4 proj, model;
  GLint view[4];
  glGetFloatv(GL_PROJECTION_MATRIX, proj);
  glGetFloatv(GL_MODELVIEW_MATRIX, model);
  glGetIntegerv(GL_VIEWPORT, view);

  float depth_range[2];
  glGetFloatv(GL_DEPTH_RANGE, depth_range);

  mat3 Rt(model.transpose());
  vec3 t(model[12], model[13], model[14]);

  const float &fx = proj[0];
  const float &fy = proj[5];
  const float &cx = proj[8];
  const float &cy = proj[9];
  const float &z1 = proj[10];
  const float &z2 = proj[14];
  const float &dNear = depth_range[0];
  const float &dFar = depth_range[1];
  const int &width = view[2];
  const int &height = view[3];

  float zFar = z2 / (z1 + 1.0f);
  float zNear = z2 * zFar / (z2 - 2.0f * zFar);

  // grab depth buffer from OpenGL
  float *depthbuffer = (float*) malloc(sizeof(float) * width * height);
  glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, depthbuffer);

  RGBValue *colorbuffer = (RGBValue*) malloc(sizeof(RGBValue) * width * height);
  glReadPixels(0, 0, width, height, GL_BGRA, GL_UNSIGNED_BYTE, colorbuffer);

  // parse depth values and create 3D points in world coordinates
  for (int j = 0; j < height; j++)
  {
    for (int i = 0; i < width; i++)
    {
      int winY = j;
      if (world_coords)
        winY = height - j - 1; // inverse y coordinates

      int idx = (height - j - 1) * width + i; // depthbuffer index
      const float &z_b = depthbuffer[idx];
      const RGBValue &col = colorbuffer[idx];

      if (z_b > dNear && z_b < dFar) // depth range check
      {

        float z_n = 2.0 * z_b - 1.0; // normalized depth

        // transform to camera coordinates
        float z = 2.0 * zNear * zFar / (zFar + zNear - z_n * (zFar - zNear));
        float x = (2.0 * float(i) / width - 1.0f - cx) * z / fx;
        float y = (2.0 * float(winY) / height - 1.0f - cy) * z / fy;

        vec4 p(x, y, z, 1);
        if (world_coords) // transform to world coordinates
        {
          p.z = -p.z;
          p = Rt * (p - t);
        }
        p.w = col.float_value;

        points.push_back(p);

      } else if (organized)
      {
        points.push_back(vec4(NAN, NAN, NAN, col.float_value));
      }
    }
  }
  free(depthbuffer);
  free(colorbuffer);
}

vec3 tgEngine::Get3DPointFrom2D(int x, int y)
{
  vec3 vResult;
  int viewport[4];
  double modelview[16];
  double projection[16];
  float z;
  int y_new;
  double result[3];
  bool activate2D = m_activate2D;

  Activate3D();
  m_camera.SetZRange(0.0, 1.0);

  glGetDoublev(GL_MODELVIEW_MATRIX, &modelview[0]); //Aktuelle Modelview Matrix in einer Variable ablegen
  glGetDoublev(GL_PROJECTION_MATRIX, &projection[0]); //Aktuelle Projection[s] Matrix in einer Variable ablegen
  glGetIntegerv(GL_VIEWPORT, &viewport[0]); // Aktuellen Viewport in einer Variable ablegen
  y_new = viewport[3] - y; // In OpenGL steigt Y von unten (0) nach oben

  // Auslesen des Tiefenpuffers an der Position (X/Y_new)
  glReadPixels(x, y_new, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);

  // Errechnen des Punktes, welcher mit den beiden Matrizen multipliziert (X/Y_new/Z) ergibt:
  gluUnProject((double) x, (double) y_new, (double) z, modelview, projection, viewport, &result[0], &result[1],
      &result[2]);

  vResult.x = (float) result[0];
  vResult.y = (float) result[1];
  vResult.z = (float) result[2];

  if (activate2D)
    Activate2D();

  return vResult;
}

