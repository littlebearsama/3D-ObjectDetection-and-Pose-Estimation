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

// http://renderingpipeline.com/2012/05/windowless-opengl/

#include "GLWindow.h"
#include <stdio.h>
#include <stdexcept>
#include <vector>

namespace TomGine {

void GLWindow::init(unsigned int width, unsigned int height, const char* name, bool threaded,
    bool stereo)
{

  XInitThreads();

  dpy = XOpenDisplay(NULL);

  if (dpy == NULL) {
    throw std::runtime_error("[GLWindow::init] Error cannot connect to X server");
  }

  root = DefaultRootWindow(dpy);

  if (stereo) {
    GLint att[] = { GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, GLX_STEREO, None };
    vi = glXChooseVisual(dpy, 0, att);
  } else {
    GLint att[] = { GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None };
    vi = glXChooseVisual(dpy, 0, att);
  }

  if (vi == NULL)
    throw std::runtime_error("[GLWindow::init] Error no appropriate visual found");

  cmap = XCreateColormap(dpy, root, vi->visual, AllocNone);

  swa.colormap = cmap;
  swa.event_mask = ExposureMask | KeyPressMask | KeyReleaseMask | ButtonPressMask
      | ButtonReleaseMask | PointerMotionMask | StructureNotifyMask;

  glWin = XCreateWindow(dpy, root, 0, 0, width, height, 0, vi->depth, InputOutput, vi->visual,
      CWColormap | CWEventMask, &swa);
  wmDelete = XInternAtom(dpy, "WM_DELETE_WINDOW", true);
  XSetWMProtocols(dpy, glWin, &wmDelete, 1);

  XMapWindow(dpy, glWin);
  XStoreName(dpy, glWin, name);

  glc = glXCreateContext(dpy, vi, NULL, GL_TRUE);
  glXMakeCurrent(dpy, glWin, glc);

  printf("GLWindow '%s' %dx%d\nOpenGL Version: %s\n", name, width, height, glGetString(GL_VERSION));

  /* Load the font. */
#ifdef GLX_FONT
  font_info = XLoadQueryFont(dpy, GLX_FONT);
  font_base = glGenLists(256);
  if (!font_info) {
    fprintf(stderr, "XLoadQueryFont() failed - Exiting.\n");
    exit(-1);
  }
  else {
    /* Tell GLX which font & glyphs to use. */
    int first = font_info->min_char_or_byte2;
    int last = font_info->max_char_or_byte2;
    glXUseXFont(font_info->fid, first, last-first+1, font_base+first);
  }
#endif

  this->threaded = threaded;

  glXSwapBuffers(dpy, glWin);
}

void GLWindow::quit()
{
  glXMakeCurrent(dpy, None, NULL);
#ifdef GLX_FONT
  glDeleteLists(font_base, 256);
  XFreeFont(dpy, font_info);
#endif
  glXDestroyContext(dpy, glc);
  XFlush(dpy);
  XDestroyWindow(dpy, glWin);
  XFreeColormap(dpy, cmap);
  XCloseDisplay(dpy);
}

GLWindow::GLWindow(unsigned int width, unsigned int height, const char* name, bool threaded,
    bool stereo)
{
  init(width, height, name, threaded, stereo);
}
GLWindow::~GLWindow()
{
  quit();
}

void GLWindow::Activate()
{
  glXMakeCurrent(dpy, glWin, glc);
}

void GLWindow::Update()
{
  glXSwapBuffers(dpy, glWin);
}

} /* namespace */
