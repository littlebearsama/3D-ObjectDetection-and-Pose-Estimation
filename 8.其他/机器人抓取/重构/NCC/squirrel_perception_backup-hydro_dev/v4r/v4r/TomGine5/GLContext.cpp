/*
 * GLContext.cpp
 *
 *  Created on: Apr 3, 2014
 *      Author: simon
 */



//#include <GL/glew.h>

#include "GLContext.h"
#include <X11/X.h>
#include <X11/Xlib.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include <stdio.h>
#include <cstdlib>

//#include <GLFW/glfw3.h>
#define OpenGLInit( )\
{\
    glewExperimental=GL_TRUE;\
    glewInit();\
    glGetError();\
}


using namespace tg;

GLContext::GLContext(){
	//http://sidvind.com/wiki/Opengl/windowless

	GLint attr[] = { GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None };
	XVisualInfo *vi;

	// open display
	if ( ! (dpy = XOpenDisplay(NULL)) ) {
		fprintf(stderr, "cannot connect to X server\n\n");
	    exit(1);
	}
	// get root window
	root = DefaultRootWindow(dpy);

	// get visual matching attr
	if( ! (vi = glXChooseVisual(dpy, 0, attr)) ) {
		   fprintf(stderr, "no appropriate visual found\n\n");
		   exit(1);
	}
	// create a context using the root window
	if ( ! (glc = glXCreateContext(dpy, vi, NULL, GL_TRUE)) ){
		  fprintf(stderr, "failed to create context\n\n");
		  exit(1);
	}
	glXMakeCurrent(dpy, root, glc);
	// try it out, remember to *NOT* render to the default framebuffer!


    OpenGLInit();
#ifdef DEBUG
    tg::GLUtils::dumpGLInfo();
#endif
}
GLContext::~GLContext(){
    //delete context;
    glXDestroyContext(dpy,glc);
    XCloseDisplay(dpy);

}
void GLContext::makeCurrent(){
	glXMakeCurrent(dpy,root,glc);
}

