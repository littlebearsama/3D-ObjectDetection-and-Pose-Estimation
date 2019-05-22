//
//  GLContext.h
//  ConsoleOpenGLContext
//
//  Created by Simon Schreiberhuber on 25.03.14.
//  Copyright (c) 2014 Simon Schreiberhuber. All rights reserved.
//

#ifndef __OpenGLContext__GLContext__
#define __OpenGLContext__GLContext__

#include <iostream>

#include "GLUtils.h"
#include <X11/X.h>
#include <X11/Xlib.h>
#include <GL/gl.h>
#include <GL/glx.h>


namespace tg{


/**
 * @brief The GLContext class features an windowless OpenGL Context.
 */
class GLContext{
private:
#ifdef __APPLE__
    NSOpenGLContextWrapper* wrapper;
#else
	Display *dpy;
	Window root;
	GLXContext glc;
#endif
public:
    GLContext();
    ~GLContext();

    /**
     * @brief makeCurrent: Activates the OpenGL context. All GL calls are now in this context.
     */
    void makeCurrent();
};
}

#endif /* defined(__ConsoleOpenGLContext__GLContext__) */
