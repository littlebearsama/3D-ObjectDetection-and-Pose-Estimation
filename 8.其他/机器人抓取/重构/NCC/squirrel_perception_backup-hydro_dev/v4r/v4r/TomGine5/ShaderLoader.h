//
//  ShaderLoader.h
//  PlaneSegmentation
//
//  Created by Simon Schreiberhuber on 03.04.14.
//  Copyright (c) 2014 Simon Schreiberhuber. All rights reserved.
//

#ifndef __PlaneSegmentation__ShaderLoader__
#define __PlaneSegmentation__ShaderLoader__

#include <iostream>
#include "GLError.h"
#include <GL/glew.h>

namespace tg{

//Just a container for 2 functions that load and link shader while processing error messages
class ShaderLoader{
private:
public:
    static GLint loadShader(GLenum type,std::string file);
    static void linkProgram(GLuint program);
    //static GLuint compileShader();
};
//http://stackoverflow.com/questions/5415788/read-write-xml-file-in-c
//http://pugixml.org/
};
#endif /* defined(__PlaneSegmentation__ShaderLoader__) */
