/*
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (c) 2014, Simon Schreiberhuber
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
 * @author simon.schreiberhuber
 *
 */
#include <GL/glew.h>
#include "GLUtils.h"

#include <cstdio>
#include <string>
#include <assert.h>
using std::string;

namespace tg {
namespace GLUtils{

void debugOpenGLCallback( GLenum source, GLenum type, GLuint id,
    GLenum severity, GLsizei length, const GLchar * msg, const void * param ) {

    string sourceStr;
    switch(source) {
    case GL_DEBUG_SOURCE_WINDOW_SYSTEM:
        sourceStr = "WindowSys";
        break;
    case GL_DEBUG_SOURCE_APPLICATION:
        sourceStr = "App";
        break;
    case GL_DEBUG_SOURCE_API:
        sourceStr = "OpenGL";
        break;
    case GL_DEBUG_SOURCE_SHADER_COMPILER:
        sourceStr = "ShaderCompiler";
        break;
    case GL_DEBUG_SOURCE_THIRD_PARTY:
        sourceStr = "3rdParty";
        break;
    case GL_DEBUG_SOURCE_OTHER:
        sourceStr = "Other";
        break;
    default:
        sourceStr = "Unknown";
    }

    string typeStr;
    switch(type) {
    case GL_DEBUG_TYPE_ERROR:
        typeStr = "Error";
        break;
    case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
        typeStr = "Deprecated";
        break;
    case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
        typeStr = "Undefined";
        break;
    case GL_DEBUG_TYPE_PORTABILITY:
        typeStr = "Portability";
        break;
    case GL_DEBUG_TYPE_PERFORMANCE:
        typeStr = "Performance";
        break;
    case GL_DEBUG_TYPE_MARKER:
        typeStr = "Marker";
        break;
    case GL_DEBUG_TYPE_PUSH_GROUP:
        typeStr = "PushGrp";
        break;
    case GL_DEBUG_TYPE_POP_GROUP:
        typeStr = "PopGrp";
        break;
    case GL_DEBUG_TYPE_OTHER:
        typeStr = "Other";
        break;
    default:
        typeStr = "Unknown";
    }

    string sevStr;
    switch(severity) {
    case GL_DEBUG_SEVERITY_HIGH:
        sevStr = "HIGH";
        break;
    case GL_DEBUG_SEVERITY_MEDIUM:
        sevStr = "MED";
        break;
    case GL_DEBUG_SEVERITY_LOW:
        sevStr = "LOW";
        break;
    case GL_DEBUG_SEVERITY_NOTIFICATION:
        sevStr = "NOTIFY";
        break;
    default:
        sevStr = "UNK";
    }

    printf("%s:%s[%s](%d): %s\n", sourceStr.c_str(), typeStr.c_str(), sevStr.c_str(),
        id, msg);
}


int checkForOpenGLError(std::string msg) {
    //
    // Returns 1 if an OpenGL error occurred, 0 otherwise.
    //
    GLenum glErr;
    int    retCode = 0;

    glErr = glGetError();
    while (glErr != GL_NO_ERROR)
    {
        const char * message = "";
        switch( glErr )
        {
        case GL_INVALID_ENUM:
            message = "GL_INVALID_ENUM";
            break;
        case GL_INVALID_VALUE:
            message = "GL_INVALID_VALUE";
            break;
        case GL_INVALID_OPERATION:
            message = "GL_INVALID_OPERATION";
            break;
        case GL_INVALID_FRAMEBUFFER_OPERATION:
            message = "GL_INVALID_FRAMEBUFFER_OPERATION";
            break;
        case GL_OUT_OF_MEMORY:
            message = "GL_OUT_OF_MEMORY";
            break;
        default:
            message = "Unknown OpenGL error";
        }

        printf("%s Error: %s\n", msg.c_str(), message);
        retCode = 1;
        glErr = glGetError();
#ifdef DEBUG
  assert( 0 );
#endif
    }
    return retCode;
}



int checkOpenGLFramebufferStatus(std::string msg,GLenum tgtFBO){
    GLenum glErr;
    glErr=glCheckFramebufferStatus( tgtFBO );
    if (glErr){
        const char * message = "";
        switch ( glErr )
        {
        case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
            message = "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
            message = "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
            message = "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
            message = "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER";
            break;
        case GL_FRAMEBUFFER_UNSUPPORTED:
            message = "GL_FRAMEBUFFER_UNSUPPORTED";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
            message = "GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE";
            break;
        case GL_FRAMEBUFFER_UNDEFINED:
            message = "GL_FRAMEBUFFER_UNDEFINED";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
            message = "GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS";
            break;
        case GL_FRAMEBUFFER_COMPLETE:
            message = "GL_FRAMEBUFFER_COMPLETE";
            break;
        default:
            break;
        }
        return 1;
         printf("%s Error: %s\n", msg.c_str(), message);
        #ifdef DEBUG
         assert( 0 );
        #endif
    }
    return 0;

}

void dumpGLInfo(bool dumpExtensions) {
    const GLubyte *renderer = glGetString( GL_RENDERER );
    const GLubyte *vendor = glGetString( GL_VENDOR );
    const GLubyte *version = glGetString( GL_VERSION );
    const GLubyte *glslVersion = glGetString( GL_SHADING_LANGUAGE_VERSION );

    GLint major, minor;
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    glGetIntegerv(GL_MINOR_VERSION, &minor);

    printf("-------------------------------------------------------------\n");
    printf("GL Vendor    : %s\n", vendor);
    printf("GL Renderer  : %s\n", renderer);
    printf("GL Version   : %s\n", version);
    printf("GL Version   : %d.%d\n", major, minor);
    printf("GLSL Version : %s\n", glslVersion);
    printf("-------------------------------------------------------------\n");

    if( dumpExtensions ) {
        GLint nExtensions;
        glGetIntegerv(GL_NUM_EXTENSIONS, &nExtensions);
        for( int i = 0; i < nExtensions; i++ ) {
            printf("%s\n", glGetStringi(GL_EXTENSIONS, i));
        }
    }
}
} // namespace GLUtils
} // namespace tg
