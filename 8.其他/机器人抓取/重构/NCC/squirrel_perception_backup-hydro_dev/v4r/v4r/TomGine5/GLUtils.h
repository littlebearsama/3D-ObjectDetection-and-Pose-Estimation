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
#ifndef GLUTILS_H
#define GLUTILS_H

#include "GL/glew.h"

#include <string>

namespace tg
{
namespace GLUtils{
    int checkForOpenGLError(std::string msg);

    int checkOpenGLFramebufferStatus(std::string msg,GLenum tgtFBO=GL_FRAMEBUFFER);

    void dumpGLInfo(bool dumpExtensions = false);



    void debugOpenGLCallback( GLenum source, GLenum type, GLuint id,
        GLenum severity, GLsizei length, const GLchar * msg, const void * param );
}
}

#endif // GLUTILS_H
