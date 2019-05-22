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


/*
 *I have to admit: this code comes from the OpenGL 4 Shading Languaga Cookbook
 *
 */
#ifndef GLSLPROGRAM_H
#define GLSLPROGRAM_H


#include <GL/glew.h>



//#include "cookbookogl.h"

#include <string>
using std::string;
#include <map>

#include <glm/glm.hpp>
using glm::vec2;
using glm::vec3;
using glm::vec4;
using glm::mat4;
using glm::mat3;

#include <stdexcept>

namespace tg{
class GLSLProgramException : public std::runtime_error {
  public:
    GLSLProgramException( const string & msg ) :
      std::runtime_error(msg) { }
};

namespace GLSLShader {
  enum GLSLShaderType {
    VERTEX = GL_VERTEX_SHADER,
    FRAGMENT = GL_FRAGMENT_SHADER,
    GEOMETRY = GL_GEOMETRY_SHADER,
    TESS_CONTROL = GL_TESS_CONTROL_SHADER,
    TESS_EVALUATION = GL_TESS_EVALUATION_SHADER,
    COMPUTE = GL_COMPUTE_SHADER
  };
}

class GLSLProgram
{
  private:
    int  handle;
    bool linked;
    std::map<string, int> uniformLocations;


    bool fileExists( const string & fileName );
    string getExtension( const char * fileName );

    // Make these private in order to make the object non-copyable
    GLSLProgram( const GLSLProgram & other ) { }
    GLSLProgram & operator=( const GLSLProgram &other ) { return *this; }

  public:
    GLSLProgram();
    ~GLSLProgram();

    void   compileShader( std::string fileName ) throw (GLSLProgramException);
    void   compileShader( const char *fileName ) throw (GLSLProgramException);
    void   compileShader( const char * fileName, GLSLShader::GLSLShaderType type ) throw (GLSLProgramException);
    void   compileShader( const string & source, GLSLShader::GLSLShaderType type,
        const char *fileName = NULL ) throw (GLSLProgramException);

    void   link() throw (GLSLProgramException);
    void   validate() throw(GLSLProgramException);
    void   use() throw (GLSLProgramException);

    int    getHandle();
    bool   isLinked();

    void   bindAttribLocation( GLuint location, const char * name);
    void   bindFragDataLocation( GLuint location, const char * name );

    void   setUniform( const char *name, float x, float y, float z);
    void   setUniform( const char *name, const vec2 & v);
    void   setUniform( const char *name, const vec3 & v);
    void   setUniform( const char *name, const vec4 & v);
    void   setUniform( const char *name, const mat4 & m);
    void   setUniform( const char *name, const mat3 & m);
    void   setUniform( const char *name, float val );
    void   setUniform( const char *name, int val );
    void   setUniform( const char *name, bool val );
    void   setUniform( const char *name, GLuint val );

    GLint getAttribLocation(std::string attribName);
    GLint getUniformLocation(std::string uniformName);
    GLint  getUniformLocation(const char * name );


    void   printActiveUniforms();
    void   printActiveUniformBlocks();
    void   printActiveAttribs();

    const char * getTypeString( GLenum type );
};
}


#endif // GLSLPROGRAM_H
