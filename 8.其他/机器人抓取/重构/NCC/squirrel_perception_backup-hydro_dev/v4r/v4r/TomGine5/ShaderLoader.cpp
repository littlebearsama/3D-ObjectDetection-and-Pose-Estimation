//
//  ShaderLoader.cpp
//  PlaneSegmentation
//
//  Created by Simon Schreiberhuber on 03.04.14.
//  Copyright (c) 2014 Simon Schreiberhuber. All rights reserved.
//

#include "ShaderLoader.h"
#include <fstream>
//#include "LilFile.h"
#include <stdio.h>

using namespace tg;

GLint ShaderLoader::loadShader(GLenum type, std::string file){
    GLuint shader=0;
    
    
    std::string filepath =file;
    std::string contents;

    std::ifstream in(filepath.c_str(), std::ios::in | std::ios::binary);
    if (in)
    {
        in.seekg(0, std::ios::end);
        contents.resize(in.tellg());
        in.seekg(0, std::ios::beg);
        in.read(&contents[0], contents.size());
        in.close();

    }
    else{
        printf("File not found: %s\n",filepath.c_str());
    }

    if(contents.empty()){
        printf("File is empty: %s\n",file.c_str());
        return 0;
    }
    
    const char* source = contents.c_str();
    shader = glCreateShader(type);
    //I need to redo that part.
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
#if defined(DEBUG)
    GLint logLength;
    
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);
    GLGetError();
    if (logLength > 1)
    {
        GLchar *log = new GLchar[logLength];
        glGetShaderInfoLog(shader, logLength, &logLength, log);
        
        printf("Shader compilation failed with error:\n%s", log);
        delete[] log;
    }
#endif
    GLint status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    GLGetError();
    if (0 == status)
    {
        glDeleteShader(shader);
        GLGetError();
        printf("Shader compilation failed for file %s\n",file.c_str());
        
    }
    
    return shader;
}
void ShaderLoader::linkProgram(GLuint program){
    glLinkProgram(program);
    GLGetError();
    
#if defined(DEBUG)
    GLint logLength;
    
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);
    GLGetError();
    if (logLength > 1)
    {
        GLchar *log = (GLchar*)malloc((size_t)logLength);
        glGetProgramInfoLog(program, logLength, &logLength, log);
        GLGetError();
        printf("Shader program linking failed with error:\n%s", log);
        free(log);
    }
#endif
    
    GLint status;
    glGetProgramiv(program, GL_LINK_STATUS, &status);
    if (0 == status)
    {
        printf("Failed to link shader program");
        
    }
    
}
