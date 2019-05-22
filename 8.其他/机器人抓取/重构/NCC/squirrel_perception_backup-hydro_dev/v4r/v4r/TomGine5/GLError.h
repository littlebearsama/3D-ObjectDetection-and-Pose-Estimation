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

#ifndef GLContext_GLMasterHeader_h
#define GLContext_GLMasterHeader_h
//#define DEBUG

/* This file features some makros:
 *GLGetError() tests for any occured OpenGL errors
 *GLCheckFramebufferStatus() tests if the framebuffer is set up correctly
 *GLGetShaderInfoLog() returns line and error code for shader issues
 */



#ifdef  DEBUG

#include <stdlib.h>
#include <assert.h>
#include <stdio.h>

#define GLGetError( )\
{\
for ( GLenum Error = glGetError( ); ( GL_NO_ERROR != Error ); Error = glGetError( ) )\
{\
switch ( Error )\
{\
case GL_INVALID_ENUM:      printf( "\n%s\n\n", "GL_INVALID_ENUM"      ); assert( 0 ); break;\
case GL_INVALID_VALUE:     printf( "\n%s\n\n", "GL_INVALID_VALUE"     ); assert( 0 ); break;\
case GL_INVALID_OPERATION: printf( "\n%s\n\n", "GL_INVALID_OPERATION" ); assert( 0 ); break;\
case GL_OUT_OF_MEMORY:     printf( "\n%s\n\n", "GL_OUT_OF_MEMORY"     ); assert( 0 ); break;\
case GL_INVALID_FRAMEBUFFER_OPERATION: printf( "\n%s\n\n", "GL_INVALID_FRAMEBUFFER_OPERATION" ); assert( 0 ); break;\
default:                                                                              break;\
}\
}\
}

#define GLCheckFramebufferStatus( )\
{\
switch ( glCheckFramebufferStatus( GL_FRAMEBUFFER ) )\
{\
case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:         printf( "\n%s\n\n", "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT"         ); assert( 0 ); break;\
case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT: printf( "\n%s\n\n", "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT" ); assert( 0 ); break;\
case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:        printf( "\n%s\n\n", "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER"        ); assert( 0 ); break;\
case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:        printf( "\n%s\n\n", "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER"        ); assert( 0 ); break;\
case GL_FRAMEBUFFER_UNSUPPORTED:                   printf( "\n%s\n\n", "GL_FRAMEBUFFER_UNSUPPORTED"                   ); assert( 0 ); break;\
case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:        printf( "\n%s\n\n", "GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE"        ); assert( 0 ); break;\
case GL_FRAMEBUFFER_UNDEFINED:                     printf( "\n%s\n\n", "GL_FRAMEBUFFER_UNDEFINED"                     ); assert( 0 ); break;\
default:                                                                                                                              break;\
}\
}

#define GLGetShaderInfoLog( Shader, Source )\
{\
GLint   Status, Count;\
GLchar *Error;\
\
glGetShaderiv( Shader, GL_COMPILE_STATUS, &Status );\
\
if ( !Status )\
{\
glGetShaderiv( Shader, GL_INFO_LOG_LENGTH, &Count );\
\
if ( Count > 0 )\
{\
glGetShaderInfoLog( Shader, Count, NULL, ( Error = calloc( 1, Count ) ) );\
\
printf( "%s\n\n%s\n", Source, Error );\
\
free( Error );\
\
assert( 0 );\
}\
}\
}

#else

#define GLGetError( )

#define GLCheckFramebufferStatus( )

#define GLGetShaderInfoLog( Shader, Source )

#endif /*   DEBUG     */

#endif
