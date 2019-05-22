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
#include "tgTexture.h"
#include "tgError.h"
#include <opencv2/highgui/highgui.hpp>
#include <stdexcept>

using namespace TomGine;

// ############################################################################################
// tgTexture1D

tgTexture1D::tgTexture1D(){
	glGenTextures(1, &m_texture_id);
	glBindTexture(GL_TEXTURE_1D, m_texture_id);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameterf(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_1D, GL_GENERATE_MIPMAP, GL_TRUE);
#ifdef DEBUG
	tgCheckError("[tgTexture1D::tgTexture1D()]");
#endif
}

tgTexture1D::~tgTexture1D(){
	if(glIsTexture(m_texture_id))
		glDeleteTextures(1, &m_texture_id);
}

bool tgTexture1D::Load(const void* data, int width, GLenum internal, GLenum format, GLenum type){
	m_width = width;
	m_intFormat = internal;
	glBindTexture(GL_TEXTURE_1D, m_texture_id);
	glTexImage1D(GL_TEXTURE_1D, 0, internal, m_width, 0, format, type, data);
#ifdef DEBUG
	tgCheckError("[tgTexture1D::Load()]");
#endif
	return true;
}

void tgTexture1D::Bind(int stage) const{
	glEnable(GL_TEXTURE_1D);
	glActiveTexture(GL_TEXTURE0 + stage);
	glBindTexture(GL_TEXTURE_1D, m_texture_id);
	glActiveTexture(GL_TEXTURE0);
#ifdef DEBUG
	tgCheckError("[tgTexture1D::Bind()]");
#endif
}


// ############################################################################################
// tgTexture2D

tgTexture2D::tgTexture2D(){
	glGenTextures(1, &m_texture_id);
	glBindTexture(GL_TEXTURE_2D, m_texture_id);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	m_width = 0;
	m_height = 0;
	m_intFormat = GL_RGBA;
	m_tex_env_mode = GL_REPLACE;

//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
#ifdef DEBUG
	tgCheckError("[tgTexture2D::tgTexture2D()]");
#endif
}

tgTexture2D::~tgTexture2D(){
	if(glIsTexture(m_texture_id))
		glDeleteTextures(1, &m_texture_id);
}

bool tgTexture2D::Load(const void* image_data, int width, int height, GLenum internal, GLenum format, GLenum type){
	m_width = width;
	m_height = height;
	m_intFormat = internal;
	glBindTexture(GL_TEXTURE_2D, m_texture_id);
	glTexImage2D(GL_TEXTURE_2D, 0, internal, (int)m_width, (int)m_height, 0, format, type, image_data);
#ifdef DEBUG
	tgCheckError("[tgTexture2D::Load(void*, int, int, GLenum, GLenum, GLenum)]");
#endif
	return true;
}

bool tgTexture2D::Load(const char* filename, bool flip){
//	cv::Mat img = cv::imread(filename);
//	if(img.data==NULL){
//		std::string errmsg = std::string("[tgTexture2D::Load(const char*)] Error loading file '") + filename + "'.";
//		throw std::runtime_error(errmsg.c_str());
//	}
//	return Load((unsigned char*)img.data, img.cols, img.rows, 3, GL_BGR, GL_UNSIGNED_BYTE);

	IplImage* img = cvLoadImage(filename, CV_LOAD_IMAGE_COLOR);

  if(img==NULL)
    return false;

  if(flip)
    cvFlip(img, img);

	return Load((unsigned char*)img->imageData, img->width, img->height, GL_RGBA, GL_BGR, GL_UNSIGNED_BYTE);
}

bool tgTexture2D::Save(const char* filename){
//	Bind();
//	cv::Mat img( m_height, m_width, CV_8UC3 );
//	glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, img.data);
//
//	if(cv::imwrite(filename, img))
//		return true;
//	else
//		return false;
	Bind();
	IplImage* img = cvCreateImage ( cvSize ( m_width, m_height ), IPL_DEPTH_8U, 3 );
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, img->imageData);
	cvConvertImage(img, img, CV_CVTIMG_SWAP_RB | CV_CVTIMG_FLIP);
	cvSaveImage(filename, img);
	cvReleaseImage(&img);
	return true;
}

bool tgTexture2D::GetImageData(unsigned char* image_data, GLenum format){
	Bind();
	glGetTexImage(GL_TEXTURE_2D, 0, format, GL_UNSIGNED_BYTE, image_data);
	return true;
}

void tgTexture2D::Bind(int stage) const{
  glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, m_tex_env_mode);
	glEnable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0 + stage);
	glBindTexture(GL_TEXTURE_2D, m_texture_id);
	glActiveTexture(GL_TEXTURE0);
}

void tgTexture2D::CopyTexImage2D(int width, int height, GLenum internal){
	m_width = width;
	m_height = height;
	m_intFormat = internal;
	Bind();
	glCopyTexImage2D(GL_TEXTURE_2D, 0, m_intFormat, 0, 0, m_width, m_height, 0);
}

void tgTexture2D::CopyTexImage2D(int x, int y, int w, int h, GLenum internal){
	m_width = w;
	m_height = h;
	m_intFormat = internal;
	Bind();
	glCopyTexImage2D(GL_TEXTURE_2D, 0, m_intFormat, x, y, m_width, m_height, 0);
}

// ############################################################################################
// tgTexture3D

tgTexture3D::tgTexture3D(){
	glGenTextures(1, &m_texture_id);
	glBindTexture(GL_TEXTURE_3D, m_texture_id);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_GENERATE_MIPMAP, GL_TRUE);
#ifdef DEBUG
	tgCheckError("[tgTexture3D::tgTexture3D()]");
#endif
}

tgTexture3D::~tgTexture3D(){
	if(glIsTexture(m_texture_id))
		glDeleteTextures(1, &m_texture_id);
}

bool tgTexture3D::Load(const void* data, int width, int height, int depth, GLenum internal, GLenum format, GLenum type){
	m_width = width;
	m_height = height;
	m_depth = depth;
	m_intFormat = internal;
	glBindTexture(GL_TEXTURE_3D, m_texture_id);
	glTexImage3D(GL_TEXTURE_3D, 0, internal, m_width, m_height, m_depth, 0, format, type, data);
#ifdef DEBUG
	tgCheckError("[tgTexture3D::Load()]");
#endif
	return true;
}

void tgTexture3D::Bind(int stage) const{
	glEnable(GL_TEXTURE_3D);
	glActiveTexture(GL_TEXTURE0 + stage);
	glBindTexture(GL_TEXTURE_3D, m_texture_id);
	glActiveTexture(GL_TEXTURE0);
#ifdef DEBUG
	tgCheckError("[tgTexture3D::Bind()]");
#endif
}

