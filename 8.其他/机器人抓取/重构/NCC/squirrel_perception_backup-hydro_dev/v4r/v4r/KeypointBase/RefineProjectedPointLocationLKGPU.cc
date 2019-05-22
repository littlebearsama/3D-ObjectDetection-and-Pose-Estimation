/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Simon Schreiberhuber (schreiberhuber@acin.tuwien.ac.at)
 */

#include "GL/glew.h"
#include "RefineProjectedPointLocationLKGPU.hh"
#include "v4r/KeypointTools/invPose.hpp"
#include "v4r/KeypointTools/projectPointToImage.hpp"
#include "v4r/KeypointTools/warpPatchHomography.hpp"
#include "v4r/KeypointTools/Vector.hpp"
#include <opencv2/highgui/highgui.hpp>

//#include "v4r/TomGine5/ShaderLoader.h"
//#include <pcl/common/time.h>
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <stdio.h>

#include <omp.h>


namespace kp 
{

using namespace std;


/********************** RefineProjectedPointLocationLK ************************
 * Constructor/Destructor
 */
RefineProjectedPointLocationLKGPU::RefineProjectedPointLocationLKGPU(const kp::RefineProjectedPointLocationLK::Parameter &p)
 : param(p)
{
    srcTexture=0;
    tgtTexture=0;

    /*
     * More than this amount of patches can never be processed. Any higher number x must be of y^2: y element of N.
     */
    maxPatchCount=1024;


    if (p.patch_size.width!=p.patch_size.height){
        fprintf(stderr,"[RefineProjectedLocationLKGPU::constructor] The patch size is not square. They have to be square\n");

    }
    patchSize=p.patch_size.width;
    patchSize=16;
    if (fmod(log2(patchSize),1.0)){
        fprintf(stderr,"[RefineProjectedLocationLKGPU::constructor] The patch size must be of 2^x it is now set to 16\n");
        patchSize=16;
    }
    patchSize=16;
    pyrDepth=1;
    int res=sqrt(maxPatchCount)*patchSize;

    /*
     * Create an OpenGL context
     */
    context = new tg::GLContext();
    context->makeCurrent();


    /*
     * Create textures:
     * The patches around the keypoints are stored here:
     */
    srcITexture = new tg::GLTexture2D(res,res,CV_8UC1);
    srcITexture->setFilter(GL_NEAREST,GL_NEAREST_MIPMAP_NEAREST);
    tgtITexture = new tg::GLTexture2D(res,res,CV_8UC1);
    tgtITexture->setFilter(GL_NEAREST,GL_NEAREST_MIPMAP_NEAREST);

    /*
     * texture for derivatives and such
     */
    gxxxyyyTexture = new tg::GLTexture2D(res,res,CV_32FC3);
    gxxxyyyTexture->setFilter(GL_NEAREST,GL_NEAREST_MIPMAP_NEAREST);
    errxyTexture = new tg::GLTexture2D(res,res,CV_32FC2);
    errxyTexture->setFilter(GL_NEAREST,GL_NEAREST_MIPMAP_NEAREST);
    s1s2distTexture = new tg::GLTexture2D(res,res,CV_32FC3);//last texture needed for ncc calculation



    /*
     * For writing to the according textures:
     * create framebuffers:
     */
    glGenFramebuffers(1,&homographicReadFBO);
    glBindFramebuffer(GL_FRAMEBUFFER,homographicReadFBO);
    srcITexture->bind();
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glFramebufferTexture(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,srcITexture->getHandle(),0);

    glGenFramebuffers(1,&tgtReadFBO);
    glBindFramebuffer(GL_FRAMEBUFFER,tgtReadFBO);
    tgtITexture->bind();
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glFramebufferTexture(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,tgtITexture->getHandle(),0);

    /*
     * gxy gxx gyy err.x err.y are written by the same shader and stored in 2 different textures
     */
    glGenFramebuffers(1,&gxyerrFBO);
    glBindFramebuffer(GL_FRAMEBUFFER,gxyerrFBO);
    gxxxyyyTexture->bind();
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glFramebufferTexture(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,gxxxyyyTexture->getHandle(),0);
    errxyTexture->bind();
    glDrawBuffer(GL_COLOR_ATTACHMENT1);
    glFramebufferTexture(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT1,errxyTexture->getHandle(),0);

    glGenFramebuffers(1,&nccFBO);
    glBindFramebuffer(GL_FRAMEBUFFER,nccFBO);
    s1s2distTexture->bind();
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glFramebufferTexture(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,s1s2distTexture->getHandle(),0);

    glBindFramebuffer(GL_FRAMEBUFFER,0);


    /*
     * create a quad that is basically our canvas
     */
    glm::vec2 quadData[]={glm::vec2(0,0),glm::vec2(0,1),glm::vec2(1,0),glm::vec2(0,1),glm::vec2(1,1),glm::vec2(1,0),};
    glGenBuffers(1,&quadVBO);
    glBindBuffer(GL_ARRAY_BUFFER,quadVBO);
    glBufferData(GL_ARRAY_BUFFER,sizeof(quadData)*sizeof(glm::vec2),quadData,GL_STATIC_DRAW);

    /*
     * Create indices. Trough instancing our quads will be repeated to display our patches.
     */
    GLushort* indexData= new GLushort[maxPatchCount];
    for (unsigned int i=0;i<maxPatchCount;i++){
        indexData[i]=i;
    }
    glGenBuffers(1,&readIndexVBO);
    glBindBuffer(GL_ARRAY_BUFFER,readIndexVBO);
    glBufferData(GL_ARRAY_BUFFER,sizeof(GLushort)*maxPatchCount,indexData,GL_STATIC_DRAW);
    delete[] indexData;


    /*
     * next to the indices the shader also receives the homographic transformation describing the patch
     * position and it's orientation
     */
    homographicReadMatData =new glm::mat3[maxPatchCount];

    glGenBuffers(1,&homographicReadMatVBO);
    glBindBuffer(GL_ARRAY_BUFFER,homographicReadMatVBO);
    glBufferData(GL_ARRAY_BUFFER,sizeof(glm::mat3)*maxPatchCount,0,GL_DYNAMIC_DRAW);

    /*
     * For the target image there is no homography needed. Therefore we only need the point to create our
     * patches. x and y are the coordinates while z describes the convergence
     */
    tgtReadPointData= new glm::vec3[maxPatchCount];
    glGenBuffers(2,positionTFVBOs);
    glBindBuffer(GL_ARRAY_BUFFER,positionTFVBOs[0]);
    glBufferData(GL_ARRAY_BUFFER,sizeof(glm::vec3)*maxPatchCount,0,GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER,positionTFVBOs[1]);
    glBufferData(GL_ARRAY_BUFFER,sizeof(glm::vec3)*maxPatchCount,0,GL_DYNAMIC_DRAW);

    /*
     * Let the shader loading begin
     */
    glGenVertexArrays(1, &homographicReadVAO);
    glBindVertexArray(homographicReadVAO);
    homographicReadProgram.compileShader(std::string(KEYPOINT_BASE_SHADER) + "hread.fsh");
    homographicReadProgram.bindFragDataLocation(0,"fragColor");

    homographicReadProgram.compileShader(std::string(KEYPOINT_BASE_SHADER) + "hread.vsh");

    homographicReadProgram.link();

    homographicReadProgram.bindAttribLocation(0,"postion");
    glBindBuffer(GL_ARRAY_BUFFER,quadVBO);
    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,0,NULL);
    glEnableVertexAttribArray(0);
    glVertexAttribDivisor(0,0);

    homographicReadProgram.bindAttribLocation(1,"index");
    glBindBuffer(GL_ARRAY_BUFFER,readIndexVBO);
    glVertexAttribIPointer(1,1,GL_UNSIGNED_SHORT,0,NULL);
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1,1);

    /*
     * because attributes can only have 4 float values we have to split the homography matrix
     * into 3 vectors:
     */
    homographicReadProgram.bindAttribLocation(2,"homography1");
    glBindBuffer(GL_ARRAY_BUFFER,homographicReadMatVBO);
    glVertexAttribPointer(2,3,GL_FLOAT,GL_FALSE,sizeof(glm::mat3),(GLvoid*)(sizeof(glm::vec3)*0));
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2,1);

    homographicReadProgram.bindAttribLocation(3,"homography2");
    glVertexAttribPointer(3,3,GL_FLOAT,GL_FALSE,sizeof(glm::mat3),(GLvoid*)(sizeof(glm::vec3)*1));
    glEnableVertexAttribArray(3);
    glVertexAttribDivisor(3,1);

    homographicReadProgram.bindAttribLocation(4,"homography3");
    glVertexAttribPointer(4,3,GL_FLOAT,GL_FALSE,sizeof(glm::mat3),(GLvoid*)(sizeof(glm::vec3)*2));
    glEnableVertexAttribArray(4);
    glVertexAttribDivisor(4,1);

    homographicReadProgram.use();

    homographicReadProgram.setUniform("sqrt_maxPatchCount",(GLint)sqrt(maxPatchCount));
    homographicReadProgram.setUniform("intensityTexture",0);
    homographicReadProgram.setUniform("pyrLvl",(GLint)0);
    homographicReadProgram.setUniform("patchSize",(GLint)patchSize);

    glBindVertexArray(0);

    /*
     * Because we double buffered the positions of the patches in the tgt image
     * we have to make 2 VAOs
     */
    glGenVertexArrays(2, tgtReadVAOs);
    tgtReadProgram.compileShader(std::string(KEYPOINT_BASE_SHADER) + "tgtread.fsh");
    tgtReadProgram.bindFragDataLocation(0,"fragColor");
    tgtReadProgram.compileShader(std::string(KEYPOINT_BASE_SHADER) + "tgtread.vsh");
    tgtReadProgram.link();

    //VAO1:
    glBindVertexArray(tgtReadVAOs[0]);
    tgtReadProgram.bindAttribLocation(0,"postion");
    glBindBuffer(GL_ARRAY_BUFFER,quadVBO);
    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,0,NULL);
    glEnableVertexAttribArray(0);
    glVertexAttribDivisor(0,0);

    tgtReadProgram.bindAttribLocation(1,"index");
    glBindBuffer(GL_ARRAY_BUFFER,readIndexVBO);
    glVertexAttribIPointer(1,1,GL_UNSIGNED_SHORT,0,NULL);
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1,1);

    tgtReadProgram.bindAttribLocation(2,"tgtPos");
    glBindBuffer(GL_ARRAY_BUFFER,positionTFVBOs[0]);
    glVertexAttribPointer(2,3,GL_FLOAT,GL_FALSE,sizeof(glm::vec3),NULL);
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2,1);


    //VAO2:
    glBindVertexArray(tgtReadVAOs[1]);
    tgtReadProgram.bindAttribLocation(0,"postion");
    glBindBuffer(GL_ARRAY_BUFFER,quadVBO);
    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,0,NULL);
    glEnableVertexAttribArray(0);
    glVertexAttribDivisor(0,0);

    tgtReadProgram.bindAttribLocation(1,"index");
    glBindBuffer(GL_ARRAY_BUFFER,readIndexVBO);
    glVertexAttribIPointer(1,1,GL_UNSIGNED_SHORT,0,NULL);
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1,1);

    tgtReadProgram.bindAttribLocation(2,"tgtPos");
    glBindBuffer(GL_ARRAY_BUFFER,positionTFVBOs[1]);
    glVertexAttribPointer(2,3,GL_FLOAT,GL_FALSE,sizeof(glm::vec3),NULL);
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2,1);


    tgtReadProgram.use();
    tgtReadProgram.setUniform("sqrt_maxPatchCount",(GLint)sqrt(maxPatchCount));
    tgtReadProgram.setUniform("intensityTexture",0);
    tgtReadProgram.setUniform("pyrLvl",(GLint)0);
    tgtReadProgram.setUniform("patchSize",(GLint)patchSize);


    /*
     * Load shader to calculate gxx, gxy, gyy and err.x err.y
     */
    glGenVertexArrays(1, &gxyerrVAO);
    glBindVertexArray(gxyerrVAO);

    gxyerrProgram.compileShader(std::string(KEYPOINT_BASE_SHADER) + "gxyerr.fsh");
    gxyerrProgram.bindFragDataLocation(0,"g");
    gxyerrProgram.bindFragDataLocation(1,"err");
    gxyerrProgram.compileShader(std::string(KEYPOINT_BASE_SHADER) + "gxyerr.vsh");
    gxyerrProgram.link();

    gxyerrPointCountUniform = gxyerrProgram.getUniformLocation("patchCount");
    gxyerrProgram.bindAttribLocation(0,"postion");
    glBindBuffer(GL_ARRAY_BUFFER,quadVBO);
    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,0,NULL);
    glEnableVertexAttribArray(0);
    glVertexAttribDivisor(0,0);

    gxyerrProgram.use();
    gxyerrProgram.setUniform("sqrt_maxPatchCount",(GLint)sqrt(maxPatchCount));
    gxyerrProgram.setUniform("tgtTexture",0);
    gxyerrProgram.setUniform("srcTexture",1);

    /*
     * Shader to calculate the Normalized Cross Correlation
     */
    glGenVertexArrays(1, &nccVAO);
    glBindVertexArray(nccVAO);
    nccProgram.compileShader(std::string(KEYPOINT_BASE_SHADER) + "ncc.fsh");
    nccProgram.bindFragDataLocation(0,"fragColor");
    nccProgram.compileShader(std::string(KEYPOINT_BASE_SHADER) + "ncc.vsh");
    nccProgram.link();

    nccProgram.bindAttribLocation(0,"postion");
    glBindBuffer(GL_ARRAY_BUFFER,quadVBO);
    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,0,NULL);
    glEnableVertexAttribArray(0);
    glVertexAttribDivisor(0,0);

    nccProgram.bindAttribLocation(1,"index");
    glBindBuffer(GL_ARRAY_BUFFER,readIndexVBO);
    glVertexAttribIPointer(1,1,GL_UNSIGNED_SHORT,0,NULL);
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1,1);

    nccProgram.use();

    nccProgram.setUniform("sqrt_maxPatchCount",(GLint)sqrt(maxPatchCount));
    nccProgram.setUniform("tgtTexture",0);
    nccProgram.setUniform("srcTexture",1);
    nccProgram.setUniform("lod",(GLint)log2(patchSize));


    /*
     * Again we double buffered the keypoint positions:
     * The new position of the keypoints get calculated by a transform feedback shader:
     */
    glGenVertexArrays(2, solveTFVAOs);
    solveTFProgram.compileShader(std::string(KEYPOINT_BASE_SHADER) + "solveTF.vsh");

    const GLchar* feedbackVaryings[] = { "transformOut" };
    glTransformFeedbackVaryings(solveTFProgram.getHandle(),1,feedbackVaryings,GL_INTERLEAVED_ATTRIBS);
    tg::GLUtils::checkForOpenGLError("init transform feedback");

    solveTFProgram.link();

    //VAO1
    glBindVertexArray(solveTFVAOs[0]);
    GLuint TFpos=solveTFProgram.getAttribLocation("pos");
    glBindBuffer(GL_ARRAY_BUFFER,positionTFVBOs[0]);
    glVertexAttribPointer(TFpos,3,GL_FLOAT,GL_FALSE,0,NULL);
    glEnableVertexAttribArray(TFpos);

    //solveTFProgram.bindAttribLocation(1,"index");
     TFpos=solveTFProgram.getAttribLocation("index");
    glBindBuffer(GL_ARRAY_BUFFER,readIndexVBO);
    glVertexAttribIPointer(TFpos,1,GL_UNSIGNED_SHORT,0,NULL);
    glEnableVertexAttribArray(TFpos);

    //VAO2
    glBindVertexArray(solveTFVAOs[1]);
    //solveTFProgram.bindAttribLocation(0,"pos");
     TFpos=solveTFProgram.getAttribLocation("pos");
    glBindBuffer(GL_ARRAY_BUFFER,positionTFVBOs[1]);
    glVertexAttribPointer(TFpos,3,GL_FLOAT,GL_FALSE,0,NULL);
    glEnableVertexAttribArray(TFpos);

    //solveTFProgram.bindAttribLocation(1,"index");
    TFpos=solveTFProgram.getAttribLocation("index");
    glBindBuffer(GL_ARRAY_BUFFER,readIndexVBO);
    glVertexAttribIPointer(TFpos,1,GL_UNSIGNED_SHORT,0,NULL);
    glEnableVertexAttribArray(TFpos);



    solveTFProgram.use();

    solveTFProgram.setUniform("gxyTexture",0);
    solveTFProgram.setUniform("errxyTexture",1);
    int lod = log2(patchSize);//needed mipmap level
    solveTFProgram.setUniform("lod",float(lod));
    solveTFProgram.setUniform("pyrLvl",1.0f);
    solveTFProgram.setUniform("sqrt_maxPatchCount",(GLfloat)sqrt(maxPatchCount));
    solveTFProgram.setUniform("minDeterminant",param.min_determinant);
    solveTFProgram.setUniform("minDisplacement",param.min_displacement);
    solveTFProgram.setUniform("stepFactor",param.step_factor);

    glBindVertexArray(0);




}

RefineProjectedPointLocationLKGPU::~RefineProjectedPointLocationLKGPU(){
    context->makeCurrent();
    delete srcITexture;
    delete tgtITexture;
    delete gxxxyyyTexture;
    delete errxyTexture;
    delete s1s2distTexture;
    tg::GLUtils::checkForOpenGLError("[RefineProjectedPointLocationLKGPU::destructor]");

    delete[] homographicReadMatData;
    delete[] tgtReadPointData;
    delete context;

}




/************************** PRIVATE ************************/


/**
 * solve
 * [gxx gxy] [delta.x] = [err.x]
 * [gxy gyy] [delta.y] = [err.y]
 */
bool RefineProjectedPointLocationLKGPU::solve(const cv::Point2f &err, float gxx, float gxy, float gyy, cv::Point2f &delta)
{
  float det = gxx*gyy - gxy*gxy;

  if (det < param.min_determinant)  return false;

  delta.x = (gyy*err.x - gxy*err.y)/det;
  delta.y = (gxx*err.y - gxy*err.x)/det;

  return true;
}


/************************** PUBLIC *************************/

/**
 * trackImagePoints
 * @param converged 1..converged, -1..out_of_bound, -2..small_determinant, -3..large_error
 */
void RefineProjectedPointLocationLKGPU::refineImagePoints(const std::vector<Eigen::Vector3f> &pts, const std::vector<Eigen::Vector3f> &normals, std::vector<cv::Point2f> &im_pts_tgt, std::vector<int> &converged)
{
  //pcl::ScopeTime t1("GPU Tracking");
  if (!tgtTexture && !srcTexture)
    throw std::runtime_error("[RefineProjectedPointLocationLKGPU::refineImagePoints] No data available!");

  cv::Mat_<unsigned char> patch;

  Eigen::Matrix<float,3,3,Eigen::RowMajor> H, T;
  Eigen::Vector3f n, pt3;
  double d;


  delta_pose =  pose_src*inv_pose_tgt;
  delta_R = delta_pose.topLeftCorner<3,3>();
  delta_t = delta_pose.block<3,1>(0,3);

  bool have_dist = !tgt_dist_coeffs.empty();
  im_pts_tgt.resize(pts.size());
  converged.resize(pts.size());



  /*
   * Start:
   */
  //needed mipmap level
  int lod = log2(patchSize);

  unsigned int size=(int)min((int)pts.size(),(int)maxPatchCount);
  if(pts.size()>maxPatchCount){
      fprintf(stderr,"[RefineProjectedPointLocationLKGPU::refineImagePoints] To many image points!!! Only the %d points get threated!\n",maxPatchCount);
  }

  /*
   * First set the convergence to 1(converging) and upload everything to the GPU (and also create the homography)
   */
  for (unsigned i=0; i<size; i++){
      converged[i] = 1;
      pt3 = R_tgt*pts[i] + t_tgt;
      cv::Point2f &pt_im = im_pts_tgt[i];
      if (have_dist)
        kp::projectPointToImage(&pt3[0], tgt_intrinsic.ptr<double>(), tgt_dist_coeffs.ptr<double>(), &pt_im.x);
      else kp::projectPointToImage(&pt3[0], tgt_intrinsic.ptr<double>(), &pt_im.x);

      homographicReadMatData[i]=glm::make_mat3(H.data());

      tgtReadPointData[i]=glm::vec3(pt_im.x,pt_im.y,converged[i]);
  }
  //upload:
  context->makeCurrent();
  glBindBuffer(GL_ARRAY_BUFFER,positionTFVBOs[0]);
  glBufferSubData(GL_ARRAY_BUFFER, 0, size*sizeof(glm::vec3), tgtReadPointData);

  tg::GLUtils::checkForOpenGLError("[RefineProjectedLocationLKGPU::refineImagePoints]");


  /*
   * For every pyramid level:
   */
  int feedbackBuffer=0;
  int res=sqrt(maxPatchCount)*patchSize;
  for (int j=max((int)pyrDepth-1,0);j>=0;j--){

      /*
       * The scale of the patches accoarding to the pyramid level
       */
      float scale=pow(2,j);

      /*
       * Calc homography accoarding to scale or pyramid level
       */
      for (unsigned i=0; i<size; i++){
          T.setIdentity();

          converged[i] = 1;

          pt3 = R_tgt*pts[i] + t_tgt;
          n = R_tgt*normals[i];

          cv::Point2f &pt_im = im_pts_tgt[i];

          if (have_dist)
            kp::projectPointToImage(&pt3[0], tgt_intrinsic.ptr<double>(), tgt_dist_coeffs.ptr<double>(), &pt_im.x);
          else kp::projectPointToImage(&pt3[0], tgt_intrinsic.ptr<double>(), &pt_im.x);

          T(0,2) = pt_im.x - (int)patchSize/2*scale;//patch.cols/2;
          T(1,2) = pt_im.y - (int)patchSize/2*scale;//patch.rows/2;
          d = n.transpose()*pt3;
          H = delta_R + 1./d*delta_t*n.transpose();
          H = src_C * H * tgt_C.inverse() * T;

          homographicReadMatData[i]=glm::make_mat3(H.data());

      }

      //upload:
      glBindBuffer(GL_ARRAY_BUFFER,homographicReadMatVBO);
      glBufferSubData(GL_ARRAY_BUFFER, 0, size*sizeof(glm::mat3), homographicReadMatData);
      tg::GLUtils::checkForOpenGLError("[RefineProjectedLocationLKGPU::refineImagePoints]");


      //printf("pyramid level:%d\n",j);

      /*
       * Homographicly read the source patches:
       */
      glBindVertexArray(homographicReadVAO);
      homographicReadProgram.use();
      homographicReadProgram.setUniform("pyrLvl",j);
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D,srcTexture->getHandle());
      glBindFramebuffer(GL_FRAMEBUFFER,homographicReadFBO);
      glViewport(0,0,res,res);
      glDrawArraysInstanced(GL_TRIANGLES,0,6,size);

      /*
       * Do the same with the tgt Image;
       */
      glBindVertexArray(tgtReadVAOs[0]);
      tgtReadProgram.use();
      tgtReadProgram.setUniform("pyrLvl",j);
      glActiveTexture(GL_TEXTURE0);
      tgtTexture->bind();
      glBindFramebuffer(GL_FRAMEBUFFER,tgtReadFBO);
      glViewport(0,0,res,res);
      glDrawArraysInstanced(GL_TRIANGLES,0,6,size);

      /*
       * unluckily the shader can't read these textures without mipmaps. But basically they are
       * only needed afterwards for the ncc.(where we again have to update them)
       */
      tgtITexture->generateMipmaps();
      srcITexture->generateMipmaps();
      tg::GLUtils::checkForOpenGLError("[RefineProjectedLocationLKGPU::refineImagePoints]");

      int k=0;
      bool alreadyDone;
      feedbackBuffer=0;
      do{
          alreadyDone=false;

          /*
           * calculate gxx gxy gyy errx and erry
           */
          glBindVertexArray(gxyerrVAO);
          gxyerrProgram.use();
          glUniform1i(gxyerrPointCountUniform,size);
          glActiveTexture(GL_TEXTURE0);
          tgtITexture->bind();
          glActiveTexture(GL_TEXTURE1);
          srcITexture->bind();
          glBindFramebuffer(GL_FRAMEBUFFER,gxyerrFBO);
          glViewport(0,0,res,res);
          //glClearColor(0,0.,0,0);
          //glClear(GL_COLOR_BUFFER_BIT);
          GLenum buffers[]={GL_COLOR_ATTACHMENT0,GL_COLOR_ATTACHMENT1};
          glDrawBuffers(2,buffers);
          glDrawArrays(GL_TRIANGLES,0,6);
          tg::GLUtils::checkForOpenGLError("[RefineProjectedLocationLKGPU::refineImagePoints]");



          /*
           * to correct the points we need the mean values of gxx, gxy, gyy and err. therefore we
           * generate the mipmaps
           */
          gxxxyyyTexture->generateMipmaps();
          errxyTexture->generateMipmaps();


          /*
            Do the Transform feedback thingy
            */
          glBindVertexArray(solveTFVAOs[feedbackBuffer]);
          solveTFProgram.use();
          tg::GLUtils::checkForOpenGLError("[RefineProjectedLocationLKGPU::refineImagePoints]");
          solveTFProgram.setUniform("pyrLvl",j);
          tg::GLUtils::checkForOpenGLError("[RefineProjectedLocationLKGPU::refineImagePoints]");
          glActiveTexture(GL_TEXTURE0);
          glBindTexture(GL_TEXTURE_2D,gxxxyyyTexture->getHandle());
          glActiveTexture(GL_TEXTURE1);
          glBindTexture(GL_TEXTURE_2D,errxyTexture->getHandle());
          glBindFramebuffer(GL_FRAMEBUFFER,gxyerrFBO);
          glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, positionTFVBOs[1-feedbackBuffer]);
          glEnable(GL_RASTERIZER_DISCARD);
          glBeginTransformFeedback(GL_POINTS);
          glDrawArrays(GL_POINTS,0,size);
          tg::GLUtils::checkForOpenGLError("[RefineProjectedLocationLKGPU::refineImagePoints]");
          glEndTransformFeedback();
          glDisable(GL_RASTERIZER_DISCARD);

          /*
           * Reread the target patches according to the new positions:
           */
          glBindVertexArray(tgtReadVAOs[1-feedbackBuffer]);
          tgtReadProgram.use();
          glActiveTexture(GL_TEXTURE0);
          tgtTexture->bind();
          glBindFramebuffer(GL_FRAMEBUFFER,tgtReadFBO);
          glViewport(0,0,res,res);
          glDrawArraysInstanced(GL_TRIANGLES,0,6,size);
          tg::GLUtils::checkForOpenGLError("[RefineProjectedLocationLKGPU::refineImagePoints]");
          glBindVertexArray(0);
          feedbackBuffer=1-feedbackBuffer;


          k++;
      }while(k < param.max_iterations && !alreadyDone);
  }

  /*
   * After we are done with calculating all the points we calculate their ncc:
   */
  glm::vec3 feedback[maxPatchCount];
  glBindBuffer(GL_ARRAY_BUFFER,positionTFVBOs[feedbackBuffer]);
  glGetBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(feedback), feedback);

  for (unsigned i=0; i<size; i++){
      cv::Point2f &pt_im = im_pts_tgt[i];
      pt_im= cv::Point2f(feedback[i].x,feedback[i].y);
      converged[i]=feedback[i].z;
  }

  /*
   * The mipmaps are needed for the calculation of the mean value of one patch
   */
  tgtITexture->generateMipmaps();
  srcITexture->generateMipmaps();
  glBindVertexArray(nccVAO);

  nccProgram.use();

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D,tgtITexture->getHandle());
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D,srcITexture->getHandle());

  glBindFramebuffer(GL_FRAMEBUFFER,nccFBO);
  glViewport(0,0,res,res);

  glClearColor(0.99,0,0,0);
  glClear(GL_COLOR_BUFFER_BIT);

  glDrawArraysInstanced(GL_TRIANGLES,0,6,size);
  tg::GLUtils::checkForOpenGLError("[RefineProjectedLocationLKGPU::refineImagePoints]");

  //mipmapping to create average for each patch:
  s1s2distTexture->generateMipmaps();

  //Download the data:
  cv::Mat s1s2dist = s1s2distTexture->getData(lod);

  //test the results:
  for (unsigned i=0; i<size; i++){

      /*
       * calculate the NCC
       */
      float ncc;
      glm::vec3 vs1s2dist= ((glm::vec3*)s1s2dist.data)[i];
      float dist = vs1s2dist.z;
      float s1 = vs1s2dist.x;
      float s2 = vs1s2dist.y;
      s1=s1*(float)(patchSize*patchSize)/((float)patchSize*patchSize - 1.0);
      s2=s2*(float)(patchSize*patchSize)/((float)patchSize*patchSize - 1.0);
      ncc = dist/(sqrt(s1*s2));

      //test for convergence:
      if (1.-ncc > param.ncc_residual)
        converged[i] = -3;
  }


}

/**
 * setSourceImage
 */
void RefineProjectedPointLocationLKGPU::setSourceImage(const cv::Mat_<unsigned char> &_im_src, const Eigen::Matrix4f &_pose_src)
{
  pose_src = _pose_src;

  /*
   * Load source image, and if a pyramid is desired create the accoarding pyramid:
   */
  context->makeCurrent();
  if (!srcTexture){
      srcTexture = new tg::GLTexture2D(_im_src);
      if (pyrDepth>1)
          srcPyramid = new tg::Pyramid(srcTexture,pyrDepth);

  }else{
      srcTexture->updateTexture(_im_src);
      if (pyrDepth>1)
          srcPyramid->update();
  }
  //srcTexture->imshowTexture("srcTexture",1);

}

/**
 * setTargetImage
 */
void RefineProjectedPointLocationLKGPU::setTargetImage(const cv::Mat_<unsigned char> &_im_tgt, const Eigen::Matrix4f &_pose_tgt)
{

  pose_tgt = _pose_tgt;
  kp::invPose(pose_tgt, inv_pose_tgt);

  R_tgt = pose_tgt.topLeftCorner<3,3>();
  t_tgt = pose_tgt.block<3,1>(0,3);


  /*
   * For the Transform feedback shader it is actually necessary to know the target(and therefore source) image resolution:
   */
  context->makeCurrent();
  solveTFProgram.use();
  solveTFProgram.setUniform("tgtImRes",glm::vec2(_im_tgt.cols,_im_tgt.rows));

  /*
   * Otherwise it is just the typical upload image and create a pyramid if desired.
   */
  if (!tgtTexture){
      tgtTexture = new tg::GLTexture2D(_im_tgt);
      if (pyrDepth>1)
        tgtPyramid = new tg::Pyramid(tgtTexture,pyrDepth);
  }else{
      tgtTexture->updateTexture(_im_tgt);
      if (pyrDepth>1)
          tgtPyramid->update();
  }


  tg::GLUtils::checkForOpenGLError("[RefineProjectedLocationLKGPU::setTargetImage]");


}


/**
 * setSourceCameraParameter
 */
void RefineProjectedPointLocationLKGPU::setSourceCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs)
{
  src_dist_coeffs = cv::Mat_<double>();

  if (_intrinsic.type() != CV_64F)
    _intrinsic.convertTo(src_intrinsic, CV_64F);
  else src_intrinsic = _intrinsic;

  if (!_dist_coeffs.empty())
  {
    src_dist_coeffs = cv::Mat_<double>::zeros(1,8);
    for (int i=0; i<_dist_coeffs.cols*_dist_coeffs.rows && i<8; i++)
      src_dist_coeffs(0,i) = _dist_coeffs.at<double>(0,i);
  }

  src_C = Eigen::Matrix3f::Identity();
  src_C(0,0) = src_intrinsic(0,0);
  src_C(1,1) = src_intrinsic(1,1);
  src_C(0,2) = src_intrinsic(0,2);
  src_C(1,2) = src_intrinsic(1,2);
}

/**
 * setTargetCameraParameter
 */
void RefineProjectedPointLocationLKGPU::setTargetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs)
{
  tgt_dist_coeffs = cv::Mat_<double>();

  if (_intrinsic.type() != CV_64F)
    _intrinsic.convertTo(tgt_intrinsic, CV_64F);
  else tgt_intrinsic = _intrinsic;

  if (!_dist_coeffs.empty())
  {
    tgt_dist_coeffs = cv::Mat_<double>::zeros(1,8);
    for (int i=0; i<_dist_coeffs.cols*_dist_coeffs.rows && i<8; i++)
      tgt_dist_coeffs(0,i) = _dist_coeffs.at<double>(0,i);
  }

  tgt_C = Eigen::Matrix3f::Identity();
  tgt_C(0,0) = tgt_intrinsic(0,0);
  tgt_C(1,1) = tgt_intrinsic(1,1);
  tgt_C(0,2) = tgt_intrinsic(0,2);
  tgt_C(1,2) = tgt_intrinsic(1,2);
}


} //-- THE END --
