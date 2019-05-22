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
#include "Mesh.h"
#include <stdio.h>
#include <iostream>
#include <vector>


using namespace tg;


Mesh::Mesh(std::string filePath){

  m_scene = m_importer.ReadFile(filePath,0);

  if(!m_scene){
    fprintf(stderr,"[Mesh::constructor] %s",m_importer.GetErrorString());
    return;
  }

  if(m_scene->HasAnimations()){
//    printf("[Mesh::Mesh] wooohooo!!! animatiions\n");
  }
  if(m_scene->HasCameras()){
//    printf("[Mesh::Mesh] wohoooo! cameras\n");
  }
  if(m_scene->HasLights()){
//    printf("[Mesh::Mesh] lights!!\n");
  }
  if(m_scene->HasMaterials()){
//    printf("[Mesh::Mesh] materials:%d!\n",m_scene->mNumMaterials);
    for(int i=0;i<(int)m_scene->mNumMaterials;i++){
      aiMaterial* material = m_scene->mMaterials[i];

      aiString name;
      material->Get(AI_MATKEY_NAME,name);

      //material->

//      printf("[Mesh::Mesh] material%d name:%s\n",i,name.C_Str());
    }
  }
  if(m_scene->HasMeshes()){
//    printf("meshes:%d!!\n",m_scene->mNumMeshes);
    for(int i=0;i<(int)m_scene->mNumMeshes;i++)
    {
      aiMesh* mesh = m_scene->mMeshes[i];
      printf("[Mesh::Mesh] faces %d, vertices %d, bones %d, animMeshes %d \n",mesh->mNumFaces,mesh->mNumVertices,mesh->mNumBones,mesh->mNumAnimMeshes);
    }

  }
  if(m_scene->HasTextures())
  {
//    printf("[Mesh::Mesh] textures!!!\n");
  }
}

Mesh::~Mesh(){

}

void Mesh::initInContext(Scene *scene)
{
  m_shader = scene->GetShaderDiffuse();
  m_shader->use();

  glGenVertexArrays(1,&m_VAO);
  glBindVertexArray(m_VAO);

  // init mesh data
  if(m_scene->HasMeshes()){
    if(m_scene->mNumMeshes==1){
      aiMesh* mesh = m_scene->mMeshes[0];
      if(mesh->HasPositions())
      {
        glGenBuffers(1,&m_posVBO);
        glBindBuffer(GL_ARRAY_BUFFER,m_posVBO);
        glBufferData(GL_ARRAY_BUFFER,sizeof(glm::vec3)*mesh->mNumVertices,mesh->mVertices,GL_STATIC_DRAW);
        m_center=glm::vec3(0);
        for(unsigned int i=0;i<mesh->mNumVertices;i++)
          m_center += glm::vec3(mesh->mVertices[i].x,mesh->mVertices[i].y,mesh->mVertices[i].z);
        m_center =m_center/(float)mesh->mNumVertices;
        tg::GLUtils::checkForOpenGLError("[Mesh::initInContext] positions");
      }
      if(mesh->HasNormals())
      {
        glGenBuffers(1,&m_normalVBO);
        glBindBuffer(GL_ARRAY_BUFFER,m_normalVBO);
        glBufferData(GL_ARRAY_BUFFER,sizeof(glm::vec3)*mesh->mNumVertices,mesh->mNormals,GL_STATIC_DRAW);
        tg::GLUtils::checkForOpenGLError("[Mesh::initInContext] normals");
      }
      if(mesh->HasVertexColors(0))
      {
        glGenBuffers(1,&m_colorVBO);
        glBindBuffer(GL_ARRAY_BUFFER,m_colorVBO);
        glBufferData(GL_ARRAY_BUFFER,sizeof(glm::vec4)*mesh->mNumVertices,mesh->mColors[0],GL_STATIC_DRAW);
        tg::GLUtils::checkForOpenGLError("[Mesh::initInContext] colors");
      }
      else
      {
        std::vector<glm::vec4> color(mesh->mNumVertices, glm::vec4(1.0));
        glGenBuffers(1,&m_colorVBO);
        glBindBuffer(GL_ARRAY_BUFFER,m_colorVBO);
        glBufferData(GL_ARRAY_BUFFER,sizeof(glm::vec4)*mesh->mNumVertices,&color[0],GL_STATIC_DRAW);
        tg::GLUtils::checkForOpenGLError("[Mesh::initInContext] colors");
      }
      if(mesh->HasFaces())
      {
        glGenBuffers(1,&m_IBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_IBO);

        std::vector<GLuint> indexBufferData;
        m_faceCount=0;
        for(unsigned int i=0;i<mesh->mNumFaces;i++)
        {
          const aiFace& face = mesh->mFaces[i];
          if(face.mNumIndices==3)
          {
            indexBufferData.push_back(face.mIndices[0]);
            indexBufferData.push_back(face.mIndices[1]);
            indexBufferData.push_back(face.mIndices[2]);
            m_faceCount++;
            //printf("%d,%d,%d\n",indexBufferData[i*3+0],indexBufferData[i*3+1],indexBufferData[i*3+2]);
          }
        }
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,sizeof(GLuint)*indexBufferData.size(),&indexBufferData[0],GL_STATIC_DRAW);
        tg::GLUtils::checkForOpenGLError("[Mesh::initInContext] colors");
      }

    }else{
      fprintf(stderr,"[Mesh::initInContext] More than one mesh is not supportet (yet)! \n");
      return;
    }
  }

  GLuint posLoc =   m_shader->getAttribLocation("VertexPosition");
  GLuint normLoc =  m_shader->getAttribLocation("VertexNormal");
  GLuint colorLoc = m_shader->getAttribLocation("ColorDiffuse");

  glBindBuffer(GL_ARRAY_BUFFER,m_posVBO);
  glEnableVertexAttribArray(posLoc);
  glVertexAttribPointer(posLoc,3,GL_FLOAT,GL_FALSE,0,NULL);

  glBindBuffer(GL_ARRAY_BUFFER,m_normalVBO);
  glEnableVertexAttribArray(normLoc);
  glVertexAttribPointer(normLoc,3,GL_FLOAT,GL_FALSE,0,NULL);

  glBindBuffer(GL_ARRAY_BUFFER,m_colorVBO);
  glEnableVertexAttribArray(colorLoc);
  glVertexAttribPointer(colorLoc,4,GL_FLOAT,GL_FALSE,0,NULL);

  glBindVertexArray(0);
}

void Mesh::removedWhileInContext()
{

}

void Mesh::draw(Scene *scene)
{
  const glm::mat4& cam_modelview = scene->getCam()->getModelView();
  const glm::mat4& cam_projection = scene->getCam()->getProjectionMatrix();

  poseMutex.lock();
  glm::mat4 modelview = cam_modelview * m_pose;
  poseMutex.unlock();

  glm::mat3 normalmatrix(modelview);
  glm::mat4 MVP = cam_projection * modelview;

  m_shader->use();
  m_shader->setUniform("ModelViewMatrix", modelview);
  m_shader->setUniform("NormalMatrix", normalmatrix);
  m_shader->setUniform("ProjectionMatrix", cam_projection);
  m_shader->setUniform("MVP", MVP);

  glm::vec4 lightpos = glm::vec4(0.0,0.0,0.0,1.0);  // light-source attached to camera
//  glm::vec4 lightpos = cam_modelview * glm::vec4(0.0,0.0,0.0,1.0);  // light-source in world coordinates
  glm::vec3 lightdiff(1.0,1.0,1.0);
  m_shader->setUniform("LightPosition", lightpos);
  m_shader->setUniform("LightDiffuse", lightdiff);

  glBindVertexArray(m_VAO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_IBO);

  //now comes the indexed drawing:
  glDrawElements(GL_TRIANGLES,3*m_faceCount,GL_UNSIGNED_INT,0);
}

glm::vec3 Mesh::getCenter()
{
  return m_center;
}

void Mesh::setPose(const glm::mat4& pose)
{
  poseMutex.lock();
  m_pose = pose;
  poseMutex.unlock();
}
