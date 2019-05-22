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
#include <stdio.h>
#include "SceneWindowManaged.h"
#include <GL/glew.h>

using namespace tg;

#define OpenGLInit( )\
{\
  glewExperimental=GL_TRUE;\
  glewInit();\
  glGetError();\
  }


boost::atomic<bool> SceneWindowManaged::running;
boost::thread* SceneWindowManaged::managerThread = 0;
std::list<SceneWindowManaged*> SceneWindowManaged::sceneList;

std::list<SceneWindowManaged*> SceneWindowManaged::sceneListToAdd;
boost::mutex SceneWindowManaged::staticMutex;

std::list<SceneWindowManaged*> SceneWindowManaged::sceneListToRemove;


void SceneWindowManaged::managerThreadProc()
{
  printf("[SceneWindowManaged::managerThreadProc] started manager thread\n");
  fflush(stdout);



  running = true;
  while(running)
  {

    {
      boost::mutex::scoped_lock l(staticMutex);

      if(!sceneListToAdd.empty()){

        std::list<SceneWindowManaged*>::const_iterator iterator;
        for(iterator=sceneListToAdd.begin();iterator != sceneListToAdd.end();++iterator){
          sceneList.push_back(*iterator);
          (*iterator)->initInThread();
        }
        sceneListToAdd.clear();

      }


      if(!sceneListToRemove.empty()){

        //printf("here i am\n");
        //fflush(stdout);
        std::list<SceneWindowManaged*>::const_iterator iterator;
        for(iterator=sceneListToRemove.begin();iterator != sceneListToRemove.end();++iterator){
          (*iterator)->removeInThread();
          printf("[SceneWindowManaged::managerThreadProc] window to delete: %p\n",(*iterator));
          fflush(stdout);
          sceneList.remove(*iterator);
        }
        sceneListToRemove.clear();

      }
    }//end of scoped iterator

    if(sceneList.empty())
    {
      running=false;
    }
    else
    {
      //do all drawing and user input handling:
      SceneWindow::pollEvents();
      std::list<SceneWindowManaged*>::const_iterator iterator;
      for(iterator=sceneList.begin();iterator != sceneList.end();++iterator)
        (*iterator)->draw();
    }

  } // while(running)

  printf("[SceneWindowManaged::managerThreadProc] end thread\n");
  fflush(stdout);

}

void SceneWindowManaged::initInThread()
{

  window = new SceneWindow(width,height,title,share);
  OpenGLInit();

  //after we are done, we should notify the waiting constructor
  //printf("before unlocking\n");
  //fflush(stdout);
  //this->done.notify_one();
  //printf("after unlocking\n");
  //fflush(stdout);
}

void SceneWindowManaged::removeInThread()
{
  boost::mutex::scoped_lock lock(m);
  delete window;
  printf("[SceneWindowManaged::removeInThread] before unlocking\n");
  fflush(stdout);

  this->done=true;
  this->doneConditionVariable.notify_one();
  printf("[SceneWindowManaged::removeInThread] after unlocking\n");
  fflush(stdout);

}

SceneWindowManaged::SceneWindowManaged(GLuint width, GLuint height, std::string title, GLWindow *share)
{
  this->width=width;
  this->height=height;
  this->title=title;
  this->share=share;

  //boost::unique_lock<boost::mutex> ul(m);


  {
    boost::mutex::scoped_lock l(staticMutex);
    sceneListToAdd.push_back(this);
  }
  if(!managerThread){
    //start thread
    managerThread = new boost::thread(managerThreadProc);
  }
  //printf("before lock\n");
  //fflush(stdout);

  //this->done.wait(ul);

  //printf("after lock\n");
  //fflush(stdout);



}


SceneWindowManaged::~SceneWindowManaged()
{
  boost::mutex::scoped_lock ul(m);
  this->done=false;

  {
    boost::mutex::scoped_lock l(staticMutex);
    sceneListToRemove.push_back(this);
  }
  //printf("delete\n");
  //fflush(stdout);

  //wait until object is removed
  printf("[SceneWindowManaged::~SceneWindowManaged] before lock\n");
  fflush(stdout);

  while(! this->done) this->doneConditionVariable.wait(ul);
  printf("[SceneWindowManaged::~SceneWindowManaged] after lock\n");
  fflush(stdout);

  int nrOpenWindows=sceneList.size() -sceneListToRemove.size()+ sceneListToAdd.size();
  if(nrOpenWindows==0){
    managerThread->join();
    delete managerThread;
    managerThread=0;
  }
}


void SceneWindowManaged::setCam(SceneCam *cam)
{
  sceneCam=cam;
  //this->window->setCam(cam);
}


void SceneWindowManaged::addSceneObject(SceneObject *object)
{
  this->objectMutex.lock();

  this->objectsToAdd.push_back(object);
  //remove this object from other list:
  this->objectsToRemove.remove(object);
  this->objectMutex.unlock();

}


void SceneWindowManaged::removeSceneObject(SceneObject *object)
{

  this->objectMutex.lock();
  this->objectsToRemove.push_back(object);
  //remove this object from other list:
  this->objectsToAdd.remove(object);
  this->objectMutex.unlock();
}


void SceneWindowManaged::draw()
{

  //add objects
  this->window->setCam(sceneCam);
  this->objectMutex.lock();
  if(!this->objectsToAdd.empty()){
    std::list<SceneObject*>::const_iterator iterator;
    for(iterator=objectsToAdd.begin();iterator != objectsToAdd.end();++iterator){
      this->window->addSceneObject(*iterator);
    }

  }
  this->objectsToAdd.clear();
  if(!this->objectsToRemove.empty()){
    std::list<SceneObject*>::const_iterator iterator;
    for(iterator=objectsToRemove.begin();iterator != objectsToRemove.end();++iterator){

      this->window->removeSceneObject(*iterator);
    }
  }
  this->objectsToRemove.clear();
  this->objectMutex.unlock();

  //draw everything
  window->draw();
  window->swapBuffers();

}
