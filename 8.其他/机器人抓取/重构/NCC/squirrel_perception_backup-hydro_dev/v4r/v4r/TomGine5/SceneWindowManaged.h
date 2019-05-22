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
#ifndef _TG_SCENE_WINDOW_MANAGED_
#define _TG_SCENE_WINDOW_MANAGED_

#include "SceneWindow.h"
#include <boost/thread.hpp>
#include <list>

namespace tg{

class SceneWindowManaged : public Scene{
private:
    static boost::atomic<bool> running;
    static void managerThreadProc();
    static boost::thread* managerThread;
    static std::list<SceneWindowManaged*> sceneList;

    static std::list<SceneWindowManaged*> sceneListToAdd;
    static boost::mutex staticMutex;

    static std::list<SceneWindowManaged*> sceneListToRemove;



    std::list<SceneObject*> objectsToAdd;
    std::list<SceneObject*> objectsToRemove;
    boost::mutex objectMutex;


    void initInThread();
    void removeInThread();
    //managed SceneWindow
    SceneWindow* window;

    /*
     * Variables stored to create the window inside the thread
     */
    int width;
    int height;
    std::string title;
    GLWindow* share;

    bool done;
    boost::condition_variable doneConditionVariable;
    boost::mutex m;
    SceneCam* sceneCam;
public:
    SceneWindowManaged(GLuint width=800,GLuint height=600,std::string title=std::string("SceneWindow (Managed)"),GLWindow* share=0);
    ~SceneWindowManaged();


    void setCam(SceneCam* cam);//call base class set cam + call set cam of containing image
    void addSceneObject(SceneObject* object);
    void removeSceneObject(SceneObject* object);
    //void keyCallback(int key,int scancode, int action, int mods);
    void draw();






};
}

#endif
