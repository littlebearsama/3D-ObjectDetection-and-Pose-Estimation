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
#ifndef _SCENE_WINDOW_H_
#define _SCENE_WINDOW_H_
#include "GLWindow.h"
#include "Scene.h"
namespace tg {
class SceneWindow : public GLWindow, public Scene{
private:
public:
    SceneWindow(GLuint width=800,GLuint height=600,std::string title=std::string("SceneWindow"),GLWindow* share=0);
    void draw();
    bool flipYAxis(){return true;}
    //glm::ivec2 getResolution();
    virtual void keyCallback(int key,int scancode,int action,int mods);
    virtual void cursorCallback(double x,double y);
    virtual void mouseButtonCallback(int button, int action, int mod);
};


}
#endif
