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

#ifndef _BACKGROUND_H_
#define _BACKGROUND_H_
#include "Scene.h"
#include "GLSLProgram.h"
namespace tg{

//TODO!!!!!!!!!!! it can't work that way!!!!!!!!!!
//REDO That part. the texture has to be created in context!!!!!!
/**
 * @brief The Background class renders the image content given by tex directly to the furthest
 * position of the scene. therefore it should get rendered behind every other object.
 * Don't delete the texture while the Background object is still active or even rendering.
 * The background object is never deleting the texture do it yourself.
 * Note that the texture get's stretched to meet the window borders.
 */
class Background : public SceneObject{
private:
    GLTexture2D* m_texture;
    GLSLProgram* m_program;
    GLuint m_VAO;
    GLuint m_quadVBO;
public:

    /**
     * @brief Background
     * @param tex texture to get rendered to the background
     */
    Background(GLTexture2D* tex);
    Background();

    /**
     * @brief initInContext Outsources the initializing OpenGL calls from the constructor.
     *        This allowes for rendering the object in another thread than the constructor was called.
     *        Gets called by Scene.
     */
    void initInContext();

    /**
     * @brief removedWhileInContext same as init in context.
     */
    void removedWhileInContext();

    /**
     * @brief draw: do rendering
     * @param scene
     */
    void draw(tg::Scene* scene);
};
}
#endif
