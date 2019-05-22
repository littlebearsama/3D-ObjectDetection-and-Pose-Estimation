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
#include "Background.h"
#include "stdio.h"
#include "GLTexture.h"

using namespace tg;


//TODO!!!!!!!!!!! it can't work that way!!!!!!!!!!
//REDO That part. the texture has to be created in context!!!!!!
Background::Background(tg::GLTexture2D *tex)
{
    m_texture = tex;
}


Background::Background()
{

}


void Background::initInContext()
{
    glm::vec2 quadData[]={glm::vec2(0,0),glm::vec2(0,1),glm::vec2(1,1),glm::vec2(1,0)};
    glGenBuffers(1,&m_quadVBO);
    glBindBuffer(GL_ARRAY_BUFFER,m_quadVBO);
    glBufferData(GL_ARRAY_BUFFER,sizeof(quadData)*sizeof(glm::vec2),quadData,GL_STATIC_DRAW);



    glGenVertexArrays(1,&m_VAO);
    glBindVertexArray(m_VAO);

    m_program = new GLSLProgram();

    m_program->compileShader(std::string(TOMGINE_5_SHADER) + "background.fsh");
    m_program->bindFragDataLocation(0,"fragColor");

    m_program->compileShader(std::string(TOMGINE_5_SHADER) + "background.vsh");

    m_program->link();
    m_program->bindAttribLocation(0,"postion");
    glBindBuffer(GL_ARRAY_BUFFER,m_quadVBO);
    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,0,NULL);
    glEnableVertexAttribArray(0);
    glVertexAttribDivisor(0,0);

    m_program->use();
    m_program->setUniform("tex",0);




}


void Background::removedWhileInContext()
{

}


void Background::draw(Scene *scene)
{
    m_program->use();

    glActiveTexture(GL_TEXTURE0);
    m_texture->bind();
    glBindVertexArray(m_VAO);
    glDrawArrays(GL_TRIANGLE_FAN,0,4);
    //printf("where are we?\n");
}



