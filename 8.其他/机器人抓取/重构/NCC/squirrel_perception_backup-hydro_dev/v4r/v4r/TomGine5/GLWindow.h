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
#ifndef _TG_GL_WINDOW_H_
#define _TG_GL_WINDOW_H_


#include <iostream>
#include <list>

#include <GL/glew.h>
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>


struct GLFWwindow;
namespace tg{

/*namespace KEY{
enum key{
    A = XK_a,
    B = XK_b,
    C = XK_c,
    D = XK_d,
    E = XK_e,
    F = XK_f,
    G = XK_g,
    H = XK_h,
    I = XK_i,
    J = XK_j,
    K = XK_k,
    L = XK_l,
    M = XK_m,
    N = XK_n,
    O = XK_o,
    P = XK_p,
    Q = XK_q,
    R = XK_r,
    S = XK_s,
    t = XK_t,
    U = XK_u,
    V = XK_v,
    W = XK_w,
    X = XK_x,
    Y = XK_y,
    Z = XK_z,

    T_0 = XK_0,
    T_1 = XK_1,
    T_2 = XK_2,
    T_3 = XK_3,
    T_4 = XK_4,
    T_5 = XK_5,
    T_6 = XK_6,
    T_7 = XK_7,
    T_8 = XK_8,
    T_9 = XK_9,

    F_1 = XK_F1,
    F_2 = XK_F2,
    F_3 = XK_F3,
    F_4 = XK_F4,
    F_5 = XK_F5,
    F_6 = XK_F6,
    F_7 = XK_F7,
    F_8 = XK_F8,
    F_9 = XK_F9,
    F_10 = XK_F10,
    F_11 = XK_F11,
    F_12 = XK_F12,
    Shift_L = XK_Shift_L,
    Shift_R = XK_Shift_R,
    Alt_L = XK_Alt_L,
    Alt_R = XK_Alt_R,
    Control_L = XK_Control_L,
    Control_R = XK_Control_R,
    Caps_Lock = XK_Caps_Lock,
    Return = XK_Return


};
}
namespace ACTION{
enum action{
    pressed,
    released
};
}
namespace MOUSE{
enum button{
    BUTTON_0 = 0,
    BUTTON_1 = 1,
    BUTTON_2 = 2,
    BUTTON_3 = 3,
    BUTTON_4 = 4,
    BUTTON_5 = 5,
    BUTTON_6 = 6,
    BUTTON_7 = 7,
    BUTTON_8 = 8,
    BUTTON_9 = 9,
};
}
*/
class GLWindow;
/** @brief: A simple class, not unlike the GLFW librarie, to handle X11 windows and input. It additionally handles OpenGL contexts.*/
class GLWindow{
protected:
    //parameters shared throughout the whole program:

    static bool glfwRunning;

    static void errorCallback(int error, const char* description);

    static std::list<GLWindow*> windows;
    GLFWwindow* window;

   //process event:

   //void keyEvent();
   //void mouseButtonEvent();
   //void mouseCursorEvent();

    //this is a callback method
    void (*keyCallbackPointer)(GLWindow* window, int key, int scancode, int action, int mods);
    void (*mouseButtonCallbackPointer)(GLWindow* window, int button,int action, int mod);
    void (*cursorCallbackPointer)(GLWindow* window,double x,double y);


    static void keyCallback(GLFWwindow* window,int key,int scancode,int action,int mods);
    static void mouseButtonCallback(GLFWwindow* window,int button,int action, int mod);
    static void cursorCallback(GLFWwindow* window,double x,double y);


public:




   /**
    * @brief create a window and the according context
    * @param width/height: they should be self explaning
    * @param title: self explaining
    * @param share: share the OpenGL context with an other window or context
    */
    GLWindow(GLuint width=800,GLuint height=600,std::string title=std::string("GLWindow"),GLWindow* share=0);


    /**
     * @brief removes the window and destroys the context
     */
    virtual ~GLWindow();

    /**
     * @brief swapBuffers: Swap front with back buffer (show rendered stuff)
     */
    void swapBuffers();

    /**
     * @brief makeContextCurrent: Sets the OpenGL context and the framebuffer active.
     */
    void makeContextCurrent();





    /**
     * @brief getFramebufferSize: returns the framebuffers size.
     * @param width
     * @param height
     */
    void getFramebufferSize(int *width,int* height);

    /**
     * @brief getShouldClose: As soon as the user clicks the x on the window, the shouldClose flag gets set.
     *        the program itself decides if it really want's to close the window or not. (call the destructor)
     * @return
     */
    bool getShouldClose();

    /**
     * @brief setShouldClose: see getShouldClose
     * @param b
     */
    void setShouldClose(int b);


    /**
     * @brief keyCallback: Callback method, that get's called at every keystroke (or release)
     *          This method is used to forward the event to the registered callback function.
     * @param key: Tells wich key is pressed e.g. KEY::A or KEY::Return
     * @param action: Tells if the key got pressed or released ACTION::pressed/ACTION::released
     * @param mods: not used yet. but could be used for testing if Shift Alt or somthing similar is pressed
     */
    //virtual void keyCallback(KEY::key key,ACTION::action action,int mods);

    /**
     * @brief cursorCallback: Callback method, that get's called at every mouse movement inside the window
     *          This method is used to forward the event to the registered callback function.
     * @param pos: The current cursor position
     * @param mods: not used yet. but could be used for testing if Shift Alt or somthing similar is pressed
     */
    //virtual void cursorCallback(glm::vec2 pos,int mods);

    /**
     * @brief mouseButtonCallback: Callback method, that get's called at every mouse button event
     *          This method is used to forward the event to the registered callback function.
     * @param button: Mouse button 0 to 7 e.g.  MOUSE::BUTTON_0 or straight 0
     * @param action: Tells if the key got pressed or released ACTION::pressed/ACTION::released
     * @param pos: Mouse cursor position at the event.
     * @param mods: not used yet. but could be used for testing if Shift Alt or somthing similar is pressed
     */
    //virtual void mouseButtonCallback(MOUSE::button button, ACTION::action action, glm::vec2 pos,int mods);


    /**
     * @brief windowResizeCallback: Does exactly what you think it does
     * @param res
     */
    virtual void windowResizeCallback(glm::ivec2 res);

    static void setErrorCallback(void (*a)(int error,const char* description));

    void setKeyCallback(void (*a)(GLWindow* window,int key,int scancode,int action,int mods));
    void setMouseButtonCallback(void (*a)(GLWindow* window,int button,int action,int mod));
    void setCursorCallback(void (*a)(GLWindow* window,double x,double y));

    virtual void keyCallback(int key,int scancode,int action,int mods);
    virtual void cursorCallback(double x,double y);
    virtual void mouseButtonCallback(int button, int action, int mod);




    /*
     * these here: have yet to be implemented:
     *
     */
    glm::dvec2 getCursorPosition();
    void setCursorPosition(glm::dvec2 pos);
    int getMouseButton(int button);
    int getKey(int key);
    void setInputMode(int mode, int value);
    glm::ivec2 getResolution();

    /**
     * @brief pollEvents and waitEvents should get called every few milliseconds. They handle all user input events and so on.
     * waitEvents waits for one event to come.
     * pollEvents handles all events inside the queue (if there are some) but returns immediately if there are no events.
     */
    static void pollEvents();

    /**
     * @brief waitEvents
     */
    static void waitEvents();

};
}

#endif
