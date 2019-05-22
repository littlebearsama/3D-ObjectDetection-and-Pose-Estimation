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

#include "GLWindow.h"
#include "GLEvent.h"
#include <stdio.h>

namespace TomGine {

void MapMouse(unsigned int xbutton, Input &input);
void MapKey(KeySym ks, Input &input);

void SearchEventType(int type)
{
  switch (type) {
  case KeyPress:
    printf("[GLWindow::SearchEventType] KeyPress\n");
    break;
  case KeyRelease:
    printf("[GLWindow::SearchEventType] KeyRelease\n");
    break;
  case ButtonPress:
    printf("[GLWindow::SearchEventType] ButtonPress\n");
    break;
  case ButtonRelease:
    printf("[GLWindow::SearchEventType] ButtonRelease\n");
    break;
  case MotionNotify:
    printf("[GLWindow::SearchEventType] MotionNotify\n");
    break;
  case EnterNotify:
    printf("[GLWindow::SearchEventType] EnterNotify\n");
    break;
  case LeaveNotify:
    printf("[GLWindow::SearchEventType] LeaveNotify\n");
    break;
  case FocusIn:
    printf("[GLWindow::SearchEventType] FocusIn\n");
    break;
  case FocusOut:
    printf("[GLWindow::SearchEventType] FocusOut\n");
    break;
  case KeymapNotify:
    printf("[GLWindow::SearchEventType] KeymapNotify\n");
    break;
  case Expose:
    printf("[GLWindow::SearchEventType] Expose\n");
    break;
  case GraphicsExpose:
    printf("[GLWindow::SearchEventType] GraphicsExpose\n");
    break;
  case NoExpose:
    printf("[GLWindow::SearchEventType] NoExpose\n");
    break;
  case CirculateRequest:
    printf("[GLWindow::SearchEventType] CirculateRequest\n");
    break;
  case ConfigureRequest:
    printf("[GLWindow::SearchEventType] ConfigureRequest\n");
    break;
  case MapRequest:
    printf("[GLWindow::SearchEventType] MapRequest\n");
    break;
  case ResizeRequest:
    printf("[GLWindow::SearchEventType] ResizeRequest\n");
    break;
  case CirculateNotify:
    printf("[GLWindow::SearchEventType] CirculateNotify\n");
    break;
  case ConfigureNotify:
    printf("[GLWindow::SearchEventType] ConfigureNotify\n");
    break;
  case CreateNotify:
    printf("[GLWindow::SearchEventType] CreateNotify\n");
    break;
  case DestroyNotify:
    printf("[GLWindow::SearchEventType] DestroyNotify\n");
    break;
  case GravityNotify:
    printf("[GLWindow::SearchEventType] GravityNotify\n");
    break;
  case MapNotify:
    printf("[GLWindow::SearchEventType] MapNotify\n");
    break;
  case MappingNotify:
    printf("[GLWindow::SearchEventType] MappingNotify\n");
    break;
  case ReparentNotify:
    printf("[GLWindow::SearchEventType] ReparentNotify\n");
    break;
  case UnmapNotify:
    printf("[GLWindow::SearchEventType] UnmapNotify\n");
    break;
  case VisibilityNotify:
    printf("[GLWindow::SearchEventType] VisibilityNotify\n");
    break;
  case ColormapNotify:
    printf("[GLWindow::SearchEventType] ColormapNotify\n");
    break;
  case ClientMessage:
    printf("[GLWindow::SearchEventType] ClientMessage\n");
    break;
  case PropertyNotify:
    printf("[GLWindow::SearchEventType] PropertyNotify\n");
    break;
  case SelectionClear:
    printf("[GLWindow::SearchEventType] SelectionClear\n");
    break;
  case SelectionNotify:
    printf("[GLWindow::SearchEventType] SelectionNotify\n");
    break;
  case SelectionRequest:
    printf("[GLWindow::SearchEventType] SelectionRequest\n");
    break;
  default:
    printf("[GLWindow::SearchEventType] Not found\n");
    break;
  }
}

void GLWindow::GetEventBlocking(Event &event)
{
  XEvent xev;
  int keysyms_per_keycode_return;

  XNextEvent(dpy, &xev);

  while (xev.type == ConfigureNotify || xev.type == MapNotify || xev.type == ReparentNotify)
    XNextEvent(dpy, &xev);

  if (xev.type == ClientMessage || xev.type == DestroyNotify || xev.type == UnmapNotify) {
    event.type = TMGL_Quit;
  } else if (xev.type == Expose) {
    XWindowAttributes gwa;
    event.type = TMGL_Expose;
    XGetWindowAttributes(dpy, glWin, &gwa);
    event.expose.width = gwa.width;
    event.expose.height = gwa.height;
  } else if (xev.type == KeyPress) {
    event.type = TMGL_Press;
    KeySym ks = *XGetKeyboardMapping(dpy,xev.xkey.keycode,1,&keysyms_per_keycode_return);
    MapKey(ks, event.input);
  } else if (xev.type == KeyRelease) {
    event.type = TMGL_Release;
    KeySym ks = *XGetKeyboardMapping(dpy,xev.xkey.keycode,1,&keysyms_per_keycode_return);
    MapKey(ks, event.input);
  } else if (xev.type == ButtonPress) {
    event.type = TMGL_Press;
    MapMouse(xev.xbutton.button, event.input);
    event.motion.x = xev.xmotion.x;
    event.motion.y = xev.xmotion.y;
  } else if (xev.type == ButtonRelease) {
    event.type = TMGL_Release;
    MapMouse(xev.xbutton.button, event.input);
    event.motion.x = xev.xmotion.x;
    event.motion.y = xev.xmotion.y;
  } else if (xev.type == MotionNotify) {
    event.type = TMGL_Motion;
    event.motion.x = xev.xmotion.x;
    event.motion.y = xev.xmotion.y;
  } else {
    printf("[GLWindow::GetEventBlocking] Warning: undefined event\n");
    event.type = TMGL_None;
    SearchEventType(xev.type);
  }
}

void GLWindow::UnBlockGetEvent()
{
  XEvent xev;
  xev.type = DestroyNotify;
  XSendEvent(dpy, glWin, True, StructureNotifyMask, &xev);
}

bool GLWindow::GetEvent(Event &event)
{

  if (XPending(dpy)) {
    int keysyms_per_keycode_return;
    XEvent xev;

    // Keyboard Key Press (Keysym code available in /usr/include/X11/keysymdef.h)
    if (XCheckWindowEvent(dpy, glWin, KeyPressMask, &xev)) {
      event.type = TMGL_Press;
      KeySym ks = *XGetKeyboardMapping(dpy,xev.xkey.keycode,1,&keysyms_per_keycode_return);
      MapKey(ks, event.input);
    }
    // Keyboard Key Release (Keysym code available in /usr/include/X11/keysymdef.h)
    else if (XCheckWindowEvent(dpy, glWin, KeyReleaseMask, &xev)) {
      event.type = TMGL_Release;
      KeySym ks = *XGetKeyboardMapping(dpy,xev.xkey.keycode,1,&keysyms_per_keycode_return);
      MapKey(ks, event.input);
    }
    // Mouse Button Press
    else if (XCheckWindowEvent(dpy, glWin, ButtonPressMask, &xev)) {
      event.type = TMGL_Press;
      MapMouse(xev.xbutton.button, event.input);
      event.motion.x = xev.xmotion.x;
      event.motion.y = xev.xmotion.y;
    }
    // Mouse Button Release
    else if (XCheckWindowEvent(dpy, glWin, ButtonReleaseMask, &xev)) {
      event.type = TMGL_Release;
      MapMouse(xev.xbutton.button, event.input);
      event.motion.x = xev.xmotion.x;
      event.motion.y = xev.xmotion.y;
    }
    // Mouse Motion
    else if (XCheckWindowEvent(dpy, glWin, PointerMotionMask, &xev)) {
      event.type = TMGL_Motion;
      event.motion.x = xev.xmotion.x;
      event.motion.y = xev.xmotion.y;
    }
    // Window Exposure
    else if (XCheckWindowEvent(dpy, glWin, ExposureMask, &xev)) {
      XWindowAttributes gwa;
      event.type = TMGL_Expose;
      XGetWindowAttributes(dpy, glWin, &gwa);
      event.expose.width = gwa.width;
      event.expose.height = gwa.height;
    }
    // Quit button pressed
    else if (XCheckTypedEvent(dpy, ClientMessage, &xev)) {
      event.type = TMGL_Quit;
    }
    // Other Events
    else {
      return false;
    }

    return true;
  }
  return false;
}

void MapMouse(unsigned int xbutton, Input &input)
{
  switch (xbutton) {
  case Button1:
    input = TMGL_Button1;
    break;
  case Button2:
    input = TMGL_Button2;
    break;
  case Button3:
    input = TMGL_Button3;
    break;
  case Button4:
    input = TMGL_Button4;
    break;
  case Button5:
    input = TMGL_Button5;
    break;
  case Button6:
    input = TMGL_Button6;
    break;
  case Button7:
    input = TMGL_Button7;
    break;
  case Button8:
    input = TMGL_Button8;
    break;
  case Button9:
    input = TMGL_Button9;
    break;
  default:
    break;
  }
}

void MapKey(KeySym ks, Input &input)
{
  switch (ks) {
  case XK_0:
    input = TMGL_0;
    break;
  case XK_1:
    input = TMGL_1;
    break;
  case XK_2:
    input = TMGL_2;
    break;
  case XK_3:
    input = TMGL_3;
    break;
  case XK_4:
    input = TMGL_4;
    break;
  case XK_5:
    input = TMGL_5;
    break;
  case XK_6:
    input = TMGL_6;
    break;
  case XK_7:
    input = TMGL_7;
    break;
  case XK_8:
    input = TMGL_8;
    break;
  case XK_9:
    input = TMGL_9;
    break;

  case XK_a:
    input = TMGL_a;
    break;
  case XK_b:
    input = TMGL_b;
    break;
  case XK_c:
    input = TMGL_c;
    break;
  case XK_d:
    input = TMGL_d;
    break;
  case XK_e:
    input = TMGL_e;
    break;
  case XK_f:
    input = TMGL_f;
    break;
  case XK_g:
    input = TMGL_g;
    break;
  case XK_h:
    input = TMGL_h;
    break;
  case XK_i:
    input = TMGL_i;
    break;
  case XK_j:
    input = TMGL_j;
    break;
  case XK_k:
    input = TMGL_k;
    break;
  case XK_l:
    input = TMGL_l;
    break;
  case XK_m:
    input = TMGL_m;
    break;
  case XK_n:
    input = TMGL_n;
    break;
  case XK_o:
    input = TMGL_o;
    break;
  case XK_p:
    input = TMGL_p;
    break;
  case XK_q:
    input = TMGL_q;
    break;
  case XK_r:
    input = TMGL_r;
    break;
  case XK_s:
    input = TMGL_s;
    break;
  case XK_t:
    input = TMGL_t;
    break;
  case XK_u:
    input = TMGL_u;
    break;
  case XK_v:
    input = TMGL_v;
    break;
  case XK_w:
    input = TMGL_w;
    break;
  case XK_x:
    input = TMGL_x;
    break;
  case XK_y:
    input = TMGL_y;
    break;
  case XK_z:
    input = TMGL_z;
    break;

  case XK_space:
    input = TMGL_Space;
    break;
  case XK_BackSpace:
    input = TMGL_BackSpace;
    break;
  case XK_Tab:
    input = TMGL_Tab;
    break;
  case XK_Return:
    input = TMGL_Return;
    break;
  case XK_Pause:
    input = TMGL_Pause;
    break;
  case XK_Escape:
    input = TMGL_Escape;
    break;
  case XK_Delete:
    input = TMGL_Delete;
    break;
  case XK_Left:
    input = TMGL_Left;
    break;
  case XK_Up:
    input = TMGL_Up;
    break;
  case XK_Right:
    input = TMGL_Right;
    break;
  case XK_Down:
    input = TMGL_Down;
    break;
  case XK_Page_Up:
    input = TMGL_Page_Up;
    break;
  case XK_Page_Down:
    input = TMGL_Page_Down;
    break;
  case XK_End:
    input = TMGL_End;
    break;
  case XK_Begin:
    input = TMGL_Begin;
    break;
  case XK_KP_Enter:
    input = TMGL_Return;
    break;
  case XK_KP_Multiply:
    input = TMGL_KP_Multiply;
    break;
  case XK_KP_Add:
    input = TMGL_KP_Add;
    break;
  case XK_KP_Subtract:
    input = TMGL_KP_Subtract;
    break;
  case XK_KP_Divide:
    input = TMGL_KP_Divide;
    break;
  case XK_KP_0:
  case XK_KP_Insert:
    input = TMGL_KP_0;
    break;
  case XK_KP_1:
  case XK_KP_End:
    input = TMGL_KP_1;
    break;
  case XK_KP_2:
  case XK_KP_Down:
    input = TMGL_KP_2;
    break;
  case XK_KP_3:
  case XK_KP_Page_Down:
    input = TMGL_KP_3;
    break;
  case XK_KP_4:
  case XK_KP_Left:
    input = TMGL_KP_4;
    break;
  case XK_KP_5:
  case XK_KP_Begin:
    input = TMGL_KP_5;
    break;
  case XK_KP_6:
  case XK_KP_Right:
    input = TMGL_KP_6;
    break;
  case XK_KP_7:
  case XK_KP_Home:
    input = TMGL_KP_7;
    break;
  case XK_KP_8:
  case XK_KP_Up:
    input = TMGL_KP_8;
    break;
  case XK_KP_9:
  case XK_KP_Page_Up:
    input = TMGL_KP_9;
    break;
  case XK_F1:
    input = TMGL_F1;
    break;
  case XK_F2:
    input = TMGL_F2;
    break;
  case XK_F3:
    input = TMGL_F3;
    break;
  case XK_F4:
    input = TMGL_F4;
    break;
  case XK_F5:
    input = TMGL_F5;
    break;
  case XK_F6:
    input = TMGL_F6;
    break;
  case XK_F7:
    input = TMGL_F7;
    break;
  case XK_F8:
    input = TMGL_F8;
    break;
  case XK_F9:
    input = TMGL_F9;
    break;
  case XK_F10:
    input = TMGL_F10;
    break;
  case XK_F11:
    input = TMGL_F11;
    break;
  case XK_F12:
    input = TMGL_F12;
    break;
  case XK_Shift_L:
    input = TMGL_Shift_L;
    break;
  case XK_Shift_R:
    input = TMGL_Shift_R;
    break;
  case XK_Control_L:
    input = TMGL_Control_L;
    break;
  case XK_Control_R:
    input = TMGL_Control_R;
    break;
  }
}

} /* namespace */

