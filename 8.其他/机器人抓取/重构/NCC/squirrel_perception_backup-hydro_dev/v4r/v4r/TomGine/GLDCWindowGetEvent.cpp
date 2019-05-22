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

#ifdef WIN32

#include "GLWindow.h"
#include <stdio.h>

namespace V4R{

void MapKey(WPARAM wp, Input &input);

bool GLWindow::GetEvent(Event &event)
{
	short wheel = 0;
	MSG msg;
	if( PeekMessage( &msg, hWnd, 0, 0, PM_REMOVE )  ){
		// handle or dispatch messages
		//printf("msg.wParam: %d\n", msg.wParam);
		switch ( msg.message ){
			case WM_QUIT:
				event.type = TMGL_Quit;
				break;
			case WM_KEYDOWN:
				event.type = TMGL_Press;
				MapKey(msg.wParam, event.input);
				break;
			case WM_KEYUP:
				event.type = TMGL_Release;
				MapKey(msg.wParam, event.input);
				break;

			// Mouse input
			case WM_MOUSEMOVE:
				event.type = TMGL_Motion;
				event.motion.x = LOWORD(msg.lParam);
				event.motion.y = HIWORD(msg.lParam);
				break;
			case WM_LBUTTONDOWN:
				event.type = TMGL_Press;
				event.input = TMGL_Button1;
				break;
			case WM_LBUTTONUP:
				event.type = TMGL_Release;
				event.input = TMGL_Button1;
				break;
			case WM_MBUTTONDOWN:
				event.type = TMGL_Press;
				event.input = TMGL_Button2;
				break;
			case WM_MBUTTONUP:
				event.type = TMGL_Release;
				event.input = TMGL_Button2;
				break;
			case WM_RBUTTONDOWN:
				event.type = TMGL_Press;
				event.input = TMGL_Button3;
				break;
			case WM_RBUTTONUP:
				event.type = TMGL_Release;
				event.input = TMGL_Button3;
				break;
			case WM_MOUSEWHEEL:
				event.type = TMGL_Press;
				wheel = (short)HIWORD(msg.wParam)/WHEEL_DELTA;
				if(wheel == -1)
						event.input = TMGL_Button4;
				else
						event.input = TMGL_Button5;
				break;
			
			
			default:
				TranslateMessage( &msg );
				DispatchMessage( &msg );
				return false;
				break;
		}
		return true;
	}
	return false;
}

void MapKey(WPARAM wp, Input &input)
{
		switch(wp)
		{
		case 0x30:
			input = TMGL_0;
			break;
		case 0x31:
			input = TMGL_1;
			break;
		case 0x32:
			input = TMGL_2;
			break;
		case 0x33:
			input = TMGL_3;
			break;
		case 0x34:
			input = TMGL_4;
			break;
		case 0x35:
			input = TMGL_5;
			break;
		case 0x36:
			input = TMGL_6;
			break;
		case 0x37:
			input = TMGL_7;
			break;
		case 0x38:
			input = TMGL_8;
			break;
		case 0x39:
			input = TMGL_9;
			break;
		
		case 0x41:
			input = TMGL_a;
			break;
		case 0x42:
			input = TMGL_b;
			break;
		case 0x43:
			input = TMGL_c;
			break;
		case 0x44:
			input = TMGL_d;
			break;
		case 0x45:
			input = TMGL_e;
			break;
		case 0x46:
			input = TMGL_f;
			break;
		case 0x47:
			input = TMGL_g;
			break;
		case 0x48:
			input = TMGL_h;
			break;
		case 0x49:
			input = TMGL_i;
			break;
		case 0x4A:
			input = TMGL_j;
			break;
		case 0x4B:
			input = TMGL_k;
			break;
		case 0x4C:
			input = TMGL_l;
			break;
		case 0x4D:
			input = TMGL_m;
			break;
		case 0x4E:
			input = TMGL_n;
			break;
		case 0x4F:
			input = TMGL_o;
			break;
		case 0x50:
			input = TMGL_p;
			break;
		case 0x51:
			input = TMGL_q;
			break;
		case 0x52:
			input = TMGL_r;
			break;
		case 0x53:
			input = TMGL_s;
			break;
		case 0x54:
			input = TMGL_t;
			break;
		case 0x55:
			input = TMGL_u;
			break;
		case 0x56:
			input = TMGL_v;
			break;
		case 0x57:
			input = TMGL_w;
			break;
		case 0x58:
			input = TMGL_x;
			break;
		case 0x59:
			input = TMGL_y;
			break;
		case 0x5A:
			input = TMGL_z;
			break;

		case VK_BACK:
			input = TMGL_BackSpace;
			break;
		case VK_TAB:
			input = TMGL_Tab;
			break;
		case VK_RETURN:
			input = TMGL_Return;
			break;
		case VK_SPACE:
			input = TMGL_Space;
			break;
		case VK_PAUSE:
			input = TMGL_Pause;
			break;
		case VK_ESCAPE:
			input = TMGL_Escape;
			break;
		case VK_DELETE:
			input = TMGL_Delete;
			break;
		case VK_LEFT:
			input = TMGL_Left;
			break;
		case VK_UP:
			input = TMGL_Up;
			break;
		case VK_RIGHT:
			input = TMGL_Right;
			break;
		case VK_DOWN:
			input = TMGL_Down;
			break;
		case VK_PRIOR:
			input = TMGL_Page_Up;
			break;
		case VK_NEXT:
			input = TMGL_Page_Down;
			break;
		case VK_END:
			input = TMGL_End;
			break;
		case VK_HOME:
			input = TMGL_Home;
			break;
		case VK_MULTIPLY:
			input = TMGL_KP_Multiply;
			break;
		case VK_ADD:
			input = TMGL_KP_Add;
			break;
		case VK_SUBTRACT:
			input = TMGL_KP_Subtract;
			break;
		case VK_DIVIDE:
			input = TMGL_KP_Divide;
			break;
		case VK_NUMPAD0:
			input = TMGL_KP_0;
			break;
		case VK_NUMPAD1:
			input = TMGL_KP_1;
			break;
		case VK_NUMPAD2:
			input = TMGL_KP_2;
			break;
		case VK_NUMPAD3:
			input = TMGL_KP_3;
			break;
		case VK_NUMPAD4:
			input = TMGL_KP_4;
			break;
		case VK_NUMPAD5:
			input = TMGL_KP_5;
			break;
		case VK_NUMPAD6:
			input = TMGL_KP_6;
			break;
		case VK_NUMPAD7:
			input = TMGL_KP_7;
			break;
		case VK_NUMPAD8:
			input = TMGL_KP_8;
			break;
		case VK_NUMPAD9:
			input = TMGL_KP_9;
			break;
		case VK_F1:
			input = TMGL_F1;
			break;
		case VK_F2:
			input = TMGL_F2;
			break;
		case VK_F3:
			input = TMGL_F3;
			break;
		case VK_F4:
			input = TMGL_F4;
			break;
		case VK_F5:
			input = TMGL_F5;
			break;
		case VK_F6:
			input = TMGL_F6;
			break;
		case VK_F7:
			input = TMGL_F7;
			break;
		case VK_F8:
			input = TMGL_F8;
			break;
		case VK_F9:
			input = TMGL_F9;
			break;
		case VK_F10:
			input = TMGL_F10;
			break;
		case VK_F11:
			input = TMGL_F11;
			break;
		case VK_F12:
			input = TMGL_F12;
			break;
		case VK_LSHIFT:
			input = TMGL_Shift_L;
			break;
		case VK_RSHIFT:
			input = TMGL_Shift_R;
			break;
		case VK_LCONTROL:
			input = TMGL_Control_L;
			break;
		case VK_RCONTROL:
			input = TMGL_Control_R;
			break;
	}
}

} /* namespace */

#endif /* WIN32 */

