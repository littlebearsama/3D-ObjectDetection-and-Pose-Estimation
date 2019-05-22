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

#ifndef _GL_WINDOW_INPUT_H_
#define _GL_WINDOW_INPUT_H_

namespace TomGine{

enum Type{
	TMGL_None=0,
	TMGL_Press=1,
	TMGL_Release=2,
	TMGL_Motion=3,
	TMGL_Expose=4,
	TMGL_Quit=5,
};

enum Input{
	/* Mouse */
	TMGL_Button1=0,
	TMGL_Button2=1,
	TMGL_Button3=2,
	TMGL_Button4=3,
	TMGL_Button5=4,
	TMGL_Button6=5,
	TMGL_Button7=6,
	TMGL_Button8=7,
	TMGL_Button9=8,
	/* Key */
	TMGL_0=8,
	TMGL_1=9,
	TMGL_2=10,
	TMGL_3=11,
	TMGL_4=12,
	TMGL_5=13,
	TMGL_6=14,
	TMGL_7=15,
	TMGL_8=16,
	TMGL_9=17,
	TMGL_a=18,
	TMGL_b=19,
	TMGL_c=20,
	TMGL_d=21,
	TMGL_e=22,
	TMGL_f=23,
	TMGL_g=24,
	TMGL_h=25,
	TMGL_i=26,
	TMGL_j=27,
	TMGL_k=28,
	TMGL_l=29,
	TMGL_m=30,
	TMGL_n=31,
	TMGL_o=32,
	TMGL_p=33,
	TMGL_q=34,
	TMGL_r=35,
	TMGL_s=36,
	TMGL_t=37,
	TMGL_u=38,
	TMGL_v=39,
	TMGL_w=40,
	TMGL_x=41,
	TMGL_y=42,
	TMGL_z=43,

	TMGL_Space=44,
	TMGL_BackSpace=45,
	TMGL_Tab=46,
	TMGL_Linefeed=47,
	TMGL_Clear=48,
	TMGL_Return=49,
	TMGL_Pause=50,
	TMGL_Scroll_Lock=51,
	TMGL_Sys_Req=52,
	TMGL_Escape=53,
	TMGL_Delete=54,
	TMGL_Home=55,
	TMGL_Left=56,
	TMGL_Up=57,
	TMGL_Right=58,
	TMGL_Down=59,
	TMGL_Prior=60,
	TMGL_Page_Up=61,
	TMGL_Next=62,
	TMGL_Page_Down=63,
	TMGL_End=64,
	TMGL_Begin=65,
	TMGL_KP_Enter=66,
	TMGL_KP_Home=67,
	TMGL_KP_Left=68,
	TMGL_KP_Up=69,
	TMGL_KP_Right=70,
	TMGL_KP_Down=71,
	TMGL_KP_Prior=72,
	TMGL_KP_Page_Up=73,
	TMGL_KP_Next=74,
	TMGL_KP_Page_Down=75,
	TMGL_KP_End=76,
	TMGL_KP_Begin=77,
	TMGL_KP_Insert=78,
	TMGL_KP_Delete=79,
	TMGL_KP_Equal=80,
	TMGL_KP_Multiply=81,
	TMGL_KP_Add=82,
	TMGL_KP_Separator=83,
	TMGL_KP_Subtract=84,
	TMGL_KP_Decimal=85,
	TMGL_KP_Divide=86,
	TMGL_KP_0=87,
	TMGL_KP_1=88,
	TMGL_KP_2=89,
	TMGL_KP_3=90,
	TMGL_KP_4=91,
	TMGL_KP_5=92,
	TMGL_KP_6=93,
	TMGL_KP_7=94,
	TMGL_KP_8=95,
	TMGL_KP_9=96,
	TMGL_F1=97,
	TMGL_F2=98,
	TMGL_F3=99,
	TMGL_F4=100,
	TMGL_F5=101,
	TMGL_F6=102,
	TMGL_F7=103,
	TMGL_F8=104,
	TMGL_F9=105,
	TMGL_F10=106,
	TMGL_F11=107,
	TMGL_F12=108,
	TMGL_Shift_L=109,
	TMGL_Shift_R=110,
	TMGL_Control_L=111,
	TMGL_Control_R=112,
	TMGL_Caps_Lock=113,
	TMGL_Shift_Lock=114,
	TMGL_Meta_L=115,
	TMGL_Meta_R=116,
	TMGL_Alt_L=117,
	TMGL_Alt_R=118,
	TMGL_Super_L=119,
	TMGL_Super_R=120,
	TMGL_Hyper_L=121,
	TMGL_Hyper_R=122,
};

} /* namespace */


#endif /* _GL_WINDOW_INPUT_H_ */
