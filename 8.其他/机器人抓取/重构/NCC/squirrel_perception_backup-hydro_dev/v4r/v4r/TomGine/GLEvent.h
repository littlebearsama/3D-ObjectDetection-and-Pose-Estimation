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

#ifndef _GL_WINDOW_EVENT_H_
#define _GL_WINDOW_EVENT_H_

#include "GLInput.h"
#include <iostream>

namespace TomGine{

/** @brief Input event class */
class Event{
public:
	struct Motion {
		int x;
    int y;
	};
	
  struct Exposure {
		int width;
		int height;
  };
	
	Type		type;		///> Type of input event (defined in GLInput.h)
	Input		input;	///> Key- or mouse constant (defined in GLInput.h)
	Motion		motion; ///> Mouse position in pixels if event is of type TMGL_Motion
	Exposure	expose; ///> Exposure parameter if event is of type TMGL_Expose
	
  Event() : type(TMGL_None){}
};

} /* namespace */

#endif /* _GL_WINDOW_EVENT_H_ */
