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

#ifndef _TG_ERROR_H
#define _TG_ERROR_H

#include "headers.h"
#include <string>

namespace TomGine{

/** @brief Check for OpenGL errors with a message prefix. \n
 * 	Note that only the first error since the last call of tgCheckError will be reported by OpenGL.
 * 	This might lead to wrong prefix messages.
 *  @param pre_msg	Message prefix.
 *  @param detailed	Enable/Disable detailed description of OpenGL errors. */
GLenum tgCheckError(std::string pre_msg, bool detailed=false);

/** @brief Check for OpenGL frame-buffer errors with a prefix message.
 *  @param target usually GL_FRAMEBUFFER (see glCheckFramebufferStatus in OpenGL spec.).
 *  @param pre_msg	message prefix. */
GLenum tgCheckFBError(GLenum target, std::string pre_msg);

} // namespace TomGine

#endif
