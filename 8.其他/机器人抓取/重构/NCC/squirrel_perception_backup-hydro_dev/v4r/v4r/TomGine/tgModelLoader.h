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
 
#ifndef TG_MODELLOADER
#define TG_MODELLOADER

#include <stdlib.h>
#include <stddef.h>
#include <string>

#include "ply.h"
#include "PlyStructure.h"
#include "tgModel.h"
#include "tgRenderModel.h"


namespace TomGine{

/** @brief Loading geometrical models from file. */
class tgModelLoader
{
private:

	static bool propertyIsInList(PlyProperty* prop, PlyProperty* list, int n, int* index);

public:
	
	/** @brief Loads PLY files (Polygon File Format, Stanford Triangle Format)
	*		@param model	Model to load file to.
	*		@param filename	Path and filename of file to load.
	*		@return True on success, false on failure. */
	static bool LoadPly(tgModel &model, const char* filename);
	
	/** @brief Saves PLY files (Polygon File Format, Stanford Triangle Format)
	*	@param model Model to save.
	*	@param filename Path and filename of file to save to.
	*	@return True on success, false on failure. */
  static bool SavePly(const tgModel &model, const std::string &filename,
                      bool with_normals=true, bool with_tex_coords=false, bool with_color=false);
	   
};

} // namespace TomGine

#endif
