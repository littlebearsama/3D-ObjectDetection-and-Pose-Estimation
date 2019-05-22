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

#ifndef TOMGUI_TGPLOT2D
#define TOMGUI_TGPLOT2D

#include "tgMathlib.h"
#include "tgTimer.h"
#include "tgFont.h"

#include <string>
#include <vector>

namespace TomGine{

/** @brief Display data in an oscilloscope style. */
class tgPlot2D
{
protected:
	int x,y;			///<	x and y position of plot grid in pixel
	unsigned w,h;		///<	width and height of plot grid in pixel
	int m_fontsize;		///<	font size for lables, legend, ...
	vec3 m_fontcolor; ///< font color for lables, legend, ...
	vec3 m_gridcolor;
	vec3 m_axiscolor;
	vec3 m_y1_color;
	vec3 m_y2_color;
	vec3 m_y3_color;
	vec3 m_y4_color;
	
	std::vector<float> m_buffer_x;
	std::vector<float> m_buffer_y_1;
	std::vector<float> m_buffer_y_2;
	std::vector<float> m_buffer_y_3;
	std::vector<float> m_buffer_y_4;
	unsigned m_buffer_size;
	
	float x_max, x_min, y_min, y_max;
	float x_scale, y_scale;
	
	TomGine::tgTimer m_timer;
	
	float findmax(const std::vector<float> &x) const;
	float findmin(const std::vector<float> &x) const;
	
	void updateBuffer(float y1, float y2=0.0f, float y3=0.0f, float y4=0.0f);
	

public:

	/** @brief Create plot and define position and size.
	 *  @param x,y	Position of the lower left corner of the plot grid in pixels in window coordinates.
	 *  @param w,h	Size of the plot grid in pixels. */
	tgPlot2D(int x=0, int y=0, unsigned w=100, unsigned h=100);
	
	/** @brief Define position and size.
	 *  @param x,y	Position of the lower left corner of the plot grid in pixels in window coordinates.
	 *  @param w,h	Size of the plot grid in pixels. */
	void Define(int x, int y, unsigned w, unsigned h);

	/** @brief Clears the plot and starts drawing data from left to right. */
	void Reset();

	/** @brief Define range of axis of plot.
	 *  @param x_min	Minimum value on x-axis.
	 *  @param x_max	Maximum value on x-axis.
	 *  @param y_min	Minimum value on y-axis.
	 *  @param y_max	Maximum value on y-axis. */
	void Axis(float x_min, float x_max, float y_min, float y_max);
	
	/** @brief Set the size of the buffer.
	 *  @brief This is the maximum number of values displayed over the width of the plot (default = w of plot). */
	void SetBufferSize(unsigned size){ m_buffer_size = size; }
	
	/** @brief Set size of font for lables, legend, ...
	 *  @param size		Size of the font (default is 14) */
	void SetFontSize(unsigned size){ m_fontsize = size; }

	/** @brief Set color of font for lables, legend, ...
   *  @param r,g,b   Color of the font (default is 0.5) */
	void SetFontColor(float r, float g, float b){ m_fontcolor = vec3(r,g,b); }

	/** @brief Set color of grid
   *  @param r,g,b   Color of the grid (default is 0.5) */
	void SetGridColor(float r, float g, float b){ m_gridcolor = vec3(r,g,b); }

	/** @brief Set color of signal y1
   *  @param r,g,b   Color of the signal (default is 1.0, 1.0, 0.0) */
	void SetY1Color(float r, float g, float b){ m_y1_color = vec3(r,g,b); }

	/** @brief Set color of signal y2
   *  @param r,g,b   Color of the signal (default is 1.0, 0.0, 1.0) */
	void SetY2Color(float r, float g, float b){ m_y2_color = vec3(r,g,b); }

	/** @brief Set color of signal y3
   *  @param r,g,b   Color of the signal (default is 0.0, 1.0, 1.0) */
	void SetY3Color(float r, float g, float b){ m_y3_color = vec3(r,g,b); }

	/** @brief Set color of signal y4
   *  @param r,g,b   Color of the signal (default is 0.0, 1.0, 0.0) */
	void SetY4Color(float r, float g, float b){ m_y4_color = vec3(r,g,b); }

	/** @brief Set color of axis
   *  @param r,g,b   Color of the axis (default is 0.5) */
	void SetAxisColor(float r, float g, float b){ m_axiscolor = vec3(r,g,b); }

	/** @brief Draw the axis including numbers and grid. */
	void DrawAxis(bool lables=true, bool grid=true);
	
	/** @brief Draws the labels to x and y axis. (not implemented)
	 *  @param xlabel	Text for the label of the x-axis
	 *  @param ylabel	Text for the label of the y-axis */
	void DrawLabel(std::string xlabel="", std::string ylabel="");

	/** @brief Draw color legend to data signals.
	 *  @param x,y		position of legend (top-left corner)
	 *  @param y1-y4	strings displayed as lables.	 */
	void DrawLegend(float x, float y, std::string y1, std::string y2="", std::string y3="", std::string y4="");

	/** @brief Plots vector y versus vector x.
	 *  @param x,y	Vectors of same length where each pair x[i],y[i] is a data point on the plot. */
	void DrawData(const std::vector<float> &x, const std::vector<float> &y) const;
	
	/** @brief Pushes data into the buffer, where x is updated using the current time.
	 *  y1	Data displayed in yellow. */
	void Push(float y1);

	/** @brief Pushes data into the buffers, where x is updated using the current time. Two streams of data parallel.
	 *  y1	Data displayed in yellow.
	 *  y2	Data displayed in magenta */
	void Push(float y1, float y2);

	/** @brief Pushes data into the buffers, where x is updated using the current time. Three streams of data parallel.
	 *  y1	Data displayed in yellow.
	 *  y2	Data displayed in magenta.
	 *  y3	Data displayed in cyan. */
	void Push(float y1, float y2, float y3);

	/** @brief Pushes data into the buffers, where x is updated using the current time. Four streams of data parallel.
	 *  y1	Data displayed in yellow.
	 *  y2	Data displayed in magenta.
	 *  y3	Data displayed in cyan.
	 *  y4	Data displayed in green. */
	void Push(float y1, float y2, float y3, float y4);
	
};

}

#endif
