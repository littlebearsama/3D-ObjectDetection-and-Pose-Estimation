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
#include "tgTimer.h"

using namespace TomGine;

// ***********************************************************************************

tgTimer::tgTimer(void) {
	Reset();
}

void tgTimer::Reset() {
#ifdef WIN32
	QueryPerformanceFrequency((LARGE_INTEGER*) &m_Frequency);
	QueryPerformanceCounter((LARGE_INTEGER*)(&m_StartTicks));
	m_EndTicks = m_StartTicks;
#else
	clock_gettime(CLOCK_REALTIME, &AppStart);
	clock_gettime(CLOCK_REALTIME, &old);
#endif
	m_fAppTime = 0.0;
}

double tgTimer::Update() {
#ifdef WIN32
	QueryPerformanceCounter((LARGE_INTEGER*)(&m_EndTicks));
	fNow = (double)(m_EndTicks - m_StartTicks) / m_Frequency;
	m_fTime = fNow - m_fAppTime;
	m_fAppTime = fNow;
#else
	clock_gettime(CLOCK_REALTIME, &act);
	m_fTime = (act.tv_sec - old.tv_sec) + (act.tv_nsec - old.tv_nsec) / 1e9;
	old = act;		
	m_fAppTime += m_fTime;
#endif
	return m_fTime;
}
