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

#include <stdio.h>

#ifndef _TG_SINGLETON_H_
#define _TG_SINGLETON_H_

namespace TomGine
{

  /** @brief singleton class for global instantiation. */
  template<typename T>
    class tgSingleton
    {
    public:
      static T*
      GetInstance ()
      {
        if (!m_instance)
          m_instance = new T ();
        return m_instance;
      }
      virtual
      ~tgSingleton ()
      {
//        if (m_instance)
//          delete (m_instance);
//        m_instance = 0;
      }

    private:
      static T* m_instance;
    protected:
      tgSingleton ()
      {
      }
    };

  template<typename T>
    T* tgSingleton<T>::m_instance = 0;

}

#endif
