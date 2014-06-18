/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "aslam/calibration/exceptions/InvalidOperationException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Statics                                                                    */
/******************************************************************************/

    template <class C> C* Singleton<C>::instance = 0;

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    template <class C>
    Singleton<C>::Singleton() {
      if (instance)
        throw InvalidOperationException(
          "Singleton<C>::Singleton(): a singleton cannot be instantiated");
      instance = (C*)this;
    }

    template <class C>
    Singleton<C>::~Singleton() {
      instance = 0;
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <class C>
    C& Singleton<C>::getInstance() {
      if (!instance)
        new C();
      return *instance;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    template <class C>
    bool Singleton<C>::exists() {
      return (instance != 0);
    }

  }
}
