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

/** \file NormalDistribution.h
    \brief This file is an interface to the normal distributions
  */

#ifndef ASLAM_CALIBRATION_STATISTICS_NORMALDISTRIBUTION_H
#define ASLAM_CALIBRATION_STATISTICS_NORMALDISTRIBUTION_H

namespace aslam {
  namespace calibration {

    template <int M = 1> class NormalDistribution;

  }
}

#include "aslam/calibration/statistics/NormalDistribution1v.h"
#include "aslam/calibration/statistics/NormalDistributionMv.h"

#endif // ASLAM_CALIBRATION_STATISTICS_NORMALDISTRIBUTION_H
