/******************************************************************************
 * Copyright (C) 2011 by Jerome Maye                                          *
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

/** \file Grid.h
    \brief This file defines the Grid class, which represents an n-dimensional
           grid.
  */

#ifndef ASLAM_CALIBRATION_DATA_GRID_H
#define ASLAM_CALIBRATION_DATA_GRID_H

#include <vector>

#include <Eigen/Core>

#include "aslam/calibration/base/Serializable.h"
#include "aslam/calibration/utils/SizeTSupport.h"
#include "aslam/calibration/tpl/IsReal.h"
#include "aslam/calibration/tpl/IsInteger.h"

namespace aslam {
  namespace calibration {

    /** The class Grid represents an n-dimensional grid.
        \brief An n-dimensional grid
      */
    template <typename T, typename C, int M> class Grid :
      public virtual Serializable {
    public:
      /// \cond
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      // Template parameters assertion
      static_assert(M > 0 || M == Eigen::Dynamic, "M should be larger than 0!");
      /// \endcond

      /** \name Types definitions
        @{
        */
      /// Container type
      typedef std::vector<C> Container;
      /// Constant iterator type
      typedef typename Container::const_iterator ConstCellIterator;
      /// Iterator type
      typedef typename Container::iterator CellIterator;
      /// Index type
      typedef Eigen::Matrix<int, M, 1> Index;
      /// Coordinate type
      typedef Eigen::Matrix<T, M, 1> Coordinate;
      /** @}
        */

      /** \name Traits
        @{
        */
      /// Specialization for integer or real types
      struct Traits {
      public:
        /// Ceil function for real types
        template <typename Z, typename IsReal<Z>::Result::Numeric>
          static Z ceil(const Z& value);
        /// Ceil function for integer types
        template <typename Z, typename IsInteger<Z>::Result::Numeric>
          static Z ceil(const Z& value);
      };
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructs grid with parameters
      Grid(const Coordinate& minimum, const Coordinate& maximum,
        const Coordinate& resolution);
      /// Copy constructor
      Grid(const Grid& other);
      /// Assignment operator
      Grid& operator = (const Grid& other);
      /// Destructor
      virtual ~Grid();
      /** @}
        */

      /** \name Accessors
          @{
        */
      /// Returns iterator at start of the container
      ConstCellIterator getCellBegin() const;
      /// Returns iterator at start of the container
      CellIterator getCellBegin();
      /// Returns iterator at end of the container
      ConstCellIterator getCellEnd() const;
      /// Returns iterator at end of the container
      CellIterator getCellEnd();
      /// Returns the container
      const Container& getCells() const;
      /// Returns the cell at index
      const C& getCell(const Index& idx) const;
      /// Returns the cell at index
      C& getCell(const Index& idx);
      /// Returns a cell using [index] operator
      const C& operator [] (const Index& idx) const;
      /// Returns a cell using [index] operator
      C& operator [] (const Index& idx);
      /// Returns the index of a cell using coordinates
      virtual Index getIndex(const Coordinate& point) const;
      /// Returns a cell using (coordinate) operator
      virtual const C& operator () (const Coordinate& point) const;
      /// Returns a cell using (coordinate) operator
      virtual C& operator () (const Coordinate& point);
      /// Returns the coordinates of a cell using index
      virtual Coordinate getCoordinates(const Index& idx) const;
      /// Check if the grid contains the point
      virtual bool isInRange(const Coordinate& point) const;
      /// Check if an index is valid
      bool isValidIndex(const Index& idx) const;
      /// Returns the number of cells in each dimension
      const Index& getNumCells() const;
      /// Returns the total number of cells
      size_t getNumCellsTot() const;
      /// Returns the minimum of the grid
      const Coordinate& getMinimum() const;
      /// Returns the maximum of the grid
      const Coordinate& getMaximum() const;
      /// Returns the resolution of the grid
      const Coordinate& getResolution() const;
      /** @}
        */

      /** \name Methods
          @{
        */
      /// Computes linear index
      size_t computeLinearIndex(const Index& idx) const;
      /// Increment an index
      Index& incrementIndex(Index& idx) const;
      /// Reset the grid
      void reset();
      /** @}
        */

      /** \name Stream methods
        @{
        */
      /// Stream the grid into binary format
      virtual void writeBinary(std::ostream& stream) const;
      /// Reads the grid from a binary format
      virtual void readBinary(std::istream& stream);
      /** @}
        */

    protected:
      /** \name Stream methods
        @{
        */
      /// Reads from standard input
      virtual void read(std::istream& stream);
      /// Writes to standard output
      virtual void write(std::ostream& stream) const;
      /// Reads from a file
      virtual void read(std::ifstream& stream);
      /// Writes to a file
      virtual void write(std::ofstream& stream) const;
      /** @}
        */

      /** \name Protected members
          @{
        */
      /// Cell container
      Container mCells;
      /// Minimum coordinate of the grid
      Coordinate mMinimum;
      /// Maximum coordinate of the grid
      Coordinate mMaximum;
      /// Resolution of the grid
      Coordinate mResolution;
      /// Number of cells in each dimension
      Index mNumCells;
      /// Total number of cells
      size_t mNumCellsTot;
      /// Pre-computation for linear indices
      Index mLinProd;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/data-structures/Grid.tpp"

#endif // ASLAM_CALIBRATION_DATA_GRID_H
