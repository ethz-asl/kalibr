// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SBM_G2O_SPARSE_HELPER_H
#define SBM_G2O_SPARSE_HELPER_H


#include <string>
#include <fstream>
#include <iomanip>
#include <vector>
#include <algorithm>

#include <cholmod.h>

namespace sparse_block_matrix {



namespace {
  struct TripletEntry
  {
    int r, c;
    double x;
    TripletEntry(int r_, int c_, double x_) : r(r_), c(c_), x(x_) {}
  };
  struct TripletColSort
  {
    bool operator()(const TripletEntry& e1, const TripletEntry& e2) const
    {
      return e1.c < e2.c || (e1.c == e2.c && e1.r < e2.r);
    }
  };
}



  /**
   * write an array to a file, debugging
   */
  bool writeVector(const char* filename, const double*v, int n);

  /**
   * write a CCS matrix given by pointer to column, row, and values
   */
  template<typename T>
  bool writeCCSMatrix(const char* filename, int rows, int cols, const T* p, const T* i, const double* v, bool upperTriangleSymmetric = true);

 // bool writeCCSMatrix(const char* filename, int rows, int cols, const int* Ap, const int* Ai, const double* Ax, bool upperTriangleSymmetric = true);
    
    
    /**
     * \brief Our extension of the CHOLMOD matrix struct
     *  used to express sparseBlockMatrix as a Cholmod Sparse Matrix
     */

  template<typename T>
  struct IntType
  {
    static const int intType = CHOLMOD_INT;
  };
  template<>
  struct IntType< long >
  {
      static const int intType = CHOLMOD_INTLONG;
  };
  template<>
  struct IntType< int >
  {
    static const int intType = CHOLMOD_INT;
  };

    template<typename INTTYPE>
    struct CholmodExt: public cholmod_sparse
    {
        CholmodExt()
        {
            nzmax = 0;
            nrow  = 0;
            ncol  = 0;
            p     = 0;
            i     = 0;
            nz    = 0;
            x     = 0;
            z     = 0;
            stype = 1; // upper triangular block only
            itype = IntType<INTTYPE>::intType;
            xtype = CHOLMOD_REAL;
            dtype = CHOLMOD_DOUBLE;
            sorted = 1;
            packed = 1;
            columnsAllocated = 0;
        }

        ~CholmodExt()
        {
            if(p) delete[] (INTTYPE*)p; p = 0;
            if(x) delete[] (double*)x; x = 0;
            if(i) delete[] (INTTYPE*)i; i = 0;
        }
        size_t columnsAllocated;
    };
    
    
    


} // end namespace

#include "implementation/sparse_helper.h"

#endif


