#ifndef GRIDDER_H
#define GRIDDER_H

#include <algorithm>
#include <iterator>
#include <vector>

#include "apriltags/Segment.h"

namespace AprilTags {

//! A lookup table in 2D for implementing nearest neighbor.
template <class T>
class Gridder {
  private:
	Gridder(const Gridder&); //!< don't call
	Gridder& operator=(const Gridder&); //!< don't call
	
  struct Cell {
    T* object;
    Cell *next;

    Cell() : object(NULL), next(NULL) {}

    Cell(const Cell& c) : object(c.object), next(c.next) {}

    // Destructor
    ~Cell() {
      delete next;
    }

    Cell& operator=(const Cell &other) {
      if (this == &other)
	return *this;

      object = other.object;
      next = other.next;
      return *this;
    }
  };

  //! Initializes Gridder constructor
  void gridderInit(float x0Arg, float y0Arg, float x1Arg, float y1Arg, float ppCell) {
    width = (int) ((x1Arg - x0Arg)/ppCell + 1);
    height = (int) ((y1Arg - y0Arg)/ppCell + 1);
    
    x1 = x0Arg + ppCell*width;
    y1 = y0Arg + ppCell*height;
    cells = std::vector< std::vector<Cell*> >(height, std::vector<Cell*>(width,(Cell*)NULL));
  }

  float x0,y0,x1,y1;
  int width, height;
  float pixelsPerCell; //pixels per cell
  std::vector< std::vector<Cell*> > cells;
 
public:
  Gridder(float x0Arg, float y0Arg, float x1Arg, float y1Arg, float ppCell)
    : x0(x0Arg), y0(y0Arg), x1(), y1(), width(), height(), pixelsPerCell(ppCell),
      cells() { gridderInit(x0Arg, y0Arg, x1Arg, y1Arg, ppCell); }

  // Destructor
  ~Gridder() {
    for (unsigned int i = 0; i < cells.size(); i++) {
      for (unsigned int j = 0; j < cells[i].size(); j++)
	delete cells[i][j];
    }
  }

  void add(float x, float y, T* object) {
    int ix = (int) ((x - x0)/pixelsPerCell);
    int iy = (int) ((y - y0)/pixelsPerCell);

    if (ix>=0 && iy>=0 && ix<width && iy<height) {
      Cell *c = new Cell;
      c->object = object;
      c->next = cells[iy][ix];
      cells[iy][ix] = c;
      // cout << "Gridder placed seg " << o->getId() << " at (" << ix << "," << iy << ")" << endl;
    }
  }
  
  // iterator begin();
  // iterator end();

  //! Iterator for Segment class.
  class Iterator {
  public:
    Iterator(Gridder* grid, float x, float y, float range)
      : outer(grid), ix0(), ix1(), iy0(), iy1(), ix(), iy(), c(NULL) { iteratorInit(x,y,range); }
	  
	  Iterator(const Iterator& it)
	  : outer(it.outer), ix0(it.ix0), ix1(it.ix1), iy0(it.iy0), iy1(it.iy1), ix(it.ix), iy(it.iy), c(it.c) {}
	  
	  Iterator& operator=(const Iterator& it) {
		  outer = it.outer;
		  ix0 = it.ix0;
		  ix1 = it.ix1;
		  iy0 = it.iy0;
		  iy1 = it.iy1;
		  ix = it.ix;
		  iy = it.iy;
		  c = it.c;
	  }

    bool hasNext() {
      if (c == NULL)
	findNext();
      return (c != NULL);
    }

    T& next() {
      T* thisObj = c->object;
      findNext();
      return *thisObj; // return Segment
    }

  private:
    void findNext() {
      if (c != NULL)
	c = c->next;
      if (c != NULL)
	return;

      ix++;
      while (true) {
	if (ix > ix1) {
	  iy++;
	  ix = ix0;
	}
	if (iy > iy1)
	  break;

        c = outer->cells[iy][ix];
       
	if (c != NULL)
	  break;
	ix++;
      }
    }

    //! Initializes Iterator constructor
    void iteratorInit(float x, float y, float range) {
      ix0 = (int) ((x - range - outer->x0)/outer->pixelsPerCell);
      iy0 = (int) ((y - range - outer->y0)/outer->pixelsPerCell);

      ix1 = (int) ((x + range - outer->x0)/outer->pixelsPerCell);
      iy1 = (int) ((y + range - outer->y0)/outer->pixelsPerCell);

      ix0 = std::max(0, ix0);
      ix0 = std::min(outer->width-1, ix0);

      ix1 = std::max(0, ix1);
      ix1 = std::min(outer->width-1, ix1);

      iy0 = std::max(0, iy0);
      iy0 = std::min(outer->height-1, iy0);

      iy1 = std::max(0, iy1);
      iy1 = std::min(outer->height-1, iy1);

      ix = ix0;
      iy = iy0;

      c = outer->cells[iy][ix];
    }

    Gridder* outer;
    int ix0, ix1, iy0, iy1;
    int ix, iy;
    Cell *c;
  };

  typedef Iterator iterator;
  iterator find(float x, float y, float range) { return Iterator(this,x,y,range); }
};

} // namespace

#endif
