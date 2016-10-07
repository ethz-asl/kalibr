/*
 * DiffManifoldBSplineTools.hpp
 *
 *  Created on: Aug 27, 2013
 *      Author: hannes
 */

#ifndef DIFFMANIFOLDBSPLINETOOLS_HPP_
#define DIFFMANIFOLDBSPLINETOOLS_HPP_


namespace bsplines {
namespace internal{

template<typename Iterator>
inline int moveIterator(Iterator & it, const Iterator & limit, int steps)
{
	if(steps == 0)
		return 0;
	int count = 0;
	if(steps > 0){
		for (int c = steps; c>0; c--){
			if(it == limit)
				break;
			it++;
			count ++;
		}
	}
	else{
		for (int c = -steps; c>0; c--){
			if(it == limit)
				break;
			it--;
			count --;
		}
	}

	return count;
}

template<typename Iterator>
inline Iterator getMovedIterator(const Iterator & it, const Iterator & limit, int steps)
{
	Iterator nIt(it);
	moveIterator(nIt, limit, steps);
	return nIt;
}

}
}

#endif /* DIFFMANIFOLDBSPLINETOOLS_HPP_ */
