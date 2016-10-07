/*
 * OPTBSpline.hpp
 *
 *  Created on: 05.08.2012
 *      Author: hannes
 */

#ifndef OPTEUCLIDEANBSPLINE_HPP_
#define OPTEUCLIDEANBSPLINE_HPP_

#include "OPTBSpline.hpp"
#include "bsplines/EuclideanBSpline.hpp"

namespace bsplines {

template <typename TDiffManifoldConfiguration, int ISplineOrder, typename TTimePolicy, typename TModifiedDerivedConf>
class DiffManifoldBSpline<aslam::splines::DesignVariableSegmentBSplineConf<EuclideanBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>, TModifiedDerivedConf>, aslam::splines::DesignVariableSegmentBSplineConf<TModifiedDerivedConf> >
	: public DiffManifoldBSpline<aslam::splines::DesignVariableSegmentBSplineConf<typename EuclideanBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>::ParentConf, TModifiedDerivedConf>, aslam::splines::DesignVariableSegmentBSplineConf<TModifiedDerivedConf> >{
 protected:
	typedef DiffManifoldBSpline<aslam::splines::DesignVariableSegmentBSplineConf<typename EuclideanBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>::ParentConf, TModifiedDerivedConf>, aslam::splines::DesignVariableSegmentBSplineConf<TModifiedDerivedConf> > parent_t;
	typedef aslam::splines::DesignVariableSegmentBSplineConf<TModifiedDerivedConf> CONF;
 public:
	DiffManifoldBSpline(const CONF & config = CONF()) : parent_t(config){}
	DiffManifoldBSpline(int splineOrder, int dimension = parent_t::Dimension) : parent_t(typename CONF::ParentConf(typename CONF::ManifoldConf(dimension), splineOrder)){}

};

} // namespace bsplines

#endif /* OPTEUCLIDEANBSPLINE_HPP_ */
