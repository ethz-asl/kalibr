#ifndef ASLAM_FRONTEND_IDS_HPP
#define ASLAM_FRONTEND_IDS_HPP

#include <sm/Id.hpp>

namespace aslam {

// The camera system frame id.
SM_DEFINE_ID (MultiFrameId);
SM_DEFINE_ID (FrameId);
SM_DEFINE_ID (LandmarkId);

}  // namespace aslam

// These macros specialize std::tr1::hash for 
// The id types. They must be defined outside
// of a namespace.
SM_DEFINE_ID_HASH (aslam::MultiFrameId);
SM_DEFINE_ID_HASH (aslam::FrameId);
SM_DEFINE_ID_HASH (aslam::LandmarkId);

#endif /* ASLAM_FRONTEND_IDS_HPP */
