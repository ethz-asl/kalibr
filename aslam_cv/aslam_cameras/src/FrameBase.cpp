#include <aslam/FrameBase.hpp>

namespace aslam {

FrameBase::FrameBase() {
}
FrameBase::~FrameBase() {
}

size_t FrameBase::numKeypointsWithALandmarkId() const
{
	size_t numKeypointsWithALandmarkId = 0;
	size_t n = numKeypoints();
	for(size_t k = 0; k < n; k++)
	{
		if (keypointBase(k).landmarkId().isSet())
		{
			numKeypointsWithALandmarkId++;
		}
	}
	return numKeypointsWithALandmarkId;
}

}  // namespace aslam
