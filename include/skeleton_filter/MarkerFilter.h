#ifndef hiros_skeleton_filter_MarkerFilter_h
#define hiros_skeleton_filter_MarkerFilter_h

// Custom external dependencies
#include "rtb/Filter/StateSpaceFilter.h"
#include "skeletons/types.h"

namespace hiros {
  namespace skeletons {

    class MarkerFilter
    {
    public:
      void filter(hiros::skeletons::types::Point& point, const double& time, const double& cutoff);

    private:
      std::array<rtb::Filter::StateSpaceFilter<double>, 3> filters_;
    };

  } // namespace skeletons
} // namespace hiros

#endif
