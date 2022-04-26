#ifndef hiros_skeleton_filter_KinematicStateFilter_h
#define hiros_skeleton_filter_KinematicStateFilter_h

// Custom external dependencies
#include "rtb/Filter/StateSpaceFilter.h"
#include "skeletons/types.h"

namespace hiros {
  namespace skeletons {

    class KinematicStateFilter
    {
    public:
      KinematicStateFilter() {}
      KinematicStateFilter(const double& cutoff_frequency);
      ~KinematicStateFilter() {}

      void filter(hiros::skeletons::types::KinematicState& state, const double& time);

    private:
      void computePosition(hiros::skeletons::types::KinematicState& state, const double& time);
      void computeLinearVelocity(hiros::skeletons::types::KinematicState& state);
      void computeLinearAcceleration(hiros::skeletons::types::KinematicState& state);

      void computeOrientation(hiros::skeletons::types::KinematicState& state, const double& time);
      void computeAngularVelocity(hiros::skeletons::types::KinematicState& state,
                                  const hiros::skeletons::types::KinematicState& prev_state,
                                  const double& prev_time);
      void computeAngularAcceleration(hiros::skeletons::types::KinematicState& state,
                                      const hiros::skeletons::types::KinematicState& prev_state,
                                      const double& prev_time);

      double cutoff_frequency_{};
      std::array<rtb::Filter::StateSpaceFilter<double>, 3> pos_filters_;

      double last_time_{std::numeric_limits<double>::quiet_NaN()};
      hiros::skeletons::types::KinematicState last_state_{};
    };

  } // namespace skeletons
} // namespace hiros

#endif
