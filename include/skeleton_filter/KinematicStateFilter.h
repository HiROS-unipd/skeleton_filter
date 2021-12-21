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
      void filter(hiros::skeletons::types::KinematicState& state, const double& time, const double& cutoff);

    private:
      void computePosition(hiros::skeletons::types::KinematicState& state, const double& time, const double& cutoff);
      void computeLinearVelocity(hiros::skeletons::types::KinematicState& state);
      void computeLinearAcceleration(hiros::skeletons::types::KinematicState& state);

      void computeOrientation(hiros::skeletons::types::KinematicState& state, const double& time, const double& cutoff);
      void computeAngularVelocity(hiros::skeletons::types::KinematicState& state,
                                  const hiros::skeletons::types::KinematicState& prev_state,
                                  const double& prev_time,
                                  const double& cutoff);
      void computeAngularAcceleration(hiros::skeletons::types::KinematicState& state,
                                      const hiros::skeletons::types::KinematicState& prev_state,
                                      const double& prev_time,
                                      const double& cutoff);

      std::array<rtb::Filter::StateSpaceFilter<double>, 3> pos_filters_;

      double m_last_time{std::numeric_limits<double>::quiet_NaN()};
      hiros::skeletons::types::KinematicState m_last_state{};
    };

  } // namespace skeletons
} // namespace hiros

#endif
