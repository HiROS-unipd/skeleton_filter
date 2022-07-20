#ifndef hiros_skeleton_filter_KinematicStateFilter_h
#define hiros_skeleton_filter_KinematicStateFilter_h

// Custom external dependencies
#include "rtb/Filter/Filter.h"
#include "rtb/Filter/StateSpaceFilter.h"
#include "skeletons/types.h"

namespace hiros {
  namespace skeletons {

    enum class Type
    {
      StateSpace,
      Butterworth,
      Undefined
    };

    class KinematicStateFilter
    {
    public:
      KinematicStateFilter() {}
      KinematicStateFilter(const Type& filter_type,
                           const double& cutoff_frequency,
                           const double& sample_frequency = 0,
                           const unsigned int& butterworth_order = 0);
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

      Type filter_type_{Type::Undefined};
      double cutoff_frequency_{};
      double sample_frequency_{};
      unsigned int butterworth_order_{};

      std::array<rtb::Filter::StateSpaceFilter<double>, 3> statespace_filters_;
      std::array<rtb::Filter::Filter<double>, 3> butterworth_filters_;

      double last_time_{std::numeric_limits<double>::quiet_NaN()};
      hiros::skeletons::types::KinematicState last_state_{};
    };

  } // namespace skeletons
} // namespace hiros

#endif
