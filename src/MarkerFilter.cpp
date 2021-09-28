// Internal dependencies
#include "skeleton_filter/MarkerFilter.h"

void hiros::skeletons::MarkerFilter::filter(hiros::skeletons::types::Point& point,
                                            const double& time,
                                            const double& cutoff)
{
  point.position.setX(filters_[0].filter(point.position.x(), time, cutoff));
  point.position.setY(filters_[1].filter(point.position.y(), time, cutoff));
  point.position.setZ(filters_[2].filter(point.position.z(), time, cutoff));

  point.velocity.setX(filters_[0].getFilteredFirstDerivative());
  point.velocity.setY(filters_[1].getFilteredFirstDerivative());
  point.velocity.setZ(filters_[2].getFilteredFirstDerivative());

  point.acceleration.setX(filters_[0].getFilteredSecondDerivative());
  point.acceleration.setY(filters_[1].getFilteredSecondDerivative());
  point.acceleration.setZ(filters_[2].getFilteredSecondDerivative());
}
