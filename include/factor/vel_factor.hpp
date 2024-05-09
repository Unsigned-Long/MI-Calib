#ifndef MI_CALIB_VEL_FACTOR_HPP
#define MI_CALIB_VEL_FACTOR_HPP

#include <utility>

#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"

namespace ns_mi {
    template<int Order>
    struct LinVelFactor {
    private:
        ns_ctraj::SplineMeta<Order> _splineMeta;

        double _time;
        Eigen::Vector3d _vel;
        double _dtInv;
        double _weight;

    public:
        LinVelFactor(ns_ctraj::SplineMeta<Order> splineMeta, double time, Eigen::Vector3d velocity, double weight)
                : _splineMeta(std::move(splineMeta)), _time(time), _vel(std::move(velocity)),
                  _dtInv(1.0 / _splineMeta.segments.front().dt), _weight(weight) {}

        static auto Create(const ns_ctraj::SplineMeta<Order> &splineMeta, double time,
                           const Eigen::Vector3d &velocity, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<LinVelFactor>(
                    new LinVelFactor(splineMeta, time, velocity, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(LinVelFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ VEL | ... | VEL ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            // array offset
            std::size_t POS_OFFSET;
            double u;
            _splineMeta.template ComputeSplineIndex(_time, POS_OFFSET, u);

            Eigen::Vector3<T> pred_LIN_VEL_BrToBr0InBr0;

            // Pretend this is a velocity spline
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    sKnots + POS_OFFSET, u, _dtInv, &pred_LIN_VEL_BrToBr0InBr0
            );

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);

            residuals = pred_LIN_VEL_BrToBr0InBr0 - _vel.template cast<T>();

            residuals = T(_weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //MI_CALIB_VEL_FACTOR_HPP
