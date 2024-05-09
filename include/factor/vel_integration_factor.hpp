#ifndef MI_CALIB_VEL_INTEGRATION_FACTOR_HPP
#define MI_CALIB_VEL_INTEGRATION_FACTOR_HPP

#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"

namespace ns_mi {
    struct VelIntegrationFactor {
    private:
        Eigen::Vector3d bVec;
        Eigen::Matrix3d AMat;
        double dt;
        double weight;

    public:
        VelIntegrationFactor(Eigen::Vector3d bVec, Eigen::Matrix3d aMat, double dt, double weight)
                : bVec(std::move(bVec)), AMat(std::move(aMat)), dt(dt), weight(weight) {}

        static auto Create(const Eigen::Vector3d &bVec, const Eigen::Matrix3d &aMat, double dt, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<VelIntegrationFactor>(
                    new VelIntegrationFactor(bVec, aMat, dt, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(VelIntegrationFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ POS_BiInBr | GRAVITY | START_VEL | END_VEL ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            Eigen::Map<const Eigen::Vector3<T>> POS_BiInBr(sKnots[0]);
            Eigen::Map<const Eigen::Vector3<T>> GRAVITY(sKnots[1]);
            Eigen::Map<const Eigen::Vector3<T>> START_VEL(sKnots[2]);
            Eigen::Map<const Eigen::Vector3<T>> END_VEL(sKnots[3]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = (bVec - AMat * POS_BiInBr) - (END_VEL - START_VEL - GRAVITY * dt);
            residuals = T(weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //MI_CALIB_VEL_INTEGRATION_FACTOR_HPP
