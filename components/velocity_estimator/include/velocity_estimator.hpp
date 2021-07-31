#ifndef COMPONENTS_VELOCITY_ESTIMATOR_INCLUDE_VELOCITY_ESTIMATOR_HPP_
#define COMPONENTS_VELOCITY_ESTIMATOR_INCLUDE_VELOCITY_ESTIMATOR_HPP_

namespace measurements::radar
{
    class VelocityEstimator
    {
        public:
            VelocityEstimator(void);
            ~VelocityEstimator(void);

            void Run(void);

        private:
    };
}   // namespace measurements::radar

#endif // COMPONENTS_VELOCITY_ESTIMATOR_INCLUDE_VELOCITY_ESTIMATOR_HPP_
