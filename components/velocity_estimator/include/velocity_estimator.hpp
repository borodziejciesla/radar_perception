#ifndef RADAR_PERCEPTION_COMPONENTS_VELOCITY_ESTIMATOR_INCLUDE_DEALIASER_H_
#define RADAR_PERCEPTION_COMPONENTS_VELOCITY_ESTIMATOR_INCLUDE_DEALIASER_H_

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

#endif // RADAR_PERCEPTION_COMPONENTS_VELOCITY_ESTIMATOR_INCLUDE_DEALIASER_H_
