#ifndef RADAR_PERCEPTION_COMPONENTS_DETECTION_CLASSIFIER_INCLUDE_DEALIASER_H_
#define RADAR_PERCEPTION_COMPONENTS_DETECTION_CLASSIFIER_INCLUDE_DEALIASER_H_

namespace measurements::radar
{
    class DetectionClassifier
    {
        public:
            DetectionClassifier(void);
            ~DetectionClassifier(void);

            void Run(void);

        private:
    };
}   // namespace measurements::radar

#endif // RADAR_PERCEPTION_COMPONENTS_DETECTION_CLASSIFIER_INCLUDE_DEALIASER_H_
