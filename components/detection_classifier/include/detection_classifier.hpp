#ifndef COMPONENTS_DETECTION_CLASSIFIER_INCLUDE_DETECTION_CLASSIFIER_HPP_
#define COMPONENTS_DETECTION_CLASSIFIER_INCLUDE_DETECTION_CLASSIFIER_HPP_

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

#endif // COMPONENTS_DETECTION_CLASSIFIER_INCLUDE_DETECTION_CLASSIFIER_HPP_
