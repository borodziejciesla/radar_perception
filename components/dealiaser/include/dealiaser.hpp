#ifndef RADAR_PERCEPTION_COMPONENTS_DEALIASER_INCLUDE_DEALIASER_H_
#define RADAR_PERCEPTION_COMPONENTS_DEALIASER_INCLUDE_DEALIASER_H_

namespace measurements::radar
{
    class Dealiaser
    {
        public:
            Dealiaser(void);
            ~Dealiaser(void);

            void Run(void);

        private:
    };
}   // namespace measurements::radar

#endif // RADAR_PERCEPTION_COMPONENTS_DEALIASER_INCLUDE_DEALIASER_H_
