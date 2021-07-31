#ifndef RADAR_PERCEPTION_COMPONENTS_DEALIASER_INCLUDE_DEALIASER_H_
#define RADAR_PERCEPTION_COMPONENTS_DEALIASER_INCLUDE_DEALIASER_H_

#include "radar_scan.hpp"

namespace measurements::radar
{
    class Dealiaser
    {
        public:
            Dealiaser(void);
            ~Dealiaser(void);

            void Run(RadarScan & radar_scan);

        private:
    };
}   // namespace measurements::radar

#endif // RADAR_PERCEPTION_COMPONENTS_DEALIASER_INCLUDE_DEALIASER_H_
