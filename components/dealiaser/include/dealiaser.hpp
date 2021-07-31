#ifndef COMPONENTS_DEALIASER_INCLUDE_DEALIASER_HPP_
#define COMPONENTS_DEALIASER_INCLUDE_DEALIASER_HPP_

#include "radar_scan.hpp"

#include <vector>

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

#endif // COMPONENTS_DEALIASER_INCLUDE_DEALIASER_HPP_
