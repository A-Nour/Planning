#ifndef CONFIG_H
#define CONFIG_H
/***************************************************/
/*             standard Header Files               */
/***************************************************/
#include <set>
#include <vector>
#include <cstdint>

/***************************************************/
/*                    Flags                        */
/***************************************************/
// Bits used in the overrides image bytes
enum OverrideFlags
{
    OF_RIVER_MARSH = 0x10,
    OF_INLAND = 0x20,
    OF_WATER_BASIN = 0x40
};

/***************************************************/
/*                 Type alias                      */
/***************************************************/
using coord_t     = std::pair<int, int>;
using coord_set_t = std::set<coord_t>;
using data_t      = std::vector<uint8_t>;

#endif
