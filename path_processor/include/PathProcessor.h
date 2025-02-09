#ifndef PROCESSING_H
#define PROCESSING_H
/***************************************************/
/*             standard Header Files               */
/***************************************************/
#include <fstream>
#include <string>
#include <vector>

/***************************************************/
/*             Project Header Files                */
/***************************************************/
#include "config.h"


/***************************************************/
/*             Class Decleration                   */
/***************************************************/
class Processing
{
public:
    /**
     * @brief parsing the data files
     * @param pname: project location
     * @param elevation: elevations of the cells of the map
     * @param overrides: cell types of the map "river, land, ..."
     */
    static void preprocess(std::string pname, data_t& elevation, data_t& overrides);

    /**
     * @brief Process the data to get shortest path between srcs and dests
     * @param elevation: elevations of the map to be processed
     * @param overrides: cell types of the map "land, river, ..." to be processed
     * @return vector of shortest pathes between sources and destinations " one path
     * between each source and its destination"
     */
    static std::vector<coord_set_t> process(const data_t elevation,
                                                  const data_t overrides);


protected:
//    static std::ifstream::pos_type fileSize(const std::string &filename);
//    static data_t loadFile(const std::string &filename, size_t expectedFileSize);
//    static bool donut(int x, int y, int x1, int y1);
};



#endif // PROCESSING_H
