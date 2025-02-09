/***************************************************/
/*             standard Header Files               */
/***************************************************/
#include <exception>
#include <algorithm>
#include <unordered_set>
#include <iostream>


/***************************************************/
/*             Project Header Files                */
/***************************************************/
#include "config.h"
#include "Dijkstra.h"
#include "PathProcessor.h"


/***************************************************/
/*              constants and enums                */
/***************************************************/
#ifdef _MSC_VER
static const char* PATH_SEP = "\\";
#else
static const char* PATH_SEP = "/";
#endif

// Some constants
enum {
    IMAGE_DIM = 2048, // Width and height of the elevation and overrides image

    ROVER_X = 159,
    ROVER_Y = 1520,
    DISTINATION_A_X = 1303,
    DISTINATION_A_Y = 85,
    DISTINATION_B_X = 1577,
    DISTINATION_B_Y = 1294
};


/***************************************************/
/*                member Functions                 */
/***************************************************/


/**
 * @brief Process the data to get shortest path between srcs and dests
 * @param elevation: elevations of the map to be processed
 * @param overrides: cell types of the map "land, river, ..." to be processed
 * @return vector of shortest pathes between sources and destinations " one path
 * between each source and its destination"
 */
std::vector<coord_set_t> Processing::process(const data_t elevation,
                                             const data_t overrides)
{
    // use bidirectional dijkstra algorithm to search the shortest path as it
    // always grantees that the path is the shortest one
    Dijkstra dijkstraAlgo(0, 0, IMAGE_DIM, IMAGE_DIM, std::move(elevation),
                          std::move(overrides));

    // the positon of the rover, bachelor and the wedding
    const coord_t rover    = std::make_pair(ROVER_X, ROVER_Y);
    const coord_t bachelor = std::make_pair(DISTINATION_A_X, DISTINATION_A_Y);
    const coord_t wedding  = std::make_pair(DISTINATION_B_X, DISTINATION_B_Y);

    // get the shortest path between rover -> bacholer, and bachelor -> wedding
    coord_set_t firstPath = dijkstraAlgo.calcShortestPath(rover, bachelor);
    coord_set_t secondPath = dijkstraAlgo.calcShortestPath(bachelor, wedding);

    // vector of all pathes between all srcs and dests
    std::vector<coord_set_t> paths;
    paths.push_back(firstPath);
    paths.push_back(secondPath);

    return paths;
}

/**
 * @brief process the data and the pathes to generate an output image
 * @param elevation: the elevation of the processed map
 * @param overrides: the cell types of the processed map "river, land, ..."
 * @param paths: the paths between all sources and destinations
 */
/*
void Processing::postProcess(const data_t& elevation, const data_t& overrides,
                             const std::vector<coord_set_t>& paths)
{
    std::ofstream of("pic.bmp", std::ofstream::binary);

    visualizer::createImage(
                of,
                &elevation[0],
            IMAGE_DIM,
            IMAGE_DIM,
            [&] (size_t x, size_t y, uint8_t elevation) {

        // Marks interesting positions on the map
        if (donut(x, y, ROVER_X, ROVER_Y) ||
                donut(x, y, DISTINATION_A_X, DISTINATION_A_Y) ||
                donut(x, y, DISTINATION_B_X, DISTINATION_B_Y))
        {
            return uint8_t(visualizer::IPV_PATH);
        }

        // Marks path positions on the map
        for(auto& path: paths)
        {
            if (path.end() != path.find(std::make_pair(x,y)))
            {
                return uint8_t(visualizer::IPV_PATH);
            }
        }

        // Signifies water
        if ((overrides[y * IMAGE_DIM + x] & (OF_WATER_BASIN | OF_RIVER_MARSH)) ||
                elevation == 0)
        {
            return uint8_t(visualizer::IPV_WATER);
        }

        // Signifies normal ground color
        if (elevation < visualizer::IPV_ELEVATION_BEGIN)
        {
            elevation = visualizer::IPV_ELEVATION_BEGIN;
        }
        return elevation;
    });
    of.flush();
#if __APPLE__
    auto res = system("open pic.bmp");
    (void)res;
#endif
}

std::ifstream::pos_type Processing::fileSize(const std::string& filename)
{
    std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
    if (!in.good())
    {
        throw std::exception();
    }
    return in.tellg();
}

data_t Processing::loadFile(const std::string& filename, size_t expectedFileSize)
{
    size_t fsize = fileSize(filename);
    if (fsize != expectedFileSize)
    {
        throw std::exception();
    }
    data_t data(fsize);
    std::ifstream ifile(filename, std::ifstream::binary);
    if (!ifile.good())
    {
        throw std::exception();
    }
    ifile.read((char*)&data[0], fsize);
    return data;
}

bool Processing::donut(int x, int y, int x1, int y1)
{
    int dx = x - x1;
    int dy = y - y1;
    int r2 = dx * dx + dy * dy;
    return r2 >= 150 && r2 <= 400;
}
*/
