#ifndef DIJKSTRA_H
#define DIJKSTRA_H
/***************************************************/
/*             standard Header Files               */
/***************************************************/
#include <climits>


/***************************************************/
/*             Project Header Files                */
/***************************************************/
#include "config.h"
#include "Path.h"


/***************************************************/
/*           User Defined Data Types               */
/***************************************************/
struct Cost
{
    /* structure that holds the cost of transition to the cell in both forward
       and backward direction */
    Cost()
    {
        // init the cost to be max number "infinity"
        m_FwdCost = static_cast<float>(INT_MAX);
        m_BwdCost = static_cast<float>(INT_MAX);
    }

    /**
     * constructor of the structure
     *
     * @param fwdCost : the forward transition cost to the cell "from the src
     *                  to the cell"
     * @param bwdCost : the backward transition cost to the cell "from the dest
     *                  to the cell"
     */
    Cost(float fwdCost, float bwdCost)
        :m_FwdCost(fwdCost), m_BwdCost(bwdCost)
    {

    }

    // forward cost to the cell "from src to the cell", initial with max number
    float m_FwdCost;

    // backward cost to the cell "from dest to the cell", init with max number
    float m_BwdCost;
};

struct Cell
{
    // structure that holds the properties of the map cell "its parent on the
    // path, and its cost"
    Cell()
    {
        // init the parent coordinates to be -1, -1
        m_Parent = std::make_pair(-1, -1);
    }

    // if we have 9 cell like that, 6 in case of edges or 4 in case of corners
    //    c c c                        c c c                          c c|
    //    c p c    ,in case of edges   c p c    ,in case of corners   c p|
    //    c c c                      ---------                      ------
    // the parent is the p cell
    // this variable holds the coordinates of the parent cell
    coord_t m_Parent;

    // the cost of the cell "forward _src -> cell_ and backward  _dest -> cell_"
    Cost m_Cost;
};


/***************************************************/
/*             Class Decleration                   */
/***************************************************/
class Dijkstra
{
    /* This class is doing bidirectional dijkstra search */
public:
    /**
     * constructor of the class
     *
     * @param minX : the min x coordinates of the map to be searched
     * @param minY : the min y coordinates of the map to be searched
     * @param maxX : the max x coordinates of the map to be searched
     * @param maxY : the max y coordinates of the map to be searched
     * @param elevation : the elevations of cells of the map to be searched
     * @param overrides : the cell types of the processed map "river, land, ..."
     */
    Dijkstra(int minX, int minY, int maxX, int maxY,
             const data_t &&elevation, const data_t &&overrides);

    /**
     * @brief   get the shortest path between src and destination using
     *          bidirectional dijkstra algo., in a given map
     *
     * @param src  : the start point coordinates on the path
     * @param dest : the end point coordinates on the path
     * @return the shotest path between the src and the distination
     */
    coord_set_t calcShortestPath(coord_t src, coord_t dest) const;
protected:

    /**
     * @brief get an init cell properties of the map
     * @return the initial cell properties of the map
     */
    std::vector<std::vector<Cell>> getMapInitCellsProp() const;

    /**
     * @brief   calculate the transition cost to the cell in case of forward
     *          cost "src -> cell", and the backward cost "cell -> src"
     *
     * @param src  : the source cell
     * @param cell : cell of interest
     * @return the cost of the cell in both directions "forward, backward"
     */
    Cost calcCellCost(const coord_t& src, const coord_t& cell) const;

    /**
     * @brief construct the cell by setting its cost and coordinates
     * @param src  : the parent cell to the cell of interest on a given path
     * @param cell : the cell of interest
     * @return pair of the cost of the cell of interest and its coordinates
     */
    inline std::pair<Cost, coord_t> constructCell(const coord_t& src,
                                                  const coord_t& cell) const
    {
        // calculate the cell cost
        Cost cost = calcCellCost(src, cell);

        return std::make_pair(cost, cell);
    }

    /**
     * @brief get the eight neighbors to a given cell "parent p"
     *                          c c c
     *                          c p c
     *                          c c c
     * @param parent: the parent cell "p in the sketch"
     * @return return in normal eight neighbor cells "differs in case of edges"
     */
    std::vector<std::pair<Cost, coord_t>>
    getNeighborCells(const coord_t& parent) const;

    /**
     * @brief   check if the cell is valid, by checking if it is a land,
     *          also check if it is between max, and min dimensions
     * @param cell: the cell to be checked
     * @return : return true in case of valid cell, false otherwise
     */
    bool isValidCell(const coord_t& cell) const;

    /**
     * @brief   make an iteration of searching of the path on the forward
     *          "src -> dest" direction for the dijkstra algorithm
     *
     * @param cellsProp     : the cell properties of the map "cost of each cell,
     *                        and its parent"
     * @param path          : the path to be constructed
     * @param fwdEndCoord   : the end coordinates of the forward iterations
     *                        "either the common cell or the cell before it" in
     *                        case of bidirectional search "src -> common cell"
     * @param bwdEndCoord   : the end coordinates of the forward iterations
     *                        "either the common cell or the cell after it" in
     *                        case of bidirectional search "src -> common cell"
     * @return              : the cost of the path in case of the path was
     *                        constructed, 0 OW
     */
    float pathSrchBwdItration(std::vector<std::vector<Cell>>& cellsProp,
                              Path& path, coord_t& fwdEndCoord,
                              coord_t& bwdEndCoord) const;

    /**
     * @brief   make an iteration of searching of the path on the backward
     *          "dest -> src" direction for the dijkstra algorithm
     *
     * @param cellsProp     : the cell properties of the map "cost of each cell,
     *                        and its parent"
     * @param path          : the path to be constructed
     * @param fwdEndCoord   : the end coordinates of the forward iterations
     *                        "either the common cell or the cell before it" in
     *                        case of bidirectional search "src -> common cell"
     * @param bwdEndCoord   : the end coordinates of the forward iterations
     *                        "either the common cell or the cell after it" in
     *                        case of bidirectional search "src -> common cell"
     * @return              : the cost of the path in case of the path was
     *                        constructed, 0 OW
     */
    float pathSrchFwdItration(std::vector<std::vector<Cell>>& cellsProp,
                              Path& path, coord_t& fwdEndCoord,
                              coord_t& bwdEndCoord) const;

    /**
     * @brief reconstruct the trace between the src and the destination
     * @param fwdEndCoord : the mid point which indicates the end of the forward
     *                      search "src -> fwdEndCoord"
     * @param bwdEndCoord : the mid point which indicates the end of the
     *                      backward search "bwdEndCoord -> dest"
     *
     *                      path: src -> fwdEndCoord - bwdEndCoord -> dest
     *
     * @param cellsProp   : the cell properties of the map "cost of each cell,
     *                      and its parent"
     * @return  set of the coordinates of the path sorted with respect to the
     *          coordinates
     */
    coord_set_t traceBackPath(const std::vector<std::vector<Cell>>& cellsProp,
                              const coord_t& fwdEndCoord,
                              const coord_t& bwdEndCoord) const;

    /**
     * @brief   after the path was constructed between the source and the
     *          the destination of a given path, make another search on all
     *          possible paths that may be shorter than the constructed path,
     *          this is the stopping criteria of the bidirectional search.
     * @param fwdEndCoord : the mid point which indicates the end of the forward
     *                      search "src -> fwdEndCoord"
     * @param bwdEndCoord : the mid point which indicates the end of the
     *                      backward search "bwdEndCoord -> dest"
     *
     *                      path: src -> fwdEndCoord - bwdEndCoord -> dest
     *
     * @param cost        : the cost of the previously found path using the
     *                      normal iterations of bidirectional dijekstra
     * @param path        : the constructed path between the src and the dest
     * @param cellsProp   : the cell properties of the map "cost of each cell,
     *                      and its parent"
     *
     * @return  set of the coordinates of the path sorted with respect to the
     *          coordinates
     */
    coord_set_t
    constructShortestPath(coord_t fwdEndCoord, coord_t bwdEndCoord, float cost,
                          Path &path,
                          std::vector<std::vector<Cell>> &cellsProp) const;
private:
    // map boundries
    const int m_MinX;
    const int m_MinY;
    const int m_MaxX;
    const int m_MaxY;

    // the elevations of cells of the map to be searched
    const data_t m_Elevation;
    // the cell types of the processed map "river, land, ..."
    const data_t m_Overrides;
};

#endif // DIJKSTRA_H
