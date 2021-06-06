/***************************************************/
/*             standard Header Files               */
/***************************************************/
#include <cmath>


/***************************************************/
/*             Project Header Files                */
/***************************************************/
#include "Dijkstra.h"


/***************************************************/
/*                member Functions                 */
/***************************************************/

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
Dijkstra::Dijkstra(int minX, int minY, int maxX, int maxY,
                   const data_t&& elevation, const data_t&& overrides)
    :m_MinX(minX), m_MinY(minY), m_MaxX(maxX), m_MaxY(maxY),
      m_Elevation(std::move(elevation)), m_Overrides(std::move(overrides))
{

}

/**
 * @brief get an init cell properties of the map
 * @return the initial cell properties of the map
 */
std::vector<std::vector<Cell>> Dijkstra::getMapInitCellsProp() const
{
    std::vector<std::vector<Cell>> nodeWeights;

    // initialize nodes weights "parent: (-1, -1), cost is max value"
    for(std::size_t row = 0; row <  m_MaxY; row++)
    {
        std::vector<Cell> tmp(m_MaxX);
        nodeWeights.push_back(std::move(tmp));
    }

    return nodeWeights;
}

/**
 * @brief   calculate the transition cost to the cell in case of forward
 *          cost "src -> cell", and the backward cost "cell -> src"
 *
 * @param src  : the source cell
 * @param cell : cell of interest
 * @return the cost of the cell in both directions "forward, backward"
 */
Cost Dijkstra::calcCellCost(const coord_t &src, const coord_t& cell) const
{
    // this is (sqrt(2) - 1) value
    const float diagCost = 0.41421356237F;
    float cost = 0.0F;

    // trans value = 2 if it is a diagonal transition, 1 otherwise
    int trans = std::abs(src.first - cell.first) +
            std::abs(src.second - cell.second);

    // cost = 1 if not diagonal, sqrt(2) if diagonal
    cost = 1 + (trans - 1)*diagCost;

    // normally elevation cost will cancel eachother, as normally the vehcile
    // will move downward the same distance which will be moved downward, but
    // here is the calculations of a simple modle
    // elevation transition cost
    const int srcIdx  = cell.second*m_MaxY + cell.first;
    const int destIdx = cell.second*m_MaxY + cell.first;

    // difference in elevation between parent of the current cell and the
    // current cell
    const int diff =  m_Elevation[destIdx] - m_Elevation[srcIdx];

    // the forward cost (parent"src" -> cell), backward cost (cell -> src)
    // the maximum boost due to elevation is 0.5 of the cost calculated due to
    // transition, and the minimum degradation is 0.5*cost
    float fwdCost = cost + cost * static_cast<float>(diff) /(2.0F*255.0F);
    float bwdCost = cost - cost * static_cast<float>(diff) /(2.0F*255.0F);

    // construct cost structure
    Cost result(fwdCost, bwdCost);

    return result;
}

/**
 * @brief   check if the cell is valid, by checking if it is a land,
 *          also check if it is between max, and min dimensions of the map
 * @param cell: the cell to be checked
 * @return : return true in case of valid cell, false otherwise
 */
bool Dijkstra::isValidCell(const coord_t& cell) const
{
    // check if the cell within the boundries of the map or not
    bool result = ((cell.first < m_MaxX) && ((cell.first >= m_MinX)));
    result = result && ((cell.second < m_MaxY) && ((cell.second >= m_MinY)));

    if(result)
    {
        // check if the cell type is not river or basin
        const int index = cell.second*m_MaxY + cell.first;
        if((m_Overrides[index]& ((OF_WATER_BASIN | OF_RIVER_MARSH))) ||
                (m_Elevation[index] == 0))
        {
            result = false;
        }

    }

    return result;
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
Dijkstra::getNeighborCells(const coord_t& parent) const
{
    std::vector<std::pair<Cost, coord_t>> neighbors;

    // iterate over all the feasible cells, and add the valid cells to the
    // neighbor container
    for(int i = -1; i <= 1; i++)
    {
        for(int j = -1; j <= 1; j++)
        {
            coord_t tmp = std::make_pair(parent.first + i, parent.second + j);
            if((!((0 == i) && (0 == j))) && isValidCell(tmp))
            {
                neighbors.push_back(constructCell(parent, tmp));
            }
        }
    }

    return neighbors;
}

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
coord_set_t
Dijkstra::traceBackPath(const std::vector<std::vector<Cell>>& cellsProp,
                        const coord_t& fwdEndCoord,
                        const coord_t& bwdEndCoord) const
{
    coord_set_t resultPath;

    // set the hub to the end of the forward search "mid point"
    auto hub = fwdEndCoord;

    // iterate on the path till reach the main parent node "src"
    while((hub.first != -1) && (hub.second != -1))
    {
        // insert the nodes to the result path
        resultPath.insert(hub);

        // get the parent of the hub
        hub = cellsProp[hub.first][hub.second].m_Parent;
    }

    // set the hub to the end of the backward search "mid point"
    hub = bwdEndCoord;

    // iterate on the path till reach the main parent node "dest"
    while((hub.first != -1) && (hub.second != -1))
    {
        // insert the nodes to the result path
        resultPath.insert(hub);

        // get the parent of the hub
        hub = cellsProp[hub.first][hub.second].m_Parent;
    }

    return resultPath;
}

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
Dijkstra::constructShortestPath(coord_t fwdEndCoord, coord_t bwdEndCoord,
                                float cost, Path& path,
                                std::vector<std::vector<Cell>>& cellsProp) const
{
    // init the minimum cost to the cost of the path found from the iterations
    float minCost = cost;
    const float minBwdCost =  path.backwardTop().first;

    // iterate over the opened nodes to be searched on the forward path search
    while(!path.fwdCellEmpty())
    {
        // get the best node WRT cost from the forward search container
        auto currBestNode = std::move(path.forwardTop());
        float distance = currBestNode.first;

        // the minimum route cost for that node
        // = the cost of the node + the minimum backward route cost +
        //   the minimum transition(0.5) "which is equal to min cell
        //   transition(1) - the max increase on speed due to elevation(0.5)"
        //   see calcCellCost function
        float minRtCost = distance + minBwdCost + 0.5F;

        // remove the cell from the cells to be searched
        path.forwardPop();

        // in case of promissing cost, check the node, otherwise, end search
        if(minRtCost < minCost)
        {
            coord_t coord = std::move(currBestNode.second);

            // get the neighbors of the best node
            auto neighbors = std::move(getNeighborCells(coord));

            // iterate over all the neighbors and calculate their transition cost,
            // then add them to the nodes to be searched
            for(auto neighbor : neighbors)
            {
                // calculate the transiton cost to the neighbor
                float neighborDist = distance + neighbor.first.m_FwdCost;

                auto neighborCoord = neighbor.second;
                const int neighborX = neighborCoord.first;
                const int neighborY = neighborCoord.second;

                // check if the path was constructed between the src and dest
                // "case of the cell was previously checked by the backward
                // iteration"
                if(cellsProp[neighborX][neighborY].m_Cost.m_BwdCost < INT_MAX)
                {
                    // calculate the path cost
                    float cost = calcCellCost(neighborCoord, coord).m_BwdCost +
                            cellsProp[coord.first][coord.second].m_Cost.m_FwdCost +
                            cellsProp[neighborX][neighborY].m_Cost.m_BwdCost;

                    // if path cost is less than the min cost, set the min cost
                    // to the current path cost, and set the mid points of the
                    // path to that of the current path
                    if(cost < minCost)
                    {
                        minCost = cost;
                        fwdEndCoord = coord;
                        bwdEndCoord = neighborCoord;
                    }
                    continue;
                }

                // if the path wasn't constructed between the src and the dest,
                // add the neighbor cell to the cells to be searched on the
                // forward search
                if(neighborDist < cellsProp[neighborX][neighborY].m_Cost.m_FwdCost)
                {
                    // set the cell cost on the cell properties
                    cellsProp[neighborX][neighborY].m_Cost.m_FwdCost = neighborDist;

                    // set the cell parent on the cell properties
                    cellsProp[neighborX][neighborY].m_Parent = std::move(coord);

                    // add the cell to the cells to be searched on the forward search
                    path.addNodeToFwd(std::make_pair(neighborDist,
                                                     std::move(neighborCoord)));
                }
            }
        }
        else
        {
            // if the rest of the nodes will not be promissing
            break;
        }
    }

    // trace back the path
    auto shortestPath = traceBackPath(cellsProp, fwdEndCoord, bwdEndCoord);

    return shortestPath;
}

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
float Dijkstra::pathSrchFwdItration(std::vector<std::vector<Cell>>& cellsProp,
                                    Path& path, coord_t& fwdEndCoord,
                                    coord_t& bwdEndCoord) const
{
    float cost = 0.0F;

    /// 1- get the first best node and remove it from the nodes to be searched
    auto currBestNode = std::move(path.forwardTop());
    path.forwardPop();

    // distance and coordinates of the best node
    float distance = currBestNode.first;
    coord_t coord = std::move(currBestNode.second);

    /// 2- get the neighbors of the best node
    auto neighbors = std::move(getNeighborCells(coord));

    /// 3- iterate over all the neighbors and calculate their transition cost,
    ///    then add them to the nodes to be searched
    for(auto neighbor : neighbors)
    {
        // calculate the transiton cost to the neighbor
        float neighborDist = distance + neighbor.first.m_FwdCost;

        auto neighborCoord = std::move(neighbor.second);
        const int neighborX = neighborCoord.first;
        const int neighborY = neighborCoord.second;

        /// 4- check if the path was constructed between the src and dest
        /// "case of the cell was previously checked by the backward iteration"
        if(cellsProp[neighborX][neighborY].m_Cost.m_BwdCost < INT_MAX)
        {
            // set path constructed
            path.setConstructed();

            // calculate the path cost
            cost = calcCellCost(neighborCoord, coord).m_BwdCost +
                    cellsProp[coord.first][coord.second].m_Cost.m_FwdCost +
                    cellsProp[neighborX][neighborY].m_Cost.m_BwdCost;

            // set the mid points of the path "the end of the forward search and
            // the end of the backward search"
            // path: src -> fwdEndCoord - bwdEndCoord -> dest
            fwdEndCoord = coord;
            bwdEndCoord = neighborCoord;

            break;
        }

        // if the path wasn't constructed between the src and the dest, add the
        // neighbor cell to the cells to be searched on the forward search
        if(neighborDist < cellsProp[neighborX][neighborY].m_Cost.m_FwdCost)
        {
            // set the cell cost on the cell properties
            cellsProp[neighborX][neighborY].m_Cost.m_FwdCost = neighborDist;

            // set the cell parent on the cell properties
            cellsProp[neighborX][neighborY].m_Parent = std::move(coord);

            // add the cell to the cells to be searched on the forward search
            path.addNodeToFwd(std::make_pair(neighborDist,
                                             std::move(neighborCoord)));
        }
    }

    // return the cost in case of the path was constructed, 0 OW
    return cost;
}

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
float Dijkstra::pathSrchBwdItration(std::vector<std::vector<Cell>>& cellsProp,
                                    Path& path, coord_t &fwdEndCoord,
                                    coord_t &bwdEndCoord) const
{
    float cost = 0.0F;
    /// 1- get the first best node and remove it from the nodes to be searched
    auto currBestNode = std::move(path.backwardTop());
    path.backwardPop();

    // distance and coordinates of the best node
    float distance = currBestNode.first;
    auto coord = std::move(currBestNode.second);

    /// 2- get the neighbors of the best node
    auto neighbors = std::move(getNeighborCells(coord));

    /// 3- iterate over all the neighbors and calculate their transition cost,
    ///    then add them to the nodes to be searched
    for(auto& neighbor : neighbors)
    {
        // calculate the transiton cost to the neighbor
        const float neighborDist = distance + neighbor.first.m_BwdCost;

        auto neighborCoord = std::move(neighbor.second);
        const int neighborX = neighborCoord.first;
        const int neighborY = neighborCoord.second;

        /// 4- check if the path was constructed between the src and dest
        /// "case of the cell was previously checked by the forward iteration"
        if(cellsProp[coord.first][coord.second].m_Cost.m_FwdCost < INT_MAX)
        {
            // set path constructed
            path.setConstructed();

            // calculate the path cost
            cost = calcCellCost(neighborCoord, coord).m_FwdCost +
                    cellsProp[coord.first][coord.second].m_Cost.m_BwdCost +
                    cellsProp[neighborX][neighborY].m_Cost.m_FwdCost;

            // set the mid points of the path "the end of the forward search and
            // the end of the backward search"
            // path: src -> fwdEndCoord - bwdEndCoord -> dest
            fwdEndCoord = coord;
            bwdEndCoord = neighborCoord;

            break;
        }

        // if the path wasn't constructed between the src and the dest, add the
        // neighbor cell to the cells to be searched on the backward search
        if(neighborDist < cellsProp[neighborX][neighborY].m_Cost.m_BwdCost)
        {
            // set the cell cost on the cell properties
            cellsProp[neighborX][neighborY].m_Cost.m_BwdCost = neighborDist;

            // set the cell parent on the cell properties
            cellsProp[neighborX][neighborY].m_Parent = coord;

            // add the cell to the cells to be searched on the backward search
            path.addNodeToBwd(std::make_pair(neighborDist,
                                             std::move(neighborCoord)));
        }
    }

    // return the cost in case of the path was constructed, 0 OW
    return cost;
}

/**
 * @brief   get the shortest path between src and destination using
 *          bidirectional dijkstra algo., in a given map
 *
 * @param src  : the start point coordinates on the path
 * @param dest : the end point coordinates on the path
 * @return the shotest path between the src and the distination
 */
coord_set_t Dijkstra::calcShortestPath(coord_t src, coord_t dest) const
{
    // init the end coordinates of the forward search and the beackward search
    coord_t fwdEndCoord = dest;
    coord_t bwdEndCoord = src;

    // initialize a path between src and dest
    Path path(src, dest);

    // the result path to be filled when the path between src and destination
    // is constructed
    coord_set_t resultPath;

    // initialize the properties of the map "parent (-1,-1), cost: max value"
    auto cellsProp = std::move(getMapInitCellsProp());

    float cost = 0.0F;

    // set the src and dest cost to 0
    cellsProp[src.first][src.second].m_Cost.m_FwdCost = 0;
    cellsProp[dest.first][dest.second].m_Cost.m_BwdCost = 0;

    // in case of the path wasn't constructed and the path wasn't blocked
    // iterate once on the forward direction and then once on the backward dir
    while(!(path.blocked() || path.constructed()))
    {
        // make dijkstra forward direction iteration, cost is the cost of the
        // path in case its constructed, 0 OW
        cost = pathSrchFwdItration(cellsProp, path, fwdEndCoord, bwdEndCoord);

        if(!path.constructed())
        {
            // make dijkstra backward direction iteration, cost is the cost of
            // the path in case its constructed, 0 OW
            cost = pathSrchBwdItration(cellsProp, path, fwdEndCoord, bwdEndCoord);
        }
    }

    if(path.constructed())
    {
        // check if there is any other better pathes
        resultPath = constructShortestPath(fwdEndCoord, bwdEndCoord, cost, path,
                                           cellsProp);
    }

    return resultPath;
}
