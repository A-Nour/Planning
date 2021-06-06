#ifndef PATH_H
#define PATH_H

/***************************************************/
/*             Project Header Files                */
/***************************************************/
#include "config.h"

/***************************************************/
/*             Class Decleration                   */
/***************************************************/
class Path
{
public:
    /**
     * @brief constructor of the path
     * @param src : the start point of the path
     * @param dest: the destination to be reached
     */
    Path(coord_t src, coord_t dest);

    /**
     * @brief   get the top cell from the forward container "the best cell with
     *          lower cost in case of forward search
     * @return  the cell with best cost in case of forward search
     */
    std::pair<float, coord_t> forwardTop() const;

    /**
     * @brief delete the first cell from the forward search container
     */
    void forwardPop();

    /**
     * @brief   get the top cell from the backward container "the best cell with
     *          lower cost in case of backward search
     * @return  the cell with best cost in case of backward search
     */
    std::pair<float, coord_t> backwardTop() const;

    /**
     * @brief delete the first cell from the backward search container
     */
    void backwardPop();

    /**
     * @brief   add node the the forward search container, and check if the path
     *          was constructed.
     * @param node: the node to be added
     */
    void addNodeToFwd(const std::pair<float, coord_t>&& node);

    /**
     * @brief  add node the the backward search container, and check if the path
     *         was constructed.
     * @param node: the node to be added
     */
    void addNodeToBwd(const std::pair<float, coord_t>&& node);

    /**
     * @brief check if the path is blocked "no path between the start and the end"
     * @return true if the path is blocked, false otherwise
     */
    bool blocked() const;

    /**
     * @brief   check if the path was constructed or not
     * @return  true if the path was constructed, false otherwise
     */
    bool constructed() const;

    /**
     * @brief set the path to be constructed
     */
    void setConstructed();

    /**
     * @brief check if the forward search container is empty
     * @return true if the forward search container is empty, false otherwise
     */
    bool fwdCellEmpty() const;

    /**
     * @brief check if the backward search container is empty
     * @return true if the backward search container is empty, false otherwise
     */
    bool bwdCellEmpty() const;
protected:
    // start coordinates of the path
    coord_t m_Start;

    // end coordinates of the path
    coord_t m_End;

    // container used in case of forward search, which holds the nodes
    // coordinates in order with respect to their transition cost from start
    // to the current node "best first"
    std::set<std::pair<float, coord_t>> m_FwdCellsUndrPrcs;

    // container used in case of backward search, which holds the nodes
    // coordinates in order with respect to their transition cost from end
    // to the current node "best first"
    std::set<std::pair<float, coord_t>> m_BwdCellsUndrPrcs;

    // flag which indicates if the path was constructed or not
    bool m_WasConstructed;
};

#endif // PATH_H
