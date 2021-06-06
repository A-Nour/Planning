/***************************************************/
/*             Project Header Files                */
/***************************************************/
#include "Path.h"

/***************************************************/
/*                member Functions                 */
/***************************************************/
/**
 * @brief constructor of the path
 * @param src : the start point of the path
 * @param dest: the destination to be reached
 */
Path::Path(coord_t src, coord_t dest)
    :m_Start(src), m_End(dest)
{
    m_WasConstructed = false;

    // insert the start to the forward search continer and the end node to the
    // backward search container and set their costs to zero
    m_FwdCellsUndrPrcs.insert(std::make_pair(0.0F, m_Start));
    m_BwdCellsUndrPrcs.insert(std::make_pair(0.0F, m_End));
}

/**
 * @brief   check if the path was constructed or not
 * @return  true if the path was constructed, false otherwise
 */
bool Path::constructed() const
{
    return m_WasConstructed;
}

/**
 * @brief set the path to be constructed
 */
void Path::setConstructed()
{
    m_WasConstructed = true;
}

/**
 * @brief   get the top cell from the forward container "the best cell with
 *          lower cost in case of forward search
 * @return  the cell with best cost in case of forward search
 */
std::pair<float, coord_t > Path::forwardTop() const
{
    return (*(m_FwdCellsUndrPrcs.begin()));
}

/**
 * @brief delete the first cell from the forward search container
 */
void Path::forwardPop()
{
    m_FwdCellsUndrPrcs.erase(m_FwdCellsUndrPrcs.begin());
}

/**
 * @brief   get the top cell from the backward container "the best cell with
 *          lower cost in case of backward search
 * @return  the cell with best cost in case of backward search
 */
std::pair<float, coord_t > Path:: backwardTop() const
{
    return (*(m_BwdCellsUndrPrcs.begin()));
}

/**
 * @brief delete the first cell from the backward search container
 */
void Path::backwardPop()
{
    m_BwdCellsUndrPrcs.erase(m_BwdCellsUndrPrcs.begin());
}

/**
 * @brief   add node the the forward search container, and check if the path was
 *          constructed.
 * @param node: the node to be added
 */
void Path::addNodeToFwd(const std::pair<float, coord_t> &&node)
{
    // add the cell
    m_FwdCellsUndrPrcs.insert(std::move(node));

    // check if the path was constructed
    m_WasConstructed = m_WasConstructed ||
            ((node.second.first == m_End.first) &&
             (node.second.second == m_End.second));
}

/**
 * @brief  add node the the backward search container, and check if the path was
 *         constructed.
 * @param node: the node to be added
 */
void Path::addNodeToBwd(const std::pair<float, coord_t> &&node)
{
    // add the cell
    m_BwdCellsUndrPrcs.insert(std::move(node));

    // check if the path was constructed
    m_WasConstructed = m_WasConstructed ||
            ((node.second.first == m_Start.first) &&
             (node.second.second == m_Start.second));
}

/**
 * @brief check if the path is blocked "no path between the start and the end"
 * @return true if the path is blocked, false otherwise
 */
bool Path::blocked() const
{
    bool result = (!m_WasConstructed) && (m_FwdCellsUndrPrcs.empty()
                                          || m_BwdCellsUndrPrcs.empty());

    return result;
}

/**
 * @brief check if the forward search container is empty
 * @return true if the forward search container is empty, false otherwise
 */
bool Path::fwdCellEmpty() const
{
    bool result = m_FwdCellsUndrPrcs.empty();

    return result;
}

/**
 * @brief check if the backward search container is empty
 * @return true if the backward search container is empty, false otherwise
 */
bool Path::bwdCellEmpty() const
{
    bool result = m_BwdCellsUndrPrcs.empty();

    return result;
}
