/***************************************************/
/*             standard Header Files               */
/***************************************************/
#include <time.h>
#include <iostream>

/***************************************************/
/*             Project Header Files                */
/***************************************************/
#include "Processing.h"


/***************************************************/
/*             Project Functions                   */
/***************************************************/
int main(int, char** argv)
{
    data_t elevation;
    data_t overrides;

    // parse the files and fill the elevation and the override containers
    Processing::preprocess(argv[0], elevation, overrides);

    // time calculation variables
    clock_t start = 0.0, end = 0.0;
    double cpuTimeUsed = 0.0;

    // start recording time
    start = clock();

    // processing of the path, and return the best paths to the targets
    auto paths = Processing::process(elevation, overrides);

    // end recording time
    end = clock();

    // cpu time in seconds
    cpuTimeUsed = ((double) (end - start)) / CLOCKS_PER_SEC;
    std::cout<<"\n\ncpu time: "<<cpuTimeUsed<<"\n";

    // construct an output image
    Processing::postProcess(elevation, overrides, paths);

    return 0;
}

