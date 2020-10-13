#ifndef RANDOM_HEADER
#define RANDOM_HEADER

#include <cstdlib>
const float randf();

#endif


//#define RANDOM_HEADER_IMPLEMENTATION
#ifdef RANDOM_HEADER_IMPLEMENTATION
const float randf()
{

    return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}
#endif