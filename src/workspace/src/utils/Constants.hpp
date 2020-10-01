#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#ifdef _WIN32
#define M_PI 3.14159265358979323846
#endif
#define PI M_PI

#define R2D(x) (x * ((double)180/(double)PI))
#define D2R(x) (x * ((double)PI/(double)180))


#endif