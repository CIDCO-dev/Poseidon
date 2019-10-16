#include <stdint.h>

struct Position {
  uint64_t timeStamp;
  double x;
  double y;
  double z;
};

struct Imu {
  uint64_t timeStamp;
  double w;
  double x;
  double y;
  double z;
};

struct Sonar {
  uint64_t timeStamp;
  double depth;
};

