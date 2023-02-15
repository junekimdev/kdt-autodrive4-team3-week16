#ifndef CONTROL_STATE_H
#define CONTROL_STATE_H

#include <string>
#include <vector>

namespace control_state
{

constexpr int16_t ANGLE_CENTER = 0;
constexpr int16_t ANGLE_MIN = -50;
constexpr int16_t ANGLE_MAX = 50;

enum struct Mode
{
  Init,
  Left,
  LeftPrepToStop,
  LeftStop,
  Right,
  RightPrepToStop,
  RightStop
};

struct State
{
  Mode mode;
  float speed;
  int16_t angle;
  bool started;

  State() : mode(Mode::Init), speed(0), angle(ANGLE_CENTER), started(false)
  {
  }

  void reduce(Mode mode, int angle, float speed){
    this->mode = mode;
    this->angle = angle;
    this->speed = speed;
  }
};

}  // namespace control

#endif
