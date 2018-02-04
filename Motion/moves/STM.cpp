#include "STM.h"

void STM::setState(std::string state_)
{
    if (state != state_) {
        exitState(state);
        state = state_;
        enterState(state);
    }
}

std::string STM::getStatus() const
{
  if (isRunning()) {
    return state;
  } else {
    return "-";
  }
}

void STM::enterState(std::string state)
{
}

void STM::exitState(std::string state)
{
}
