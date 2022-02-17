#include <agent.hpp>

bool Agent::checkPreconditions(const State *state, const Action &action) {

  for (const auto &precondition : action.preconditions)
    if (!(*precondition)(state)) {
      return false;
    }

  return true;
}
