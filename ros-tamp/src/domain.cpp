#include <domain.hpp>
std::map<std::string,std::vector<Action>> Domain::getValidActions(const State *state) {
  std::map<std::string, std::vector<Action>> valid_actions;
  for (auto &agent : agents) {
      for (auto &action: agent.getActions()){
         if (agent.checkPreconditions(state,action)){
             valid_actions.insert(std::make_pair<std::string,)
         }
      }
  }
}
