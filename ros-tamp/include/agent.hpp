#pragma once

//STD includes
#include <vector>
#include <string>
//ros-tamp includes
#include <state.hpp>>


class Agent
{
public:
struct Action
{
    std::string action_name;    
    std::vector<bool (*)(const State*)> preconditions;
    std::vector<void (*)()> effects;
};
//functions
virtual  std::vector<Action> getActions();
bool checkPreconditions(const State *state,const Action &action);

//vars
std::string name_;

protected:
std::vector<Action> actions_;



};


