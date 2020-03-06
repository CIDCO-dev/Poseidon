#ifndef GOAL_HPP
#define GOAL_HPP

class Goal
{

public:

    Goal() {}

/*
    // Return type will depend on the derived class
    virtual void getGoal() const = 0;   //Pure virtual
*/

private:
    virtual void junkFunctionForPureVirtual() = 0; //Pure virtual


};

#endif
