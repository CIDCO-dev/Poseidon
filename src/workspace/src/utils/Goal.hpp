#ifndef GOAL_HPP
#define GOAL_HPP

class Goal
{

public:

    Goal() {}

    virtual ~Goal() {};

    virtual void start() = 0;


    virtual bool execute( const double currentLatitude, 
                    const double currentLongitude ) = 0; //Pure virtual

private:



};

#endif
