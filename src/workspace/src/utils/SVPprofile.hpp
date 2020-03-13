#ifndef SVPPROFILE_HPP
#define SVPPROFILE_HPP





#include "Goal.hpp"


class SVPprofile : public Goal 
{

public:

    SVPprofile() {}

    virtual ~SVPprofile() {}

    virtual void start() override {
    }

    virtual bool execute( const double currentLatitude, 
                    const double currentLongitude ) override {
    }

private:


};

#endif
