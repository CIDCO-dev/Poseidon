#ifndef SVPPROFILE_HPP
#define SVPPROFILE_HPP





#include "Goal.hpp"

// #include "Structs.hpp"


class SVPprofile : public Goal 
{

public:

    SVPprofile() {}

    virtual ~SVPprofile() {}

    virtual void start() override {
    }

private:

    virtual void junkFunctionForPureVirtual() override {} 

};

#endif
