/*
* Copyright 2020 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/


#ifndef TWODOUBLESROSTIMEWITHMUTEX_H
#define TWODOUBLESROSTIMEWITHMUTEX_H

#include <mutex>

#include "ros/time.h"

class TwoDoublesRosTimeWithMutex
{
public:
    TwoDoublesRosTimeWithMutex( const double firstIn, const double secondIn, 
                                const ros::Time timeIn )
    : first( firstIn ), second( secondIn ), time( timeIn.sec, timeIn.nsec )
    {
    }

    double getFirst() {
        std::lock_guard<std::mutex> lock ( mutex );
        return first;
    }

    double getSecond() {
        std::lock_guard<std::mutex> lock ( mutex );
        return second;
    }

    ros::Time getTime() {
        std::lock_guard<std::mutex> lock ( mutex );
        return time;
    }

    void getValues( double & firstOut, double & secondOut, ros::Time & timeOut ) {
        std::lock_guard<std::mutex> lock ( mutex );
        firstOut = first;
        secondOut = second;
        timeOut = time;
    }

/*    void getDoubles( double & firstOut, double & secondOut) {
        std::lock_guard<std::mutex> lock ( mutex );
        firstOut = first;
        secondOut = second;
    }
*/

    double setFirst( const double valueIn ) {
        std::lock_guard<std::mutex> lock ( mutex );
        first = valueIn;
        return first;
    }

    double setSecond( const double valueIn ) {
        std::lock_guard<std::mutex> lock ( mutex );
        second = valueIn;
        return second;
    }

    double setTime( const ros::Time timeIn ) {
        std::lock_guard<std::mutex> lock ( mutex );
        time = timeIn;
        return second;
    }

    void setValues( const double firstIn, const double secondIn,
                    const ros::Time timeIn ) {
        std::lock_guard<std::mutex> lock ( mutex );
        first = firstIn;
        second = secondIn;
        time = timeIn;
    }

    bool isTimeZero() {
        std::lock_guard<std::mutex> lock ( mutex );
        return time.sec == 0 && time.nsec == 0;
    }

private:
    std::mutex mutex;
    double first;
    double second;

    ros::Time time;
};

#endif // TWODOUBLESROSTIMEWITHMUTEX_H

