/*
* Copyright 2020 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/


#ifndef TWODOUBLESWITHMUTEX_H
#define TWODOUBLESWITHMUTEX_H

#include <mutex>


class TwoDoublesWithMutex
{
public:
    TwoDoublesWithMutex( const double firstIn, const double secondIn )
    : first( firstIn ), second( secondIn )
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

    void getValues( double & firstOut, double & secondOut ) {
        std::lock_guard<std::mutex> lock ( mutex );
        firstOut = first;
        secondOut = second;
    }

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

    void setValues( const double firstIn, const double secondIn ) {
        std::lock_guard<std::mutex> lock ( mutex );
        first = firstIn;
        second = secondIn;
    }


private:
    std::mutex mutex;
    double first;
    double second;
};

#endif // TWODOUBLESWITHMUTEX_H

