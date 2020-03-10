/*
* Copyright 2020 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/


#ifndef DOUBLEWITHMUTEX_H
#define DOUBLEWITHMUTEX_H

#include <mutex>


class DoubleWithMutex
{
public:
    DoubleWithMutex( const double valueIn )
    : value( valueIn )
    {
    }

    double getValue() {
        std::lock_guard<std::mutex> lock ( mutex );
        return value;
    }

    double setValue( const double valueIn ) {
        std::lock_guard<std::mutex> lock ( mutex );
        value = valueIn;
        return value;
    }

private:
    std::mutex mutex;
    double value;
};

#endif // DOUBLEWITHMUTEX_H
