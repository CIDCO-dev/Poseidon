/*
* Copyright 2020 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/


#ifndef BOOLWITHMUTEX_H
#define BOOLWITHMUTEX_H

#include <mutex>


class BoolWithMutex
{
public:
    BoolWithMutex( const bool valueIn )
    : value( valueIn )
    {
    }

    bool getValue() {
        std::lock_guard<std::mutex> lock ( mutex );
        return value;
    }

    bool setValue( const bool valueIn ) {
        std::lock_guard<std::mutex> lock ( mutex );
        value = valueIn;
        return value;
    }

private:
    std::mutex mutex;
    bool value;
};

#endif // BOOLWITHMUTEX_H
