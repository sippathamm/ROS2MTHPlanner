//
// Created by Sippawit Thammawiset on 7/2/2024 AD.
//

#ifndef DEBUG_H
#define DEBUG_H

#include "Point.h"

#include <ostream>

std::ostream& operator << (std::ostream &OS, const std::vector<MTH::APoint> &Position)
{
    OS << "[";
    for (const auto &i : Position)
    {
        OS << "(" << i.X << ", " << i.Y << "), ";
    }
    OS << "]";

    return OS;
}

std::ostream& operator << (std::ostream &OS, const MTH::APoint &Point)
{
    OS << "(" << Point.X << ", " << Point.Y << ")";

    return OS;
}

#endif // DEBUG_H
