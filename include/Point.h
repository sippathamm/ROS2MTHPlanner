//
// Created by Sippawit Thammawiset on 14/1/2024 AD.
//

#ifndef POINT_H
#define POINT_H

#include <cmath>

namespace MTH
{
    typedef struct APoint
    {
    public:
        APoint () : X(0.0f), Y(0.0f) {}

        APoint (double X, double Y) : X(X), Y(Y) {}

        APoint (const APoint &AnotherPoint)
        {
            X = AnotherPoint.X;
            Y = AnotherPoint.Y;
        }

        double X;
        double Y;

        APoint operator + (const APoint &AnotherPoint) const
        {
            return {X + AnotherPoint.X, Y + AnotherPoint.Y};
        }

        APoint operator - (const APoint &AnotherPoint) const
        {
            return {X - AnotherPoint.X, Y - AnotherPoint.Y};
        }

        APoint operator * (const APoint &AnotherPoint) const
        {
            return {X * AnotherPoint.X, Y * AnotherPoint.Y};
        }

        APoint operator * (double Constant) const
        {
            return {X * Constant, Y * Constant};
        }

        APoint operator / (double Constant) const
        {
            return {X / Constant, Y / Constant};
        }

        bool operator == (const APoint &AnotherPoint) const
        {
            return (X == AnotherPoint.X) && (Y == AnotherPoint.Y);
        }

        bool operator != (const APoint &AnotherPoint) const
        {
            return !(*this == AnotherPoint);
        }
    } APoint;

    APoint Absolute (const APoint &Point)
    {
        return {std::abs(Point.X), std::abs(Point.Y)};
    }
}

#endif //POINT_H