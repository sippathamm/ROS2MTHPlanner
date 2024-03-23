//
// Created by Sippawit Thammawiset on 17/2/2024 AD.
//

#ifndef BASE_PLANNER_H
#define BASE_PLANNER_H

#include "Debug.h"
#include "Utility.h"
#include "Point.h"
#include "spline.h"

#include <iostream>
#include <vector>
#include <random>
#include <cmath>

namespace MTH
{
    class ABasePlanner
    {
    public:
        ABasePlanner (const APoint &LowerBound, const APoint &UpperBound,
                      int MaximumIteration, int NPopulation, int NBreakpoint, int NWaypoint,
                      TRAJECTORY_TYPE TrajectoryType = TRAJECTORY::CUBIC_SPLINE) :
                      LowerBound_(LowerBound),
                      UpperBound_(UpperBound),
                      MaximumIteration_(MaximumIteration),
                      NPopulation_(NPopulation),
                      NBreakpoint_(NBreakpoint),
                      NWaypoint_(NWaypoint),
                      TrajectoryType_(TrajectoryType)
        {

        }

        virtual ~ABasePlanner() = default;

        virtual bool CreatePlan (const unsigned char *CostMap,
                                 const APoint &Start,
                                 const APoint &Goal,
                                 std::vector<APoint> &Path) = 0;

        std::vector<APoint> GetGlobalBestPosition () const
        {
            return this->GlobalBestPosition_;
        }

        double GetGlobalBestCost () const
        {
            return this->GlobalBestCost_;
        }

        double GetPathLength () const
        {
            return this->PathLength_;
        }

        virtual void Clear ()  = 0;

    protected:
        APoint LowerBound_, UpperBound_;
        int MaximumIteration_, NPopulation_, NBreakpoint_, NWaypoint_;

        const unsigned char *CostMap_ = nullptr;
        const APoint *Start_ = nullptr;
        const APoint *Goal_ = nullptr;

        float ObstacleCostFactor_ = 1.0f;
        float CostScalingFactor_ = 1.0f;
        float PenaltyScalingFactor_ = 1000.0f;

        APoint Range_;
        int N_{};

        std::vector<APoint> GlobalBestPosition_;
        double GlobalBestCost_ = (double) INFINITY;

        double PathLength_ = 0.0f;

        TRAJECTORY_TYPE TrajectoryType_;

        APoint GenerateDistributedPosition ()
        {
            APoint RandomPosition;

            RandomPosition.X = GenerateRandom(this->LowerBound_.X, this->UpperBound_.X);
            RandomPosition.Y = GenerateRandom(this->LowerBound_.Y, this->UpperBound_.Y);

            return RandomPosition;
        }

        APoint GenerateCircularPosition ()
        {
            static double Radius = std::hypot(this->Start_->X - this->Goal_->X,
                                              this->Start_->Y - this->Goal_->Y) / 2.0f;
            static APoint Center = (*this->Goal_ + *this->Start_) / 2.0f;

            double Angle = GenerateRandom(0.0f, 2.0f * M_PI);
            double R = std::sqrt(GenerateRandom(0.0f, 1.0f)) * Radius;

            APoint RandomPosition;
            RandomPosition.X = Center.X + R * std::cos(Angle);
            RandomPosition.Y = Center.Y + R * std::sin(Angle);

            RandomPosition.X = CLAMP(RandomPosition.X, this->LowerBound_.X, this->UpperBound_.X);
            RandomPosition.Y = CLAMP(RandomPosition.Y, this->LowerBound_.Y, this->UpperBound_.Y);

            return RandomPosition;
        }

        APoint GenerateLinearPosition (int BreakpointIndex)
        {
            double M = static_cast<double>(BreakpointIndex) / (this->NBreakpoint_ - 1);

            APoint RandomPosition = *this->Start_ + (*this->Goal_ - *this->Start_) * M;
            RandomPosition.X += GenerateRandom(-this->Range_.X * 0.05, this->Range_.X * 0.05);
            RandomPosition.Y += GenerateRandom(-this->Range_.Y * 0.05, this->Range_.Y * 0.05);

            return RandomPosition;
        }

        double ObjectiveFunction (const std::vector<APoint> &Position)
        {
            auto Breakpoint = ConstructBreakpoint(Position);
            auto X = Breakpoint.first;
            auto Y = Breakpoint.second;

            double Length = 0.0f;
            std::vector<APoint> Waypoint;

            switch (this->TrajectoryType_)
            {
                case TRAJECTORY::LINEAR:
                    LinearPath(Length, Waypoint, X, Y);
                    break;

                case TRAJECTORY::CUBIC_SPLINE:
                    CubicSplinePath(Length, Waypoint, X, Y);
                    break;

                default:
                    CubicSplinePath(Length, Waypoint, X, Y);
            }

            double Error = Penalty(Waypoint);
            Error = (Error == 0.0f ? 0.0f : this->PenaltyScalingFactor_ * Error);

            double Cost = this->CostScalingFactor_ * Length * (1.0f + Error);

            return Cost;
        }

        std::pair<std::vector<double>, std::vector<double>> ConstructBreakpoint (const std::vector<APoint> &Position)
        {
            std::vector<double> X(this->NBreakpoint_ + 2);
            std::vector<double> Y(this->NBreakpoint_ + 2);

            X.front() = this->Start_->X;
            Y.front() = this->Start_->Y;

            for (int BreakpointIndex = 1; BreakpointIndex <= this->NBreakpoint_; BreakpointIndex++)
            {
                X[BreakpointIndex] = Position[BreakpointIndex - 1].X;
                Y[BreakpointIndex] = Position[BreakpointIndex - 1].Y;
            }

            X.back() = this->Goal_->X;
            Y.back() = this->Goal_->Y;

            return {X, Y};
        }

        void CubicSplinePath (double &Length,
                              std::vector<APoint> &Waypoint,
                              const std::vector<double> &X,
                              const std::vector<double> &Y) const
        {
            std::vector<double> Interpolation = LinearInterpolation(0.0f, 1.0f, this->NBreakpoint_ + 2);

            tk::spline CubicSplineX(Interpolation, X);
            tk::spline CubicSplineY(Interpolation, Y);

            Interpolation = LinearInterpolation(0.0f, 1.0f, this->NWaypoint_);

            Waypoint.clear();

            for (int WaypointIndex = 0; WaypointIndex < this->NWaypoint_; WaypointIndex++)
            {
                double XInterpolation = CubicSplineX(Interpolation[WaypointIndex]);
                double YInterpolation = CubicSplineY(Interpolation[WaypointIndex]);

                Waypoint.emplace_back(XInterpolation, YInterpolation);

                CalculateLength(Length, Waypoint, WaypointIndex);
            }
        }

        void LinearPath (double &Length,
                         std::vector<APoint> &Waypoint,
                         const std::vector<double> &X,
                         const std::vector<double> &Y) const
        {
            int NInterpolationPoint = (this->NWaypoint_ - 1) / (this->NBreakpoint_ + 1);

            Waypoint.clear();
            Waypoint.emplace_back(X[0], Y[0]);

            for (int i = 1; i < this->NBreakpoint_ + 2; i++)
            {
                std::vector<double> InterpolationX = LinearInterpolation(X[i - 1], X[i], NInterpolationPoint);
                std::vector<double> InterpolationY = LinearInterpolation(Y[i - 1], Y[i], NInterpolationPoint);

                for (int j = 0; j < NInterpolationPoint; j++)
                {
                    Waypoint.emplace_back(InterpolationX[j], InterpolationY[j]);
                }
            }

            for (int WaypointIndex = 0; WaypointIndex < this->NWaypoint_; WaypointIndex++)
            {
                CalculateLength(Length, Waypoint, WaypointIndex);
            }
        }

        static void CalculateLength (double &Length, const std::vector<APoint> &Waypoint, int WaypointIndex)
        {
            if (WaypointIndex >= 1)
            {
                double DX = Waypoint[WaypointIndex].X - Waypoint[WaypointIndex - 1].X;
                double DY = Waypoint[WaypointIndex].Y - Waypoint[WaypointIndex - 1].Y;
                Length += std::hypot(DX, DY);
            }
        }

        int XYToIndex (int X, int Y) const
        {
            return Y * static_cast<int>(this->Range_.X) + X;
        }

        double Penalty (const std::vector<APoint> &Waypoint) const
        {
            int BreakpointIndex = 0;
            int NInterpolationPoint = (this->NWaypoint_ - 1) / (this->NBreakpoint_ + 1);
            double Penalty = 0.0f;

            for (int WaypointIndex = 0; WaypointIndex < this->NWaypoint_; WaypointIndex++)
            {
                int X = static_cast<int>(Waypoint[WaypointIndex].X);
                int Y = static_cast<int>(Waypoint[WaypointIndex].Y);

                int Index = XYToIndex(X, Y);

                if (Index < 0 || Index >= this->N_ || this->CostMap_[Index] >= static_cast<int>(this->ObstacleCostFactor_ * LETHAL_COST))
                {
                    if (WaypointIndex == BreakpointIndex)
                    {
                        Penalty += 100.0f;
                    } else {
                        Penalty += 20.0;
                    }
                }

                if (WaypointIndex % NInterpolationPoint == 0)
                {
                    BreakpointIndex += NInterpolationPoint;
                }
            }

            return Penalty;
        }
    };
} // MTH

#endif // BASE_PLANNER_H
