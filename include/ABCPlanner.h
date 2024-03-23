//
// Created by Sippawit Thammawiset on 11/2/2024 AD.
//

#ifndef ABC_PLANNER_H
#define ABC_PLANNER_H

#include "BasePlanner.h"

namespace MTH
{
    namespace ABC
    {
        typedef struct ABee
        {
            ABee () : Cost(0.0f), FitnessValue(0.0f), Probability(0.0f), Trial(0) {}

            std::vector<APoint> Position;
            double Cost;
            double FitnessValue;

            double Probability;
            int Trial;
        } ABee;

        class AABCPlanner : public ABasePlanner
        {
        public:
            AABCPlanner(const APoint &LowerBound, const APoint &UpperBound,
                        int MaximumIteration, int NPopulation, int NBreakpoint, int NWaypoint,
                        INITIAL_POSITION_TYPE InitialPositionType = INITIAL_POSITION::DISTRIBUTED,
                        TRAJECTORY_TYPE TrajectoryType = TRAJECTORY::CUBIC_SPLINE,
                        bool Log = true) :
                        ABasePlanner(LowerBound, UpperBound,
                                     MaximumIteration, NPopulation, NBreakpoint, NWaypoint,
                                     TrajectoryType),
                        InitialPositionType_(InitialPositionType),
                        Log_(Log)
            {
                std::cout << "[INFO] ABC Planner instance created." << std::endl;
            }

            ~AABCPlanner () override = default;

            bool CreatePlan (const unsigned char *CostMap,
                             const APoint &Start,
                             const APoint &Goal,
                             std::vector<APoint> &Waypoint) override
            {
                this->CostMap_ = CostMap;
                this->Start_ = &Start;
                this->Goal_ = &Goal;

                // Initialize
                this->Range_ = this->UpperBound_ - this->LowerBound_;
                this->N_ = static_cast<int>(this->Range_.X * this->Range_.Y);

                this->NEmployedBee_ = this->NPopulation_ / 2;
                this->NOnLookerBee_ = this->NEmployedBee_;
                this->TrialLimit_ = this->NPopulation_ * this->NBreakpoint_ / 2;

                this->FoodSource_ = std::vector<ABee>(this->NEmployedBee_);

                for (int EmployedBeeIndex = 0; EmployedBeeIndex < this->NEmployedBee_; EmployedBeeIndex++)
                {
                    auto *CurrentEmployedBee = &this->FoodSource_[EmployedBeeIndex];

                    std::vector<APoint> Position(this->NBreakpoint_);

                    for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; BreakpointIndex++)
                    {
                        APoint RandomPosition;

                        switch (this->InitialPositionType_)
                        {
                            case INITIAL_POSITION::DISTRIBUTED:
                                RandomPosition = GenerateDistributedPosition();
                                break;

                            case INITIAL_POSITION::CIRCULAR:
                                RandomPosition = GenerateCircularPosition();
                                break;

                            case INITIAL_POSITION::LINEAR:
                                RandomPosition = GenerateLinearPosition(BreakpointIndex);
                                break;

                            default:
                                RandomPosition = GenerateDistributedPosition();
                        }

                        Position[BreakpointIndex] = RandomPosition;
                    }

                    CurrentEmployedBee->Position = Position;

                    double Cost = ObjectiveFunction(CurrentEmployedBee->Position);
                    double FitnessValue = FitnessFunction(Cost);

                    CurrentEmployedBee->Cost = Cost;
                    CurrentEmployedBee->FitnessValue = FitnessValue;

                    if (CurrentEmployedBee->Cost < this->GlobalBestCost_)
                    {
                        this->GlobalBestPosition_ = CurrentEmployedBee->Position;
                        this->GlobalBestCost_ = CurrentEmployedBee->Cost;
                    }
                }

                // Optimize
                for (int Iteration = 1; Iteration <= this->MaximumIteration_; Iteration++)
                {
                    Optimize();

                    if (this->Log_)
                    {
                        std::cout << "[INFO] Iteration: " << Iteration << " >>> "
                                  << "Best Cost: " << this->GlobalBestCost_ << std::endl;
                    }
                }

                std::cout << "[INFO] Completed." << std::endl;

                auto Breakpoint = ConstructBreakpoint(this->GlobalBestPosition_);
                auto X = Breakpoint.first;
                auto Y = Breakpoint.second;

                double Length = 0.0f;

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

                if (Waypoint.empty())
                {
                    std::cerr << "[INFO] Path not found!" << std::endl;

                    return STATE::FAILED;
                }

                this->PathLength_ = Length;

                return STATE::SUCCESS;
            }

            void Clear () override
            {
                this->GlobalBestPosition_.clear();
                this->GlobalBestCost_ = (double) INFINITY;
            }

        private:
            int NEmployedBee_{}, NOnLookerBee_{}, NScoutBee = 1;
            int TrialLimit_{};

            std::vector<ABee> FoodSource_;
            double MaximumFitnessValue_ = -(double) INFINITY;

            INITIAL_POSITION_TYPE InitialPositionType_;
            bool Log_;

            static double FitnessFunction (double Cost)
            {
                if (Cost >= 0)
                {
                    return 1.0f / (1.0f + Cost);
                }

                return 1.0f + abs(Cost);
            }

            void Optimize ()
            {
                SendEmployedBee();
                CalculateProbability();
                SendOnLookerBee();
                SendScoutBee();

                for (int EmployedBeeIndex = 0; EmployedBeeIndex < this->NEmployedBee_; EmployedBeeIndex++)
                {
                    auto *CurrentEmployedBee = &this->FoodSource_[EmployedBeeIndex];

                    if (CurrentEmployedBee->Cost < this->GlobalBestCost_)
                    {
                        this->GlobalBestPosition_ = CurrentEmployedBee->Position;
                        this->GlobalBestCost_ = CurrentEmployedBee->Cost;
                    }
                }
            }

            void SendEmployedBee ()
            {
                for (int EmployedBeeIndex = 0; EmployedBeeIndex < this->NEmployedBee_; EmployedBeeIndex++)
                {
                    auto *CurrentEmployedBee = &this->FoodSource_[EmployedBeeIndex];

                    int BreakpointIndex = GenerateRandomIndex(this->NBreakpoint_);
                    int PartnerBeeIndex;

                    do
                    {
                        PartnerBeeIndex = GenerateRandomIndex(this->NEmployedBee_);
                    }
                    while (PartnerBeeIndex == EmployedBeeIndex);

                    std::vector<APoint> UpdatedPosition = CurrentEmployedBee->Position;

                    UpdatedPosition[BreakpointIndex] = CurrentEmployedBee->Position[BreakpointIndex] +
                                                       (CurrentEmployedBee->Position[BreakpointIndex] -
                                                        this->FoodSource_[PartnerBeeIndex].Position[BreakpointIndex]) *
                                                       ((GenerateRandom(0.0f, 1.0f) - 0.5f) * 2);

                    UpdatedPosition[BreakpointIndex].X = CLAMP(UpdatedPosition[BreakpointIndex].X,
                                                               this->LowerBound_.X,
                                                               this->UpperBound_.X);
                    UpdatedPosition[BreakpointIndex].Y = CLAMP(UpdatedPosition[BreakpointIndex].Y,
                                                               this->LowerBound_.Y,
                                                               this->UpperBound_.Y);

                    std::vector<APoint> Waypoint;
                    double Cost = ObjectiveFunction(UpdatedPosition);
                    double FitnessValue = FitnessFunction(Cost);

                    if (FitnessValue > CurrentEmployedBee->FitnessValue)
                    {
                        CurrentEmployedBee->Position = UpdatedPosition;
                        CurrentEmployedBee->Cost = Cost;
                        CurrentEmployedBee->FitnessValue = FitnessValue;
                        CurrentEmployedBee->Trial = 0;
                    }
                    else
                    {
                        CurrentEmployedBee->Trial++;
                    }

                    this->MaximumFitnessValue_ = std::max<double>(this->MaximumFitnessValue_,
                                                                  CurrentEmployedBee->FitnessValue);
                }
            }

            void CalculateProbability ()
            {
                for (int EmployedBeeIndex = 0; EmployedBeeIndex < this->NEmployedBee_; EmployedBeeIndex++)
                {
                    auto *CurrentEmployedBee = &this->FoodSource_[EmployedBeeIndex];

                    CurrentEmployedBee->Probability = (0.9f * (CurrentEmployedBee->FitnessValue / this->MaximumFitnessValue_)) + 0.1f;
                }
            }

            void SendOnLookerBee ()
            {
                int EmployedBeeIndex = 0;
                int OnLookerBeeIndex = 0;
                while (OnLookerBeeIndex < this->NOnLookerBee_)
                {
                    auto *CurrentEmployedBee = &this->FoodSource_[EmployedBeeIndex];

                    double RandomProbability = GenerateRandom(0.0f, 1.0f);

                    if (RandomProbability < CurrentEmployedBee->Probability)
                    {
                        int BreakpointIndex = GenerateRandomIndex(this->NBreakpoint_);
                        int PartnerBeeIndex;

                        do
                        {
                            PartnerBeeIndex = GenerateRandomIndex(this->NEmployedBee_);
                        }
                        while (PartnerBeeIndex == EmployedBeeIndex);

                        std::vector<APoint> UpdatedPosition = CurrentEmployedBee->Position;

                        UpdatedPosition[BreakpointIndex] = CurrentEmployedBee->Position[BreakpointIndex] +
                                                           (CurrentEmployedBee->Position[BreakpointIndex] -
                                                            this->FoodSource_[PartnerBeeIndex].Position[BreakpointIndex]) *
                                                           ((GenerateRandom(0.0f, 1.0f) - 0.5f) * 2);

                        UpdatedPosition[BreakpointIndex].X = CLAMP(UpdatedPosition[BreakpointIndex].X,
                                                                   this->LowerBound_.X,
                                                                   this->UpperBound_.X);
                        UpdatedPosition[BreakpointIndex].Y = CLAMP(UpdatedPosition[BreakpointIndex].Y,
                                                                   this->LowerBound_.Y,
                                                                   this->UpperBound_.Y);

                        std::vector<APoint> Waypoint;
                        double Cost = ObjectiveFunction(UpdatedPosition);
                        double FitnessValue = FitnessFunction(Cost);

                        if (FitnessValue > CurrentEmployedBee->FitnessValue)
                        {
                            CurrentEmployedBee->Position = UpdatedPosition;
                            CurrentEmployedBee->Cost = Cost;
                            CurrentEmployedBee->FitnessValue = FitnessValue;
                            CurrentEmployedBee->Trial = 0;
                        }
                        else
                        {
                            CurrentEmployedBee->Trial++;
                        }

                        OnLookerBeeIndex++;
                    }

                    EmployedBeeIndex = (EmployedBeeIndex + 1) % this->NOnLookerBee_;
                }
            }

            void SendScoutBee ()
            {
                for (int ScoutBeeIndex = 0; ScoutBeeIndex < this->NScoutBee; ScoutBeeIndex++)
                {
                    for (int EmployedBeeIndex = 0; EmployedBeeIndex < this->NEmployedBee_; EmployedBeeIndex++)
                    {
                        auto *CurrentEmployedBee = &this->FoodSource_[EmployedBeeIndex];

                        if (CurrentEmployedBee->Trial >= this->TrialLimit_)
                        {
                            std::vector<APoint> Position(this->NBreakpoint_);

                            for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; BreakpointIndex++)
                            {
                                APoint RandomPosition;
                                RandomPosition.X = GenerateRandom(this->LowerBound_.X,
                                                                  this->UpperBound_.X);
                                RandomPosition.Y = GenerateRandom(this->LowerBound_.Y,
                                                                  this->UpperBound_.Y);

                                Position[BreakpointIndex] = RandomPosition;
                            }

                            CurrentEmployedBee->Position = Position;

                            std::vector<APoint> Waypoint;
                            double Cost = ObjectiveFunction(CurrentEmployedBee->Position);
                            double FitnessValue = FitnessFunction(Cost);

                            CurrentEmployedBee->Cost = Cost;
                            CurrentEmployedBee->FitnessValue = FitnessValue;
                            CurrentEmployedBee->Trial = 0;
                        }
                    }
                }
            }
        };
    }
} // MTH

#endif // ABC_PLANNER_H
