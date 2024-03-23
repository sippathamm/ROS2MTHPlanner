//
// Created by Sippawit Thammawiset on 6/2/2024 AD.
//

#ifndef GWO_PLANNER_H
#define GWO_PLANNER_H

#include "BasePlanner.h"

namespace MTH
{
    namespace GWO
    {
        typedef struct AWolf
        {
            AWolf () : Cost((double) INFINITY) {}

            std::vector<APoint> Position;
            double Cost;
        } AWolf;

        typedef struct ALeaderWolf
        {
            AWolf Alpha;    // 1st Best
            AWolf Beta;     // 2nd Best
            AWolf Delta;    // 3rd Best
        } ALeaderWolf;

        class AGWOPlanner : public ABasePlanner
        {
        public:
            AGWOPlanner (const APoint &LowerBound, const APoint &UpperBound,
                         int MaximumIteration, int NPopulation, int NBreakpoint, int NWaypoint,
                         double Theta, double K,
                         double Maximum_a = 2.2f, double Minimum_a = 0.02f,
                         double MaximumWeight = 0.9f, double MinimumWeight = 0.4f,
                         double VelocityFactor = 0.5f,
                         INITIAL_POSITION_TYPE InitialPositionType = INITIAL_POSITION::DISTRIBUTED,
                         TRAJECTORY_TYPE TrajectoryType = TRAJECTORY::CUBIC_SPLINE,
                         bool Log = true) :
                         ABasePlanner(LowerBound, UpperBound,
                                      MaximumIteration, NPopulation, NBreakpoint, NWaypoint,
                                      TrajectoryType),
                         Theta_(Theta),
                         K_(K),
                         Maximum_a_(Maximum_a),
                         Minimum_a_(Minimum_a),
                         MaximumWeight_(MaximumWeight),
                         MinimumWeight_(MinimumWeight),
                         VelocityFactor_(VelocityFactor),
                         InitialPositionType_(InitialPositionType),
                         Log_(Log)
            {
                std::cout << "[INFO] GWO Planner instance has been created." << std::endl;
            }

            ~AGWOPlanner () override = default;

            bool CreatePlan (const unsigned char *CostMap,
                             const APoint &Start,
                             const APoint &Goal,
                             std::vector<APoint> &Waypoint) override
            {
                this->CostMap_ = CostMap;
                this->Start_ = &Start;
                this->Goal_ = &Goal;

                // Initialize
                this->Population_ = std::vector<AWolf> (NPopulation_);

                this->Range_ = this->UpperBound_ - this->LowerBound_;
                this->N_ = static_cast<int>(this->Range_.X * this->Range_.Y);

                this->MaximumVelocity_ = (this->UpperBound_ - this->LowerBound_) * this->VelocityFactor_;
                this->MinimumVelocity_ = MaximumVelocity_ * -1.0f;

                for (int PopulationIndex = 0; PopulationIndex < this->NPopulation_; PopulationIndex++)
                {
                    auto *CurrentPopulation = &this->Population_[PopulationIndex];

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

                    CurrentPopulation->Position = Position;

                    double Cost = ObjectiveFunction(CurrentPopulation->Position);

                    CurrentPopulation->Cost = Cost;

                    this->AverageCost_ += Cost;
                }

                this->AverageCost_ /= this->NPopulation_;

                // Optimize
                for (int Iteration = 1; Iteration <= this->MaximumIteration_; Iteration++)
                {
                    // Update Alpha, Beta, and Delta wolf
                    UpdateGlobalBestPosition();

                    // Calculate a_Alpha, a_Beta, a_Delta
                    Calculate_a(Iteration);

                    Optimize();

                    if (this->Log_)
                    {
                        std::cout << "[INFO] Iteration: " << Iteration << " >>> " << "Best Cost: "
                                  << this->GlobalBestPosition_.Alpha.Cost << std::endl;
                    }

                    this->NextAverageCost_ /= this->NPopulation_;
                    this->AverageCost_ = this->NextAverageCost_;
                    this->NextAverageCost_ = 0.0f;
                }

                std::cout << "[INFO] Completed." << std::endl;

                auto Breakpoint = ConstructBreakpoint(this->GlobalBestPosition_.Alpha.Position);
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
                ABasePlanner::GlobalBestPosition_ = this->GlobalBestPosition_.Alpha.Position;
                ABasePlanner::GlobalBestCost_ = this->GlobalBestPosition_.Alpha.Cost;

                return STATE::SUCCESS;
            }

            void Clear () override
            {
                this->GlobalBestPosition_.Alpha.Position.clear();
                this->GlobalBestPosition_.Alpha.Cost = (double) INFINITY;

                this->GlobalBestPosition_.Beta.Position.clear();
                this->GlobalBestPosition_.Beta.Cost = (double) INFINITY;

                this->GlobalBestPosition_.Delta.Position.clear();
                this->GlobalBestPosition_.Delta.Cost = (double) INFINITY;

                this->GlobalBestCost_ = (double) INFINITY;
            }

        protected:
            double Theta_, K_;
            double Weight_{}, H_{};
            double a_Alpha_{}, a_Beta_{}, a_Delta_{};
            double AlphaGrowthFactor_ = 2.0f, DeltaGrowthFactor_ = 3.0f;
            double Maximum_a_, Minimum_a_;
            double MaximumWeight_, MinimumWeight_;
            double VelocityFactor_;

            std::vector<AWolf> Population_;
            APoint MaximumVelocity_, MinimumVelocity_;

            ALeaderWolf GlobalBestPosition_;

            double AverageCost_ = 0.0f;
            double NextAverageCost_ = 0.0f;

            INITIAL_POSITION_TYPE InitialPositionType_;
            bool Log_;

        private:
            void UpdateGlobalBestPosition ()
            {
                for (int PopulationIndex = 0; PopulationIndex < this->NPopulation_; PopulationIndex++)
                {
                    auto *CurrentPopulation = &this->Population_[PopulationIndex];

                    if (CurrentPopulation->Cost < GlobalBestPosition_.Alpha.Cost)
                    {
                        GlobalBestPosition_.Alpha.Position = CurrentPopulation->Position;
                        GlobalBestPosition_.Alpha.Cost = CurrentPopulation->Cost;
                    }
                    if (CurrentPopulation->Cost > GlobalBestPosition_.Alpha.Cost &&
                        CurrentPopulation->Cost < GlobalBestPosition_.Beta.Cost)
                    {
                        GlobalBestPosition_.Beta.Position = CurrentPopulation->Position;
                        GlobalBestPosition_.Beta.Cost = CurrentPopulation->Cost;
                    }
                    if (CurrentPopulation->Cost > GlobalBestPosition_.Alpha.Cost &&
                        CurrentPopulation->Cost > GlobalBestPosition_.Beta.Cost &&
                        CurrentPopulation->Cost < GlobalBestPosition_.Delta.Cost)
                    {
                        GlobalBestPosition_.Delta.Position = CurrentPopulation->Position;
                        GlobalBestPosition_.Delta.Cost = CurrentPopulation->Cost;
                    }
                }
            }

            void Calculate_a (int Iteration)
            {
                this->a_Alpha_ = this->Maximum_a_ *
                                 exp(pow(static_cast<double>(Iteration) / this->MaximumIteration_, this->AlphaGrowthFactor_) *
                                     log(this->Minimum_a_ / this->Maximum_a_));
                this->a_Delta_ = this->Maximum_a_ *
                                 exp(pow(static_cast<double>(Iteration) / this->MaximumIteration_, this->DeltaGrowthFactor_) *
                                     log(this->Minimum_a_ / this->Maximum_a_));
                this->a_Beta_ = (this->a_Alpha_ + this->a_Delta_) * 0.5f;
            }

            void CalculateWeight (AWolf *CurrentPopulation)
            {
                this->H_ = this->K_ * ((0.0f - CurrentPopulation->Cost) / this->AverageCost_);
                //                      ^ Optimal cost
                this->Weight_ = ((this->MaximumWeight_ + this->MinimumWeight_) * 0.5f) +
                                (this->MaximumWeight_ - this->MinimumWeight_) * (this->H_ / (std::hypot(1.0f, this->H_))) * tan(this->Theta_);
            }

            void Optimize ()
            {
                for (int PopulationIndex = 0; PopulationIndex < this->NPopulation_; PopulationIndex++)
                {
                    auto *CurrentPopulation = &this->Population_[PopulationIndex];

                    CalculateWeight(CurrentPopulation);

                    double Radius = 0.0f;
                    std::vector<APoint> GWOPosition(this->NBreakpoint_);

                    for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; BreakpointIndex++)
                    {
                        double A1 = this->a_Alpha_ * (2.0f * GenerateRandom(0.0f, 1.0f) - 1);
                        double A2 = this->a_Beta_ * (2.0f * GenerateRandom(0.0f, 1.0f) - 1);
                        double A3 = this->a_Delta_ * (2.0f * GenerateRandom(0.0f, 1.0f) - 1);

                        double C1 = 2.0f * GenerateRandom(0.0f, 1.0f);
                        double C2 = 2.0f * GenerateRandom(0.0f, 1.0f);
                        double C3 = 2.0f * GenerateRandom(0.0f, 1.0f);

                        APoint X1 = GlobalBestPosition_.Alpha.Position[BreakpointIndex] -
                                    Absolute((GlobalBestPosition_.Alpha.Position[BreakpointIndex] - CurrentPopulation->Position[BreakpointIndex]) * C1) * A1;
                        APoint X2 = GlobalBestPosition_.Beta.Position[BreakpointIndex] -
                                    Absolute((GlobalBestPosition_.Beta.Position[BreakpointIndex] - CurrentPopulation->Position[BreakpointIndex]) * C2) * A2;
                        APoint X3 = GlobalBestPosition_.Delta.Position[BreakpointIndex] -
                                    Absolute((GlobalBestPosition_.Delta.Position[BreakpointIndex] - CurrentPopulation->Position[BreakpointIndex]) * C3) * A3;

                        APoint X = (X1 + X2 + X3) / 3.0f;

                        double R1 = GenerateRandom(0.0f, 1.0f);
                        double R2 = GenerateRandom(0.0f, 1.0f);
                        double R3 = GenerateRandom(0.0f, 1.0f);

                        APoint NewVelocity = (((X1 - X) * C1 * R1) + ((X2 - X) * C2 * R2) + ((X3 - X) * C3 * R3)) * this->Weight_;

                        NewVelocity.X = CLAMP(NewVelocity.X, this->MinimumVelocity_.X, this->MaximumVelocity_.X);
                        NewVelocity.Y = CLAMP(NewVelocity.Y, this->MinimumVelocity_.Y, this->MaximumVelocity_.Y);

                        APoint TemporaryNewPosition = X + NewVelocity;

                        if (IS_OUT_OF_BOUND(TemporaryNewPosition.X, this->LowerBound_.X, this->UpperBound_.X) ||
                            IS_OUT_OF_BOUND(TemporaryNewPosition.Y, this->LowerBound_.Y, this->UpperBound_.Y))
                        {
                            APoint VelocityConfinement = NewVelocity * -GenerateRandom(0.0f, 1.0f);

                            NewVelocity = VelocityConfinement;
                        }

                        APoint NewPosition = X + NewVelocity;

                        NewPosition.X = CLAMP(NewPosition.X, this->LowerBound_.X, this->UpperBound_.X);
                        NewPosition.Y = CLAMP(NewPosition.Y, this->LowerBound_.Y, this->UpperBound_.Y);

                        // R = (X_i - X_new)^2
                        Radius += std::hypot(CurrentPopulation->Position[BreakpointIndex].X - NewPosition.X,
                                             CurrentPopulation->Position[BreakpointIndex].Y - NewPosition.Y);

                        GWOPosition[BreakpointIndex] = NewPosition;
                    }

                    // Stored index which neighbor distance <= radius
                    std::vector<int> Index = GetNeighborHoodIndex(CurrentPopulation, Radius);
                    std::vector<APoint> DLHPosition = CalculateDLHPosition(CurrentPopulation, Index);

                    double GWOCost = ObjectiveFunction(GWOPosition);
                    double DLHCost = ObjectiveFunction(DLHPosition);

                    if (GWOCost < DLHCost)
                    {
                        CurrentPopulation->Position = GWOPosition;
                        CurrentPopulation->Cost = GWOCost;

                        this->NextAverageCost_ += GWOCost;
                    }
                    else
                    {
                        CurrentPopulation->Position = DLHPosition;
                        CurrentPopulation->Cost = DLHCost;

                        this->NextAverageCost_ += DLHCost;
                    }
                }
            }

            std::vector<int> GetNeighborHoodIndex (AWolf *CurrentPopulation, double &Radius)
            {
                std::vector<int> Index;

                for (int PopulationIndex = 0; PopulationIndex < this->NPopulation_; PopulationIndex++)
                {
                    auto *AnotherPopulation = &this->Population_[PopulationIndex];

                    double Distance = 0.0;

                    for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; BreakpointIndex++)
                    {
                        // D = (X_i - X_j)^2
                        Distance += std::hypot(CurrentPopulation->Position[BreakpointIndex].X - AnotherPopulation->Position[BreakpointIndex].X,
                                               CurrentPopulation->Position[BreakpointIndex].Y - AnotherPopulation->Position[BreakpointIndex].Y);
                    }

                    if (Distance <= Radius)
                    {
                        Index.push_back(PopulationIndex);
                    }
                }

                return Index;
            }

            std::vector<APoint> CalculateDLHPosition (AWolf *CurrentPopulation, const std::vector<int> &Index)
            {
                std::vector<APoint> DLHPosition(this->NBreakpoint_);

                for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; BreakpointIndex++)
                {
                    int NeighborIndex = GenerateRandomIndex((int)Index.size());
                    int PopulationIndex = GenerateRandomIndex(this->NPopulation_);

                    APoint DLH = CurrentPopulation->Position[BreakpointIndex] +
                                 (this->Population_[Index[NeighborIndex]].Position[BreakpointIndex] -
                                 this->Population_[PopulationIndex].Position[BreakpointIndex]) * GenerateRandom(0.0f, 1.0f);

                    DLH.X = CLAMP(DLH.X, this->LowerBound_.X, this->UpperBound_.X);
                    DLH.Y = CLAMP(DLH.Y, this->LowerBound_.Y, this->UpperBound_.Y);

                    DLHPosition[BreakpointIndex] = DLH;
                }

                return DLHPosition;
            }
        };
    } // GWO
} // MTH

#endif // GWO_PLANNER_H
