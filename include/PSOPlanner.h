//
// Created by Sippawit Thammawiset on 17/2/2024 AD.
//

#ifndef PSO_PLANNER_H
#define PSO_PLANNER_H

#include "BasePlanner.h"
#include <fstream>

namespace MTH
{
    namespace PSO
    {
        namespace VELOCITY_CONFINEMENT
        {
            enum VELOCITY_CONFINEMENT
            {
                RANDOM_BACK = 0,
                HYPERBOLIC = 1,
                MIXED = 2
            };
        }

        typedef struct AParticle
        {
            AParticle () : BestCost((double) INFINITY), Cost(0.0f) {}

            std::vector<APoint> Position;
            std::vector<APoint> Velocity;
            double Cost;

            std::vector<APoint> BestPosition;
            double BestCost;

            std::vector<APoint> Feedback;
        } AParticle;

        class APSOPlanner : public ABasePlanner
        {
        public:
            APSOPlanner (const APoint &LowerBound, const APoint &UpperBound,
                         int MaximumIteration, int NPopulation, int NBreakpoint, int NWaypoint,
                         double SocialCoefficient = 1.5f, double CognitiveCoefficient = 1.5f,
                         double MaximumInertialWeight = 0.9f, double MinimumInertialWeight = 0.4,
                         int VelocityConfinement = VELOCITY_CONFINEMENT::RANDOM_BACK,
                         double VelocityFactor = 0.5,
                         INITIAL_POSITION_TYPE InitialPositionType = INITIAL_POSITION::DISTRIBUTED,
                         TRAJECTORY_TYPE TrajectoryType = TRAJECTORY::CUBIC_SPLINE,
                         bool Log = true) :
                         ABasePlanner(LowerBound, UpperBound,
                                      MaximumIteration, NPopulation, NBreakpoint, NWaypoint,
                                      TrajectoryType),
                         SocialCoefficient_(SocialCoefficient),
                         CognitiveCoefficient_(CognitiveCoefficient),
                         MaximumInertialWeight_(MaximumInertialWeight),
                         MinimumInertialWeight_(MinimumInertialWeight),
                         VelocityFactor_(VelocityFactor),
                         VelocityConfinement_(VelocityConfinement),
                         InitialPositionType_(InitialPositionType),
                         Log_(Log)
            {
                std::cout << "[INFO] PSO Planner instance has been created. " << std::endl;
            }

            ~APSOPlanner () override = default;

            bool CreatePlan (const unsigned char *CostMap,
                            const APoint &Start,
                            const APoint &Goal,
                            std::vector<APoint> &Waypoint) override
            {
                this->CostMap_ = CostMap;
                this->Start_ = &Start;
                this->Goal_ = &Goal;

                // Initialize
                this->Population_ = std::vector<AParticle>(this->NPopulation_);

                this->Range_ = this->UpperBound_ - this->LowerBound_;
                this->N_ = static_cast<int>(this->Range_.X * this->Range_.Y);

                this->MaximumVelocity_ = (this->UpperBound_ - this->LowerBound_) * this->VelocityFactor_;
                this->MinimumVelocity_ = MaximumVelocity_ * -1.0f;

                for (int PopulationIndex = 0; PopulationIndex < this->NPopulation_; PopulationIndex++)
                {
                    auto *CurrentPopulation = &this->Population_[PopulationIndex];

                    std::vector<APoint> Position(this->NBreakpoint_);
                    std::vector<APoint> Velocity(this->NBreakpoint_);

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

                        APoint RandomVelocity;
                        RandomVelocity = (this->LowerBound_ - RandomPosition) +
                                         (this->UpperBound_ - LowerBound_) * GenerateRandom(0.0f, 1.0f);

                        Position[BreakpointIndex] = RandomPosition;
                        Velocity[BreakpointIndex] = RandomVelocity;
                    }

                    CurrentPopulation->Position = Position;
                    CurrentPopulation->Velocity = Velocity;

                    double Cost = ObjectiveFunction(CurrentPopulation->Position);

                    CurrentPopulation->Cost = Cost;

                    this->AverageCost_ += Cost;

                    CurrentPopulation->BestPosition = CurrentPopulation->Position;
                    CurrentPopulation->BestCost = Cost;
                    CurrentPopulation->Feedback = std::vector<APoint>(this->NBreakpoint_);

                    if (Cost < this->GlobalBestCost_)
                    {
                        this->GlobalBestPosition_ = CurrentPopulation->Position;
                        this->GlobalBestCost_ = Cost;
                    }
                }

                this->AverageCost_ /= this->NPopulation_;

                // Optimize
                for (int Iteration = 1; Iteration <= this->MaximumIteration_; Iteration++)
                {
                    Optimize(Iteration);

                    this->NextAverageCost_ /= this->NPopulation_;
                    this->AverageCost_ = this->NextAverageCost_;
                    this->NextAverageCost_ = 0.0f;

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

        protected:
            double InertialWeight_{}, SocialCoefficient_, CognitiveCoefficient_;
            double MaximumInertialWeight_, MinimumInertialWeight_;
            double VelocityFactor_;
            int VelocityConfinement_;

            std::vector<AParticle> Population_;
            APoint MaximumVelocity_, MinimumVelocity_;

            double AverageCost_ = 0.0f;
            double NextAverageCost_ = 0.0f;

            INITIAL_POSITION_TYPE InitialPositionType_;
            bool Log_;

        private:
            void CalculateAdaptiveInertialWeight(AParticle *CurrentPopulation)
            {
                if (CurrentPopulation->Cost <= this->AverageCost_)
                {
                    this->InertialWeight_ = this->MinimumInertialWeight_ +
                                            (this->MaximumInertialWeight_ - this->MinimumInertialWeight_) *
                                            ((CurrentPopulation->Cost - this->GlobalBestCost_) /
                                             (this->AverageCost_ - this->GlobalBestCost_));
                }
                else
                {
                    this->InertialWeight_ = this->MinimumInertialWeight_ +
                                            (this->MaximumInertialWeight_ - this->MinimumInertialWeight_) *
                                            ((this->AverageCost_ - this->GlobalBestCost_) /
                                             (CurrentPopulation->Cost - this->GlobalBestCost_));
                }
            }

            void Optimize(int Iteration)
            {
                for (int PopulationIndex = 0; PopulationIndex < this->NPopulation_; PopulationIndex++)
                {
                    auto *CurrentPopulation = &this->Population_[PopulationIndex];

                    CalculateAdaptiveInertialWeight(CurrentPopulation);

                    // Update Velocity
                    for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; BreakpointIndex++)
                    {
                        APoint NewVelocity = UpdateVelocity(CurrentPopulation, BreakpointIndex);

                        CurrentPopulation->Velocity[BreakpointIndex] = NewVelocity;
                    }

                    // Update Position
                    for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; BreakpointIndex++)
                    {
                        APoint NewPosition = UpdatePosition(CurrentPopulation, BreakpointIndex);

                        CurrentPopulation->Position[BreakpointIndex] = NewPosition;
                    }

                    // Evaluate Cost
                    double Cost = ObjectiveFunction(CurrentPopulation->Position);
                    CurrentPopulation->Cost = Cost;

                    this->NextAverageCost_ += Cost;

                    // Update PBest
                    if (Cost < CurrentPopulation->BestCost)
                    {
                        CurrentPopulation->BestPosition = CurrentPopulation->Position;
                        CurrentPopulation->BestCost = Cost;

                        for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; BreakpointIndex++)
                        {
                            CurrentPopulation->Feedback[BreakpointIndex] = CurrentPopulation->Feedback[BreakpointIndex] * (1.0f / static_cast<double>(Iteration)) +
                                                                           (CurrentPopulation->Velocity[BreakpointIndex]) *
                                                                           GenerateRandom(0.0f, 1.0f);
                        }
                    } else {
                        for (int BreakpointIndex = 0; BreakpointIndex < this->NBreakpoint_; BreakpointIndex++)
                        {
                            CurrentPopulation->Feedback[BreakpointIndex] = CurrentPopulation->Feedback[BreakpointIndex] * (1.0f / static_cast<double>(Iteration)) -
                                                                           (CurrentPopulation->Velocity[BreakpointIndex]) *
                                                                           GenerateRandom(0.0f, 1.0f);
                        }
                    }

                    // Update GBest
                    if (Cost < this->GlobalBestCost_)
                    {
                        this->GlobalBestPosition_ = CurrentPopulation->Position;
                        this->GlobalBestCost_ = Cost;
                    }
                }
            }

            APoint UpdateVelocity(const AParticle *CurrentPopulation, int BreakpointIndex)
            {
                APoint NewVelocity;
                NewVelocity.X = this->InertialWeight_ * CurrentPopulation->Velocity[BreakpointIndex].X +
                                this->SocialCoefficient_ * GenerateRandom(0.0f, 1.0f) *
                                (CurrentPopulation->BestPosition[BreakpointIndex].X -
                                 CurrentPopulation->Position[BreakpointIndex].X) +
                                this->CognitiveCoefficient_ * GenerateRandom(0.0f, 1.0f) *
                                (this->GlobalBestPosition_[BreakpointIndex].X -
                                 CurrentPopulation->Position[BreakpointIndex].X) +
                                CurrentPopulation->Feedback[BreakpointIndex].X;
                NewVelocity.Y = this->InertialWeight_ * CurrentPopulation->Velocity[BreakpointIndex].Y +
                                this->SocialCoefficient_ * GenerateRandom(0.0f, 1.0f) *
                                (CurrentPopulation->BestPosition[BreakpointIndex].Y -
                                 CurrentPopulation->Position[BreakpointIndex].Y) +
                                this->CognitiveCoefficient_ * GenerateRandom(0.0f, 1.0f) *
                                (this->GlobalBestPosition_[BreakpointIndex].Y -
                                 CurrentPopulation->Position[BreakpointIndex].Y) +
                                CurrentPopulation->Feedback[BreakpointIndex].Y;

                NewVelocity.X = CLAMP(NewVelocity.X, this->MinimumVelocity_.X, this->MaximumVelocity_.X);
                NewVelocity.Y = CLAMP(NewVelocity.Y, this->MinimumVelocity_.Y, this->MaximumVelocity_.Y);

                return NewVelocity;
            }

            APoint UpdatePosition(AParticle *CurrentPopulation, int BreakpointIndex) const
            {
                APoint TemporaryNewPosition =
                        CurrentPopulation->Position[BreakpointIndex] + CurrentPopulation->Velocity[BreakpointIndex];

                if (IS_OUT_OF_BOUND(TemporaryNewPosition.X, this->LowerBound_.X, this->UpperBound_.X) ||
                    IS_OUT_OF_BOUND(TemporaryNewPosition.Y, this->LowerBound_.Y, this->UpperBound_.Y)) {
                    APoint Confinement;

                    switch (this->VelocityConfinement_)
                    {
                        case VELOCITY_CONFINEMENT::RANDOM_BACK:
                            Confinement = RandomBackConfinement(CurrentPopulation->Velocity[BreakpointIndex]);

                            break;
                        case VELOCITY_CONFINEMENT::HYPERBOLIC:
                            Confinement = HyperbolicConfinement(CurrentPopulation->Position[BreakpointIndex],
                                                                CurrentPopulation->Velocity[BreakpointIndex]);

                            break;
                        case VELOCITY_CONFINEMENT::MIXED:
                            Confinement = MixedConfinement(CurrentPopulation->Position[BreakpointIndex],
                                                           CurrentPopulation->Velocity[BreakpointIndex]);

                            break;
                        default:
                            Confinement = RandomBackConfinement(CurrentPopulation->Velocity[BreakpointIndex]);
                    }

                    CurrentPopulation->Velocity[BreakpointIndex] = Confinement;
                }

                APoint NewPosition =
                        CurrentPopulation->Position[BreakpointIndex] + CurrentPopulation->Velocity[BreakpointIndex];

                NewPosition.X = CLAMP(NewPosition.X, this->LowerBound_.X, this->UpperBound_.X);
                NewPosition.Y = CLAMP(NewPosition.Y, this->LowerBound_.Y, this->UpperBound_.Y);

                return NewPosition;
            }

            static APoint RandomBackConfinement(const APoint &Velocity)
            {
                APoint VelocityConfinement = Velocity * -GenerateRandom(0.0f, 1.0f);

                return VelocityConfinement;
            }

            APoint HyperbolicConfinement(const APoint &Position, const APoint &Velocity) const
            {
                APoint VelocityConfinement;

                if (Velocity.X > 0.0f) {
                    VelocityConfinement.X = Velocity.X /
                                            (1.0f + abs(Velocity.X / (this->UpperBound_.X - Position.X)));
                } else {
                    VelocityConfinement.X = Velocity.X /
                                            (1.0f + abs(Velocity.X / (Position.X - this->LowerBound_.X)));
                }

                if (Velocity.Y > 0.0f) {
                    VelocityConfinement.Y = Velocity.Y /
                                            (1.0f + abs(Velocity.Y / (this->UpperBound_.Y - Position.Y)));
                } else {
                    VelocityConfinement.Y = Velocity.Y /
                                            (1.0f + abs(Velocity.Y / (Position.Y - this->LowerBound_.Y)));
                }

                return VelocityConfinement;
            }

            APoint MixedConfinement(const APoint &Position, const APoint &Velocity) const
            {
                APoint VelocityConfinement;

                if (GenerateRandom(0.0f, 1.0) >= 0.5f) {
                    VelocityConfinement = HyperbolicConfinement(Position, Velocity);
                } else {
                    VelocityConfinement = RandomBackConfinement(Velocity);
                }

                return VelocityConfinement;
            }
        };
    } // PSO
} // MTH

#endif // PSO_PLANNER_H
