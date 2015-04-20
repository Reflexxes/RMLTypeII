//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLStep1.cpp
//!
//! \brief
//! Implementation file for the Type II On-Line Trajectory
//! Generation algorithm
//!
//! \details
//! For further information, please refer to the file TypeIIRMLPosition.h.
//!
//! \date April 2015
//!
//! \version 1.2.7
//!
//! \author Torsten Kroeger, <info@reflexxes.com> \n
//!
//! \copyright Copyright (C) 2015 Google, Inc.
//! \n
//! \n
//! <b>GNU Lesser General Public License</b>
//! \n
//! \n
//! This file is part of the Type II Reflexxes Motion Library.
//! \n\n
//! The Type II Reflexxes Motion Library is free software: you can redistribute
//! it and/or modify it under the terms of the GNU Lesser General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Type II Reflexxes Motion Library is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU Lesser General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU Lesser General Public License
//! along with the Type II Reflexxes Motion Library. If not, see
//! <http://www.gnu.org/licenses/>.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#include <TypeIIRMLPosition.h>
#include <TypeIIRMLMath.h>
#include <TypeIIRMLStep1IntermediateProfiles.h>
#include <TypeIIRMLStep1Profiles.h>
#include <TypeIIRMLQuicksort.h>
#include <TypeIIRMLDecisions.h>
#include <TypeIIRMLDecisionTree1A.h>
#include <TypeIIRMLDecisionTree1B.h>
#include <TypeIIRMLDecisionTree1C.h>
#include <RMLPositionInputParameters.h>
#include <ReflexxesAPI.h>

using namespace TypeIIRMLMath;


//*******************************************************************************************
// Step1

void TypeIIRMLPosition::Step1(void)
{
    double                      MaximalMinimalExecutionTime                     =   0.0
                            ,   VectorStretchFactorMaxAcceleration              =   0.0
                            ,   VectorStretchFactorMaxVelocity                  =   0.0
                            ,   PhaseSyncTimeAverage                            =   0.0;

    unsigned int                i                                               =   0
                            ,   Counter                                         =   0
                            ,   PhaseSyncDOFCounter                             =   0;


    for(i = 0; i < this->NumberOfDOFs; i++)
    {
        if(this->CurrentInputParameters->SelectionVector->VecData[i])
        {
            TypeIIRMLDecisionTree1A(        this->CurrentInputParameters->CurrentPositionVector->VecData        [i]
                                        ,   this->CurrentInputParameters->CurrentVelocityVector->VecData        [i]
                                        ,   this->CurrentInputParameters->TargetPositionVector->VecData         [i]
                                        ,   this->CurrentInputParameters->TargetVelocityVector->VecData         [i]
                                        ,   this->CurrentInputParameters->MaxVelocityVector->VecData            [i]
                                        ,   this->CurrentInputParameters->MaxAccelerationVector->VecData        [i]
                                        ,   &(this->UsedStep1AProfiles->VecData                                 [i])
                                        ,   &(this->MinimumExecutionTimes->VecData                              [i]));

            if((this->MinimumExecutionTimes->VecData)[i] > MaximalMinimalExecutionTime)
            {
                MaximalMinimalExecutionTime = (this->MinimumExecutionTimes->VecData)[i];
                this->GreatestDOFForPhaseSynchronization = i;
            }
        }
    }

    this->SetupModifiedSelectionVector();
    // From now on, we only use the modified selection vector.

    if (this->CurrentTrajectoryIsNotSynchronized)
    {
        this->SynchronizationTime = MaximalMinimalExecutionTime;
        return;
    }

    // ******************************************************************
    // phase-synchronization check

    if (this->CurrentTrajectoryIsPhaseSynchronized)
    {
        // check whether all vectors head into the same direction
        this->CurrentTrajectoryIsPhaseSynchronized  = IsPhaseSynchronizationPossible(this->PhaseSynchronizationReferenceVector);
    }

    if ( (this->CurrentTrajectoryIsPhaseSynchronized)
        &&  (fabs((this->PhaseSynchronizationReferenceVector->VecData)[this->GreatestDOFForPhaseSynchronization]) > ABSOLUTE_PHASE_SYNC_EPSILON) )
    {
        VectorStretchFactorMaxAcceleration  =   (this->CurrentInputParameters->MaxAccelerationVector->VecData)[this->GreatestDOFForPhaseSynchronization]
                                                / fabs((this->PhaseSynchronizationReferenceVector->VecData)[this->GreatestDOFForPhaseSynchronization]);
        VectorStretchFactorMaxVelocity      =   (this->CurrentInputParameters->MaxVelocityVector->VecData)[this->GreatestDOFForPhaseSynchronization]
                                                / fabs((this->PhaseSynchronizationReferenceVector->VecData)[this->GreatestDOFForPhaseSynchronization]);

        for(i = 0; i < this->NumberOfDOFs; i++)
        {
            if((this->ModifiedSelectionVector->VecData)[i])
            {
                (this->PhaseSynchronizationTimeVector->VecData)[i]              =   0.0;

                (this->PhaseSynchronizationMaxAccelerationVector->VecData)[i]   =   fabs(VectorStretchFactorMaxAcceleration
                                                                                            * (this->PhaseSynchronizationReferenceVector->VecData)[i]);
                (this->PhaseSynchronizationMaxVelocityVector->VecData)[i]       =   fabs(VectorStretchFactorMaxVelocity
                                                                                            * (this->PhaseSynchronizationReferenceVector->VecData)[i]);

                if  (   ( (this->PhaseSynchronizationMaxAccelerationVector->VecData)[i]
                        > ( (this->CurrentInputParameters->MaxAccelerationVector->VecData)[i] * ( 1.0 + RELATIVE_PHASE_SYNC_EPSILON ) + ABSOLUTE_PHASE_SYNC_EPSILON ) )
                    ||  ( (this->PhaseSynchronizationMaxVelocityVector->VecData)[i]
                        > ( (this->CurrentInputParameters->MaxVelocityVector->VecData)[i] * ( 1.0 + RELATIVE_PHASE_SYNC_EPSILON ) + ABSOLUTE_PHASE_SYNC_EPSILON ) ) )
                {
                    this->CurrentTrajectoryIsPhaseSynchronized = false;
                    break;
                }
            }
        }
    }
    else
    {
        this->CurrentTrajectoryIsPhaseSynchronized = false;
    }

    if (this->CurrentTrajectoryIsPhaseSynchronized)
    {
        *(this->PhaseSynchronizationCurrentPositionVector)          =   *(this->CurrentInputParameters->CurrentPositionVector);
        *(this->PhaseSynchronizationCurrentVelocityVector)          =   *(this->CurrentInputParameters->CurrentVelocityVector);
        *(this->PhaseSynchronizationTargetPositionVector)           =   *(this->CurrentInputParameters->TargetPositionVector);
        *(this->PhaseSynchronizationTargetVelocityVector)           =   *(this->CurrentInputParameters->TargetVelocityVector);

        // check, whether all DOFs can be reached with the profile of the this->GreatestDOFForPhaseSynchronization

        for(i = 0; i < this->NumberOfDOFs; i++)
        {
            if (!Decision_1A__001(this->PhaseSynchronizationCurrentVelocityVector->VecData[i]))
            {
                NegateStep1(        &(this->PhaseSynchronizationCurrentPositionVector->VecData      [i] )
                                ,   &(this->PhaseSynchronizationCurrentVelocityVector->VecData      [i] )
                                ,   &(this->PhaseSynchronizationTargetPositionVector->VecData       [i] )
                                ,   &(this->PhaseSynchronizationTargetVelocityVector->VecData       [i] )   );
            }
            if (!Decision_1A__002(      this->PhaseSynchronizationCurrentVelocityVector->VecData    [i]
                                    ,   this->PhaseSynchronizationMaxVelocityVector->VecData        [i]     ))
            {
                VToVMaxStep1(       &(this->PhaseSynchronizationTimeVector->VecData                 [i] )
                                ,   &(this->PhaseSynchronizationCurrentPositionVector->VecData      [i] )
                                ,   &(this->PhaseSynchronizationCurrentVelocityVector->VecData      [i] )
                                ,   this->PhaseSynchronizationMaxVelocityVector->VecData            [i]
                                ,   this->PhaseSynchronizationMaxAccelerationVector->VecData        [i]     );
            }

            if (    (this->UsedStep1AProfiles->VecData[this->GreatestDOFForPhaseSynchronization] == Step1_Profile_NegLinHldPosLin   )
                ||  (this->UsedStep1AProfiles->VecData[this->GreatestDOFForPhaseSynchronization] == Step1_Profile_NegLinPosLin      )
                ||  (this->UsedStep1AProfiles->VecData[this->GreatestDOFForPhaseSynchronization] == Step1_Profile_NegTrapPosLin     )
                ||  (this->UsedStep1AProfiles->VecData[this->GreatestDOFForPhaseSynchronization] == Step1_Profile_NegTriPosLin      )   )
            {
                VToZeroStep1(       &(this->PhaseSynchronizationTimeVector->VecData                 [i] )
                                ,   &(this->PhaseSynchronizationCurrentPositionVector->VecData      [i] )
                                ,   &(this->PhaseSynchronizationCurrentVelocityVector->VecData      [i] )
                                ,   this->PhaseSynchronizationMaxAccelerationVector->VecData        [i]     );

                NegateStep1(        &(this->PhaseSynchronizationCurrentPositionVector->VecData      [i] )
                                ,   &(this->PhaseSynchronizationCurrentVelocityVector->VecData      [i] )
                                ,   &(this->PhaseSynchronizationTargetPositionVector->VecData       [i] )
                                ,   &(this->PhaseSynchronizationTargetVelocityVector->VecData       [i] )   );
            }
        }

        switch((this->UsedStep1AProfiles->VecData)[this->GreatestDOFForPhaseSynchronization])
        {
        case Step1_Profile_PosLinHldNegLin:
        case Step1_Profile_NegLinHldPosLin:
            for(i = 0; i < this->NumberOfDOFs; i++)
            {
                if((this->ModifiedSelectionVector->VecData)[i])
                {
                    if (!IsSolutionForProfile_PosLinHldNegLin_Possible(     this->PhaseSynchronizationCurrentPositionVector->VecData    [i]
                                                                        ,   this->PhaseSynchronizationCurrentVelocityVector->VecData    [i]
                                                                        ,   this->PhaseSynchronizationTargetPositionVector->VecData     [i]
                                                                        ,   this->PhaseSynchronizationTargetVelocityVector->VecData     [i]
                                                                        ,   this->PhaseSynchronizationMaxVelocityVector->VecData        [i]
                                                                        ,   this->PhaseSynchronizationMaxAccelerationVector->VecData    [i] ))
                    {
                        this->CurrentTrajectoryIsPhaseSynchronized = false;
                        break;
                    }
                }
            }
            break;
        case Step1_Profile_PosLinNegLin:
        case Step1_Profile_NegLinPosLin:
            for(i = 0; i < this->NumberOfDOFs; i++)
            {
                if((this->ModifiedSelectionVector->VecData)[i])
                {
                    if (!IsSolutionForProfile_PosLinNegLin_Possible(    this->PhaseSynchronizationCurrentPositionVector->VecData    [i]
                                                                    ,   this->PhaseSynchronizationCurrentVelocityVector->VecData    [i]
                                                                    ,   this->PhaseSynchronizationTargetPositionVector->VecData     [i]
                                                                    ,   this->PhaseSynchronizationTargetVelocityVector->VecData     [i]
                                                                    ,   this->PhaseSynchronizationMaxVelocityVector->VecData        [i]
                                                                    ,   this->PhaseSynchronizationMaxAccelerationVector->VecData    [i] ))
                    {
                        this->CurrentTrajectoryIsPhaseSynchronized = false;
                        break;
                    }
                }
            }
            break;
        case Step1_Profile_PosTrapNegLin:
        case Step1_Profile_NegTrapPosLin:
            for(i = 0; i < this->NumberOfDOFs; i++)
            {
                if((this->ModifiedSelectionVector->VecData)[i])
                {
                    if (!IsSolutionForProfile_PosTrapNegLin_Possible(       this->PhaseSynchronizationCurrentPositionVector->VecData    [i]
                                                                        ,   this->PhaseSynchronizationCurrentVelocityVector->VecData    [i]
                                                                        ,   this->PhaseSynchronizationTargetPositionVector->VecData     [i]
                                                                        ,   this->PhaseSynchronizationTargetVelocityVector->VecData     [i]
                                                                        ,   this->PhaseSynchronizationMaxVelocityVector->VecData        [i]
                                                                        ,   this->PhaseSynchronizationMaxAccelerationVector->VecData    [i] ))
                    {
                        this->CurrentTrajectoryIsPhaseSynchronized = false;
                        break;
                    }
                }
            }
            break;
        case Step1_Profile_PosTriNegLin:
        case Step1_Profile_NegTriPosLin:
            for(i = 0; i < this->NumberOfDOFs; i++)
            {
                if((this->ModifiedSelectionVector->VecData)[i])
                {
                    if (!IsSolutionForProfile_PosTriNegLin_Possible(    this->PhaseSynchronizationCurrentPositionVector->VecData    [i]
                                                                    ,   this->PhaseSynchronizationCurrentVelocityVector->VecData    [i]
                                                                    ,   this->PhaseSynchronizationTargetPositionVector->VecData     [i]
                                                                    ,   this->PhaseSynchronizationTargetVelocityVector->VecData     [i]
                                                                    ,   this->PhaseSynchronizationMaxVelocityVector->VecData        [i]
                                                                    ,   this->PhaseSynchronizationMaxAccelerationVector->VecData    [i] ))
                    {
                        this->CurrentTrajectoryIsPhaseSynchronized = false;
                        break;
                    }
                }
            }
            break;
        default:
            this->CurrentTrajectoryIsPhaseSynchronized = false;
            break;
        }
    }

    if (this->CurrentTrajectoryIsPhaseSynchronized)
    {
        this->MotionProfileForPhaseSynchronization = (unsigned int)(this->UsedStep1AProfiles->VecData)[this->GreatestDOFForPhaseSynchronization];

        // Within the following
        // switch/case part, the result value for
        // ErrorDuringCalculationOfASynchronizedProfile
        // does not have to be checked as the solution
        // for the desired value of tmin will always be valid.

        switch(this->MotionProfileForPhaseSynchronization)
        {
        case Step1_Profile_PosLinHldNegLin:
        case Step1_Profile_NegLinHldPosLin:
            for(i = 0; i < this->NumberOfDOFs; i++)
            {
                if((this->ModifiedSelectionVector->VecData)[i])
                {
                    this->PhaseSynchronizationTimeVector->VecData[i]
                        +=  ProfileStep1PosLinHldNegLin(    this->PhaseSynchronizationCurrentPositionVector->VecData    [i]
                                                        ,   this->PhaseSynchronizationCurrentVelocityVector->VecData    [i]
                                                        ,   this->PhaseSynchronizationTargetPositionVector->VecData     [i]
                                                        ,   this->PhaseSynchronizationTargetVelocityVector->VecData     [i]
                                                        ,   this->PhaseSynchronizationMaxVelocityVector->VecData        [i]
                                                        ,   this->PhaseSynchronizationMaxAccelerationVector->VecData    [i] );
                }
            }
            break;
        case Step1_Profile_PosLinNegLin:
        case Step1_Profile_NegLinPosLin:
            for(i = 0; i < this->NumberOfDOFs; i++)
            {
                if((this->ModifiedSelectionVector->VecData)[i])
                {
                    this->PhaseSynchronizationTimeVector->VecData[i]
                        +=  ProfileStep1PosLinNegLin(       this->PhaseSynchronizationCurrentPositionVector->VecData    [i]
                                                        ,   this->PhaseSynchronizationCurrentVelocityVector->VecData    [i]
                                                        ,   this->PhaseSynchronizationTargetPositionVector->VecData     [i]
                                                        ,   this->PhaseSynchronizationTargetVelocityVector->VecData     [i]
                                                        ,   this->PhaseSynchronizationMaxAccelerationVector->VecData    [i] );
                }
            }
            break;
        case Step1_Profile_PosTrapNegLin:
        case Step1_Profile_NegTrapPosLin:
            for(i = 0; i < this->NumberOfDOFs; i++)
            {
                if((this->ModifiedSelectionVector->VecData)[i])
                {
                    this->PhaseSynchronizationTimeVector->VecData[i]
                        +=  ProfileStep1PosTrapNegLin(      this->PhaseSynchronizationCurrentPositionVector->VecData    [i]
                                                        ,   this->PhaseSynchronizationCurrentVelocityVector->VecData    [i]
                                                        ,   this->PhaseSynchronizationTargetPositionVector->VecData     [i]
                                                        ,   this->PhaseSynchronizationTargetVelocityVector->VecData     [i]
                                                        ,   this->PhaseSynchronizationMaxVelocityVector->VecData        [i]
                                                        ,   this->PhaseSynchronizationMaxAccelerationVector->VecData    [i] );
                }
            }
            break;
        case Step1_Profile_PosTriNegLin:
        case Step1_Profile_NegTriPosLin:
            for(i = 0; i < this->NumberOfDOFs; i++)
            {
                if((this->ModifiedSelectionVector->VecData)[i])
                {
                    this->PhaseSynchronizationTimeVector->VecData[i]
                        +=  ProfileStep1PosTriNegLin(       this->PhaseSynchronizationCurrentPositionVector->VecData    [i]
                                                        ,   this->PhaseSynchronizationCurrentVelocityVector->VecData    [i]
                                                        ,   this->PhaseSynchronizationTargetPositionVector->VecData     [i]
                                                        ,   this->PhaseSynchronizationTargetVelocityVector->VecData     [i]
                                                        ,   this->PhaseSynchronizationMaxAccelerationVector->VecData    [i] );
                }
            }
            break;
        default:
            this->CurrentTrajectoryIsPhaseSynchronized = false;
            break;
        }

        PhaseSyncTimeAverage    =   0.0;
        PhaseSyncDOFCounter     =   0;

        for(i = 0; i < this->NumberOfDOFs; i++)
        {
            if((this->ModifiedSelectionVector->VecData)[i])
            {
                PhaseSyncTimeAverage    +=  (this->PhaseSynchronizationTimeVector->VecData)[i];
                PhaseSyncDOFCounter++;
            }
        }

        if (PhaseSyncDOFCounter == 0)
        {
            return;
        }

        PhaseSyncTimeAverage /= ((double)PhaseSyncDOFCounter);

        for(i = 0; i < this->NumberOfDOFs; i++)
        {
            if ((this->ModifiedSelectionVector->VecData)[i])
            {
                if ( fabs((this->PhaseSynchronizationTimeVector->VecData)[i] - PhaseSyncTimeAverage)
                    > (ABSOLUTE_PHASE_SYNC_EPSILON + RELATIVE_PHASE_SYNC_EPSILON * PhaseSyncTimeAverage) )
                {
                        this->CurrentTrajectoryIsPhaseSynchronized = false;
                        break;
                }
            }
        }
    }

    if (this->CurrentTrajectoryIsPhaseSynchronized)
    {
        this->SynchronizationTime   =   this->MinimumExecutionTimes->VecData[this->GreatestDOFForPhaseSynchronization];
        return;
    }

    // ******************************************************************
    // Decision trees 1B and 1C

    for(i = 0; i < this->NumberOfDOFs; i++)
    {
        if ((this->ModifiedSelectionVector->VecData)[i])
        {
            TypeIIRMLDecisionTree1B(        this->CurrentInputParameters->CurrentPositionVector->VecData        [i]
                                        ,   this->CurrentInputParameters->CurrentVelocityVector->VecData        [i]
                                        ,   this->CurrentInputParameters->TargetPositionVector->VecData         [i]
                                        ,   this->CurrentInputParameters->TargetVelocityVector->VecData         [i]
                                        ,   this->CurrentInputParameters->MaxVelocityVector->VecData            [i]
                                        ,   this->CurrentInputParameters->MaxAccelerationVector->VecData        [i]
                                        ,   &(this->BeginningsOfInoperativeTimeIntervals->VecData               [i]));

            if (this->BeginningsOfInoperativeTimeIntervals->VecData[i] != RML_INFINITY)
            {
                TypeIIRMLDecisionTree1C(        this->CurrentInputParameters->CurrentPositionVector->VecData        [i]
                                            ,   this->CurrentInputParameters->CurrentVelocityVector->VecData        [i]
                                            ,   this->CurrentInputParameters->TargetPositionVector->VecData         [i]
                                            ,   this->CurrentInputParameters->TargetVelocityVector->VecData         [i]
                                            ,   this->CurrentInputParameters->MaxVelocityVector->VecData            [i]
                                            ,   this->CurrentInputParameters->MaxAccelerationVector->VecData        [i]
                                            ,   &(this->EndingsOfInoperativeTimeIntervals->VecData                  [i]));
            }
            else
            {
                this->EndingsOfInoperativeTimeIntervals->VecData[i] =   RML_INFINITY;
            }
        }
        else
        {
            this->BeginningsOfInoperativeTimeIntervals->VecData [i] =   RML_INFINITY;
            this->EndingsOfInoperativeTimeIntervals->VecData    [i] =   RML_INFINITY;
        }
    }

    for(i = 0; i < this->NumberOfDOFs; i++)
    {
        if((this->ModifiedSelectionVector->VecData)[i])
        {
            if ( (this->BeginningsOfInoperativeTimeIntervals->VecData)[i] < (this->MinimumExecutionTimes->VecData)[i] )
            {
                (this->BeginningsOfInoperativeTimeIntervals->VecData)[i]
                    =   (this->MinimumExecutionTimes->VecData)[i];
            }

            if ( (this->EndingsOfInoperativeTimeIntervals->VecData)[i] < (this->BeginningsOfInoperativeTimeIntervals->VecData)[i] )
            {
                (this->EndingsOfInoperativeTimeIntervals->VecData)[i]
                    =   (this->BeginningsOfInoperativeTimeIntervals->VecData)[i]
                    =   ((this->BeginningsOfInoperativeTimeIntervals->VecData)[i]
                        +   (this->EndingsOfInoperativeTimeIntervals->VecData)[i]) * 0.5;

                if ( (this->BeginningsOfInoperativeTimeIntervals->VecData)[i] < (this->MinimumExecutionTimes->VecData)[i] )
                {
                    (this->MinimumExecutionTimes->VecData)[i]
                        =   (this->EndingsOfInoperativeTimeIntervals->VecData)[i];
                    (this->BeginningsOfInoperativeTimeIntervals->VecData)[i]    =   RML_INFINITY;
                    (this->EndingsOfInoperativeTimeIntervals->VecData)[i]       =   RML_INFINITY;
                }
            }

            if ( (this->EndingsOfInoperativeTimeIntervals->VecData)[i] < (this->MinimumExecutionTimes->VecData)[i] )
            {
                (this->BeginningsOfInoperativeTimeIntervals->VecData)[i]    =   RML_INFINITY;
                (this->EndingsOfInoperativeTimeIntervals->VecData)[i]       =   RML_INFINITY;
            }
        }
    }


    //***************
    // Calculate tsync

    // Determine the maximum of the minimal execution times
    for(i = 0; i < this->NumberOfDOFs; i++)
    {
        if(!(this->ModifiedSelectionVector->VecData)[i])
        {
            (this->BeginningsOfInoperativeTimeIntervals->VecData)[i]        = RML_INFINITY;
            (this->EndingsOfInoperativeTimeIntervals->VecData)[i]           = RML_INFINITY;
        }

        (this->ArrayOfSortedTimes->VecData)[i]
                =   (this->BeginningsOfInoperativeTimeIntervals->VecData)[i];

        (this->ArrayOfSortedTimes->VecData)[i + this->NumberOfDOFs]
                =   (this->EndingsOfInoperativeTimeIntervals->VecData)[i];
    }

    // Quicksort sorts all values of AlternativeExecutuionTime independent of the SelectionVector
    // because of that the alternative execution times of all no selected DOFs are set to infinity

    // Sort the alternative execution times
    Quicksort(0, (2 * this->NumberOfDOFs - 1), &((this->ArrayOfSortedTimes->VecData)[0]));

    // Which alternative execution times are bigger than the maximum of the minimal execution times
    for (Counter = 0; Counter < 2 * this->NumberOfDOFs; Counter++)
    {
        if ((this->ArrayOfSortedTimes->VecData)[Counter] > MaximalMinimalExecutionTime)
        {
            break;
        }
    }

    this->SynchronizationTime = MaximalMinimalExecutionTime;

    //calculate the minimal time, which is not in death-zone
    while((IsWithinAnInoperativeTimeInterval(   this->SynchronizationTime
                                            ,   *(this->BeginningsOfInoperativeTimeIntervals)
                                            ,   *(this->EndingsOfInoperativeTimeIntervals)      ) )
            && (Counter < 2 * this->NumberOfDOFs) )
    {
        this->SynchronizationTime = (this->ArrayOfSortedTimes->VecData)[Counter];
        Counter++;
    }

    return;
}


//************************************************************************************
// IsWithinAnInoperativeTimeInterval

bool TypeIIRMLPosition::IsWithinAnInoperativeTimeInterval(      const double            &SynchronizationTimeCandidate
                                                            ,   const RMLDoubleVector   &MaximalExecutionTime
                                                            ,   const RMLDoubleVector   &AlternativeExecutionTime) const
{
    unsigned int            i;

    for (i = 0; i < this->NumberOfDOFs; i++)
    {
        if ((this->ModifiedSelectionVector->VecData)[i])
        {
            if (    ((MaximalExecutionTime.VecData)[i] < SynchronizationTimeCandidate)
                &&  (SynchronizationTimeCandidate < (AlternativeExecutionTime.VecData)[i]) )
            {
                return(true);
            }
        }
    }
    return(false);
}

