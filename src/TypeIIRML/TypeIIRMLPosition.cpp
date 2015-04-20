//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLPosition.cpp
//!
//! \brief
//! Main implementation file for the Type II On-Line Trajectory
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
#include <TypeIIRMLStep1Profiles.h>
#include <TypeIIRMLMath.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <RMLVector.h>
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>

using namespace TypeIIRMLMath;


//****************************************************************************
// TypeIIRMLPosition()

TypeIIRMLPosition::TypeIIRMLPosition(       const unsigned int  &DegreesOfFreedom
                                       ,    const double        &CycleTimeInSeconds)
{
    this->CurrentTrajectoryIsPhaseSynchronized          =   false                                               ;
    this->CurrentTrajectoryIsNotSynchronized            =   false                                               ;
    this->CalculatePositionalExtremsFlag                =   false                                               ;

    this->ReturnValue                                   =   ReflexxesAPI::RML_ERROR                             ;

    this->NumberOfDOFs                                  =   DegreesOfFreedom                                    ;
    this->GreatestDOFForPhaseSynchronization            =   0                                                   ;
    this->MotionProfileForPhaseSynchronization          =   TypeIIRMLMath::Step1_Undefined                      ;

    this->CycleTime                                     =   CycleTimeInSeconds                                  ;
    this->SynchronizationTime                           =   0.0                                                 ;
    this->InternalClockInSeconds                        =   0.0                                                 ;

    this->PhaseSynchronizationMagnitude                 =   TypeIIRMLPosition::UNDEFINED                        ;

    this->ModifiedSelectionVector                       =   new RMLBoolVector               (this->NumberOfDOFs);

    this->UsedStep1AProfiles                            =   new RMLVector<Step1_Profile>    (this->NumberOfDOFs);

    this->StoredTargetPosition                          =   new RMLDoubleVector             (this->NumberOfDOFs);
    this->MinimumExecutionTimes                         =   new RMLDoubleVector             (this->NumberOfDOFs);
    this->BeginningsOfInoperativeTimeIntervals          =   new RMLDoubleVector             (this->NumberOfDOFs);
    this->EndingsOfInoperativeTimeIntervals             =   new RMLDoubleVector             (this->NumberOfDOFs);
    this->PhaseSynchronizationReferenceVector           =   new RMLDoubleVector             (this->NumberOfDOFs);
    this->PhaseSynchronizationCurrentPositionVector     =   new RMLDoubleVector             (this->NumberOfDOFs);
    this->PhaseSynchronizationTargetPositionVector      =   new RMLDoubleVector             (this->NumberOfDOFs);
    this->PhaseSynchronizationPositionDifferenceVector  =   new RMLDoubleVector             (this->NumberOfDOFs);
    this->PhaseSynchronizationCurrentVelocityVector     =   new RMLDoubleVector             (this->NumberOfDOFs);
    this->PhaseSynchronizationTargetVelocityVector      =   new RMLDoubleVector             (this->NumberOfDOFs);
    this->PhaseSynchronizationMaxVelocityVector         =   new RMLDoubleVector             (this->NumberOfDOFs);
    this->PhaseSynchronizationMaxAccelerationVector     =   new RMLDoubleVector             (this->NumberOfDOFs);
    this->PhaseSynchronizationTimeVector                =   new RMLDoubleVector             (this->NumberOfDOFs);
    this->PhaseSynchronizationCheckVector               =   new RMLDoubleVector             (this->NumberOfDOFs);

    this->ArrayOfSortedTimes                            =   new RMLDoubleVector         (2 * this->NumberOfDOFs);
    this->ZeroVector                                    =   new RMLDoubleVector             (this->NumberOfDOFs);

    this->OldInputParameters                            =   new RMLPositionInputParameters  (this->NumberOfDOFs);
    this->CurrentInputParameters                        =   new RMLPositionInputParameters  (this->NumberOfDOFs);

    this->OutputParameters                              =   new RMLPositionOutputParameters (this->NumberOfDOFs);

    this->VelocityInputParameters                       =   new RMLVelocityInputParameters  (this->NumberOfDOFs);

    this->VelocityOutputParameters                      =   new RMLVelocityOutputParameters (this->NumberOfDOFs);

    this->RMLVelocityObject                             =   new TypeIIRMLVelocity(      this->NumberOfDOFs
                                                                                    ,   this->CycleTime     )   ;

    this->Polynomials                                   =   new MotionPolynomials           [this->NumberOfDOFs];

    this->ZeroVector->Set(0.0);
}


//****************************************************************************
// ~TypeIIRMLPosition()

TypeIIRMLPosition::~TypeIIRMLPosition(void)
{
    delete      this->OldInputParameters                            ;
    delete      this->CurrentInputParameters                        ;
    delete      this->OutputParameters                              ;
    delete      this->RMLVelocityObject                             ;
    delete      this->ModifiedSelectionVector                       ;
    delete      this->UsedStep1AProfiles                            ;
    delete      this->StoredTargetPosition                          ;
    delete      this->MinimumExecutionTimes                         ;
    delete      this->BeginningsOfInoperativeTimeIntervals          ;
    delete      this->EndingsOfInoperativeTimeIntervals             ;
    delete      this->PhaseSynchronizationReferenceVector           ;
    delete      this->PhaseSynchronizationPositionDifferenceVector  ;
    delete      this->PhaseSynchronizationCurrentPositionVector     ;
    delete      this->PhaseSynchronizationTargetPositionVector      ;
    delete      this->PhaseSynchronizationCurrentVelocityVector     ;
    delete      this->PhaseSynchronizationTargetVelocityVector      ;
    delete      this->PhaseSynchronizationMaxVelocityVector         ;
    delete      this->PhaseSynchronizationMaxAccelerationVector     ;
    delete      this->PhaseSynchronizationTimeVector                ;
    delete      this->PhaseSynchronizationCheckVector               ;
    delete      this->ArrayOfSortedTimes                            ;
    delete      this->ZeroVector                                    ;
    delete      this->VelocityInputParameters                       ;
    delete      this->VelocityOutputParameters                      ;

    delete[]    (MotionPolynomials*)this->Polynomials               ;

    this->OldInputParameters                            =   NULL    ;
    this->CurrentInputParameters                        =   NULL    ;
    this->OutputParameters                              =   NULL    ;
    this->RMLVelocityObject                             =   NULL    ;
    this->ModifiedSelectionVector                       =   NULL    ;
    this->UsedStep1AProfiles                            =   NULL    ;
    this->StoredTargetPosition                          =   NULL    ;
    this->MinimumExecutionTimes                         =   NULL    ;
    this->BeginningsOfInoperativeTimeIntervals          =   NULL    ;
    this->EndingsOfInoperativeTimeIntervals             =   NULL    ;
    this->PhaseSynchronizationReferenceVector           =   NULL    ;
    this->PhaseSynchronizationCurrentPositionVector     =   NULL    ;
    this->PhaseSynchronizationTargetPositionVector      =   NULL    ;
    this->PhaseSynchronizationPositionDifferenceVector  =   NULL    ;
    this->PhaseSynchronizationCurrentVelocityVector     =   NULL    ;
    this->PhaseSynchronizationTargetVelocityVector      =   NULL    ;
    this->PhaseSynchronizationMaxVelocityVector         =   NULL    ;
    this->PhaseSynchronizationMaxAccelerationVector     =   NULL    ;
    this->PhaseSynchronizationTimeVector                =   NULL    ;
    this->PhaseSynchronizationCheckVector               =   NULL    ;
    this->ArrayOfSortedTimes                            =   NULL    ;
    this->ZeroVector                                    =   NULL    ;
    this->VelocityInputParameters                       =   NULL    ;
    this->VelocityOutputParameters                      =   NULL    ;

    this->Polynomials                                   =   NULL    ;
}


//****************************************************************************
// GetNextStateOfMotion()

int TypeIIRMLPosition::GetNextStateOfMotion(    const RMLPositionInputParameters    &InputValues
                                             ,  RMLPositionOutputParameters         *OutputValues
                                             ,  const RMLPositionFlags              &Flags)
{
    bool                        StartANewCalculation            =   false;

    unsigned int                i                               =   0;

    if  (   (OutputValues   ==  NULL)
        ||  (&InputValues   ==  NULL)
        ||  (&Flags         ==  NULL)   )
    {
        this->ReturnValue   =   ReflexxesAPI::RML_ERROR_NULL_POINTER;
        return(this->ReturnValue);
    }

    if (    (this->NumberOfDOFs != InputValues.GetNumberOfDOFs())
        ||  (this->NumberOfDOFs != OutputValues->GetNumberOfDOFs()) )
    {
        FallBackStrategy(       InputValues
                            ,   this->OutputParameters
                            ,   Flags);

        *OutputValues       =   *(this->OutputParameters);
        this->ReturnValue   =   ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS;
        return(this->ReturnValue);
    }

    *(this->CurrentInputParameters) =   InputValues;

    this->CalculatePositionalExtremsFlag    = Flags.EnableTheCalculationOfTheExtremumMotionStates;

    if (    (this->ReturnValue                              ==  ReflexxesAPI::RML_FINAL_STATE_REACHED)
        &&  (Flags.BehaviorAfterFinalStateOfMotionIsReached ==  RMLPositionFlags::RECOMPUTE_TRAJECTORY) )
    {
        StartANewCalculation    =   true;
    }
    else
    {
        StartANewCalculation    =   false;
    }

    //StartANewCalculation  =   true;   //! \todo Remove (this is for debuggin only)

    if (Flags   !=  this->OldFlags)
    {
        StartANewCalculation    =   true;
    }

    if (!StartANewCalculation)
    {
        if (        *(this->CurrentInputParameters->SelectionVector)
                !=
                    *(this->OldInputParameters->SelectionVector) )
        {
            StartANewCalculation = true;
        }
        else
        {
            for (i = 0; i < this->NumberOfDOFs; i++)
            {
                if ((this->CurrentInputParameters->SelectionVector->VecData)[i])
                {
                    if (!(  IsInputEpsilonEqual(
                            (this->CurrentInputParameters->CurrentVelocityVector->VecData)[i]
                        ,   (this->OutputParameters->NewVelocityVector->VecData)[i])
                            &&  IsInputEpsilonEqual(
                            (this->CurrentInputParameters->MaxAccelerationVector->VecData)[i]
                        ,   (this->OldInputParameters->MaxAccelerationVector->VecData)[i])
                            &&  IsInputEpsilonEqual(
                            (this->CurrentInputParameters->MaxVelocityVector->VecData)[i]
                        ,   (this->OldInputParameters->MaxVelocityVector->VecData)[i])
                            &&  IsInputEpsilonEqual(
                            (this->CurrentInputParameters->TargetVelocityVector->VecData)[i]
                        ,   (this->OldInputParameters->TargetVelocityVector->VecData)[i])
                            &&  IsInputEpsilonEqual(
                            ((this->CurrentInputParameters->TargetPositionVector->VecData)[i]
                                -   (this->CurrentInputParameters->CurrentPositionVector->VecData)[i])
                        ,   ((this->OldInputParameters->TargetPositionVector->VecData)[i]
                                -   (this->OutputParameters->NewPositionVector->VecData)[i]))))
                    {
                        StartANewCalculation = true;
                        break;
                    }
                }
            }
        }
    }

    if (    (StartANewCalculation)
        ||  ( ( this->ReturnValue != ReflexxesAPI::RML_WORKING)
                &&  ( this->ReturnValue != ReflexxesAPI::RML_FINAL_STATE_REACHED) ) )
    {
        this->InternalClockInSeconds = this->CycleTime;

        // if the values have changed, we have to start a
        // trajectory computation
        StartANewCalculation = true;

        this->SynchronizationTime = 0.0;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if ((this->CurrentInputParameters->SelectionVector->VecData)[i])
            {
                if  (   (   fabs((this->CurrentInputParameters->TargetVelocityVector->VecData)[i])
                        >
                        (this->CurrentInputParameters->MaxVelocityVector->VecData)[i]       )
                    ||  ((this->CurrentInputParameters->MaxVelocityVector->VecData)[i]
                        <=
                        0.0)
                    ||  ((this->CurrentInputParameters->MaxAccelerationVector->VecData)[i]
                        <=
                        0.0)    )
                {
                    FallBackStrategy(       InputValues
                                        ,   this->OutputParameters
                                        ,   Flags);

                    *OutputValues       =   *(this->OutputParameters);
                    this->ReturnValue   =   ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES;
                    return(this->ReturnValue);
                }
            }
        }
    }
    else
    {
        StartANewCalculation    =   false;
    }

    // From here on, we know whether a new calculation has to be performed or not

    *(this->OldInputParameters)     =   InputValues ;
    this->OldFlags                  =   Flags       ;

    if (StartANewCalculation)
    {
        *(this->StoredTargetPosition)   =   *(this->CurrentInputParameters->TargetPositionVector);

        this->CompareInitialAndTargetStateofMotion();

        this->CurrentTrajectoryIsPhaseSynchronized  =       ((  Flags.SynchronizationBehavior == RMLFlags::ONLY_PHASE_SYNCHRONIZATION           )
                                                        ||  (   Flags.SynchronizationBehavior == RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE    ));

        this->CurrentTrajectoryIsNotSynchronized    =       (   Flags.SynchronizationBehavior == RMLFlags::NO_SYNCHRONIZATION                   );

        // Within the call of the next method, the selection vector
        // (TypeIIRMLPosition::CurrentInputParameters->SelectionVector)
        // becomes modified to (TypeIIRMLPosition::ModifiedSelectionVector),
        // which only contains the DOFs, for which a trajectory has to be
        // calculated. In particular, this is important for the case of
        // phase-synchronization: If a DOF has already reached its target
        // state of motion AND if its target velocity is zero, it will not
        // be considered for further calculations.

        Step1();

        if (    (Flags.SynchronizationBehavior == RMLFlags::ONLY_PHASE_SYNCHRONIZATION)
            &&  (!(this->CurrentTrajectoryIsPhaseSynchronized)) )
        {
            FallBackStrategy(       InputValues
                                ,   this->OutputParameters
                                ,   Flags);

            *OutputValues       =   *(this->OutputParameters);
            if (InputValues.CheckForValidity())
            {
                this->ReturnValue   =   ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION;
            }
            else
            {
                this->ReturnValue   =   ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES;
            }
            return(this->ReturnValue);
        }

        if ( this->SynchronizationTime > RML_MAX_EXECUTION_TIME)
        {
            FallBackStrategy(       InputValues
                                ,   this->OutputParameters
                                ,   Flags);

            *OutputValues       =   *(this->OutputParameters);
            if (InputValues.CheckForValidity())
            {
                this->ReturnValue   =   ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG;
            }
            else
            {
                this->ReturnValue   =   ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES;
            }
            return(this->ReturnValue);
        }

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            (this->Polynomials)[i].ValidPolynomials = 0;
        }

        if  (   (Flags.SynchronizationBehavior          !=  RMLFlags::NO_SYNCHRONIZATION)
            &&  (InputValues.MinimumSynchronizationTime >   this->SynchronizationTime   ))
        {
            for (i = 0; i < 2 * this->NumberOfDOFs; i++)
            {
                if ((this->ArrayOfSortedTimes->VecData)[i] > InputValues.MinimumSynchronizationTime)
                {
                    break;
                }
            }

            this->SynchronizationTime = InputValues.MinimumSynchronizationTime;

            //calculate the minimal time, which is not in death-zone
            while((IsWithinAnInoperativeTimeInterval(   this->SynchronizationTime
                                                    ,   *(this->BeginningsOfInoperativeTimeIntervals)
                                                    ,   *(this->EndingsOfInoperativeTimeIntervals)      ) )
                    && (i < 2 * this->NumberOfDOFs) )
            {
                this->SynchronizationTime = (this->ArrayOfSortedTimes->VecData)[i];
                i++;
            }
        }

        Step2();
    }
    else
    {
        this->InternalClockInSeconds                    +=  this->CycleTime;
        this->SynchronizationTime                       -=  this->CycleTime;

        if (this->SynchronizationTime < 0.0)
        {
            this->SynchronizationTime = 0.0;
        }
    }

    if (this->GetNumberOfSelectedDOFs(*(this->ModifiedSelectionVector)) == 0)
    {
        this->SynchronizationTime   =   0.0;
        if (Flags.SynchronizationBehavior == RMLFlags::ONLY_TIME_SYNCHRONIZATION)
        {
            this->CurrentTrajectoryIsPhaseSynchronized  =   false;
        }
        else
        {
            this->CurrentTrajectoryIsPhaseSynchronized  =   true;
        }
    }

    this->ReturnValue   =   Step3(      this->InternalClockInSeconds
                                    ,   this->OutputParameters      );

    this->OutputParameters->ANewCalculationWasPerformed =    StartANewCalculation;

    this->OutputParameters->TrajectoryIsPhaseSynchronized   =   this->CurrentTrajectoryIsPhaseSynchronized;

    if (this->CurrentTrajectoryIsNotSynchronized)
    {
        this->OutputParameters->SynchronizationTime             =   0.0;
        this->OutputParameters->DOFWithTheGreatestExecutionTime =   this->GreatestDOFForPhaseSynchronization;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if (this->CurrentInputParameters->SelectionVector->VecData[i])
            {
                this->OutputParameters->ExecutionTimes->VecData[i]  =   (this->MinimumExecutionTimes->VecData)[i]
                                                                        -   this->InternalClockInSeconds
                                                                        +   this->CycleTime;

                if (this->OutputParameters->ExecutionTimes->VecData[i] < 0.0)
                {
                    this->OutputParameters->ExecutionTimes->VecData[i]  =   0.0;
                }
            }
            else
            {
                this->OutputParameters->ExecutionTimes->VecData[i]  =   0.0;
            }
        }
    }
    else
    {
        this->OutputParameters->SynchronizationTime             =   this->SynchronizationTime;
        this->OutputParameters->DOFWithTheGreatestExecutionTime =   0;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if (this->CurrentInputParameters->SelectionVector->VecData[i])
            {
                this->OutputParameters->ExecutionTimes->VecData[i]  =   this->SynchronizationTime;
            }
            else
            {
                this->OutputParameters->ExecutionTimes->VecData[i]  =   0.0;
            }
        }
    }

    if (this->CalculatePositionalExtremsFlag)
    {
        this->CalculatePositionalExtrems(       this->InternalClockInSeconds - this->CycleTime
                                            ,   this->OutputParameters                          );
    }
    else
    {
        this->SetPositionalExtremsToZero(this->OutputParameters);
    }

    for (i = 0; i < this->NumberOfDOFs; i++)
    {
        if ( (this->ModifiedSelectionVector->VecData)[i] )
        {
                (this->OutputParameters->NewPositionVector->VecData)[i]
                    =   (this->CurrentInputParameters->TargetPositionVector->VecData)[i]
                            -   ((this->StoredTargetPosition->VecData)[i]
                            -   (this->OutputParameters->NewPositionVector->VecData)[i] );
        }
    }

    if ((this->ReturnValue == ReflexxesAPI::RML_FINAL_STATE_REACHED) && (StartANewCalculation))
    {
        this->OutputParameters->SynchronizationTime =   this->MinimumExecutionTimes->VecData[this->GreatestDOFForPhaseSynchronization];
    }

    *OutputValues   =   *(this->OutputParameters);

    return(this->ReturnValue);
}


//****************************************************************************
// GetNextStateOfMotionAtTime()

int TypeIIRMLPosition::GetNextStateOfMotionAtTime(      const double                        &TimeValueInSeconds
                                                    ,   RMLPositionOutputParameters         *OutputValues       ) const
{
    unsigned int                i               =   0;

    int                         ReturnValue     =   ReflexxesAPI::RML_ERROR;

    double                      InternalTime    =       TimeValueInSeconds
                                                    +   this->InternalClockInSeconds
                                                    -   this->CycleTime;

    if (    (   this->ReturnValue   !=  ReflexxesAPI::RML_WORKING               )
        &&  (   this->ReturnValue   !=  ReflexxesAPI::RML_FINAL_STATE_REACHED   )   )
    {
        return(this->ReturnValue);
    }

    if (    (   TimeValueInSeconds  <   0.0                         )
        ||  (   InternalTime        >   RML_MAX_EXECUTION_TIME      )   )
    {
        return(ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE);
    }

    if (    OutputValues    ==  NULL    )
    {
        return(ReflexxesAPI::RML_ERROR_NULL_POINTER);
    }

    if (OutputValues->NumberOfDOFs  !=  this->NumberOfDOFs)
    {
        return(ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS);
    }

    OutputValues->ANewCalculationWasPerformed   =   false;

    ReturnValue =   Step3(      InternalTime
                            ,   OutputValues);

    OutputValues->TrajectoryIsPhaseSynchronized =   this->CurrentTrajectoryIsPhaseSynchronized;

    if (this->CurrentTrajectoryIsNotSynchronized)
    {
        OutputValues->SynchronizationTime               =   0.0;
        OutputValues->DOFWithTheGreatestExecutionTime   =   this->GreatestDOFForPhaseSynchronization;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if (this->CurrentInputParameters->SelectionVector->VecData[i])
            {
                OutputValues->ExecutionTimes->VecData[i]    =   (this->MinimumExecutionTimes->VecData)[i]
                                                                -   TimeValueInSeconds;

                if (OutputValues->ExecutionTimes->VecData[i] < 0.0)
                {
                    OutputValues->ExecutionTimes->VecData[i]    =   0.0;
                }
            }
            else
            {
                OutputValues->ExecutionTimes->VecData[i]    =   0.0;
            }
        }
    }
    else
    {
        OutputValues->SynchronizationTime               =   this->SynchronizationTime - TimeValueInSeconds;
        OutputValues->DOFWithTheGreatestExecutionTime   =   0;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if (this->CurrentInputParameters->SelectionVector->VecData[i])
            {
                OutputValues->ExecutionTimes->VecData[i]    =   this->SynchronizationTime
                                                                -   TimeValueInSeconds;

                if (OutputValues->ExecutionTimes->VecData[i] < 0.0)
                {
                    OutputValues->ExecutionTimes->VecData[i]    =   0.0;
                }
            }
            else
            {
                OutputValues->ExecutionTimes->VecData[i]    =   0.0;
            }
        }
    }

    if (this->CalculatePositionalExtremsFlag)
    {
        this->CalculatePositionalExtrems(       InternalTime
                                            ,   OutputValues    );
    }
    else
    {
        this->SetPositionalExtremsToZero(OutputValues);
    }

    for (i = 0; i < this->NumberOfDOFs; i++)
    {
        if ( (this->ModifiedSelectionVector->VecData)[i] )
        {
            (OutputValues->NewPositionVector->VecData)[i]
                =   (this->CurrentInputParameters->TargetPositionVector->VecData)[i]
                    -   ((this->StoredTargetPosition->VecData)[i]
                    -   (OutputValues->NewPositionVector->VecData)[i] );
        }
    }

    return(ReturnValue);
}

//****************************************************************************
// GetNumberOfSelectedDOFs()

unsigned int TypeIIRMLPosition::GetNumberOfSelectedDOFs(const RMLBoolVector &BoolVector) const
{
    unsigned int        i           =   0
                    ,   DOFCounter  =   0;

    for (i = 0; i < this->NumberOfDOFs; i++)
    {
        if ((BoolVector.VecData)[i])
        {
            DOFCounter++;
        }
    }

    return(DOFCounter);
}


//****************************************************************************
// CompareInitialAndTargetStateofMotion()

void TypeIIRMLPosition::CompareInitialAndTargetStateofMotion(void)
{

    unsigned int        i           =   0;

    for (i = 0; i < this->NumberOfDOFs; i++)
    {
        if ((this->CurrentInputParameters->SelectionVector->VecData)[i])
        {
            if (!(( (this->CurrentInputParameters->CurrentPositionVector->VecData)      [i]
            ==  (this->CurrentInputParameters->TargetPositionVector->VecData)       [i] )
                &&  (   (this->CurrentInputParameters->CurrentVelocityVector->VecData)      [i]
            ==  (this->CurrentInputParameters->TargetVelocityVector->VecData)       [i] )
                &&  (   (this->CurrentInputParameters->TargetVelocityVector->VecData)       [i]
            !=  0.0 )))
            {
                return;
            }
        }
    }

    for (i = 0; i < this->NumberOfDOFs; i++)
    {
        if ((this->CurrentInputParameters->SelectionVector->VecData)[i])
        {
            if ((this->CurrentInputParameters->CurrentPositionVector->VecData)[i] != 0.0)
            {
                (this->CurrentInputParameters->CurrentPositionVector->VecData)[i]
                *=  1.0
                    + FSign((this->CurrentInputParameters->CurrentVelocityVector->VecData)[i])
                    *   RML_ADDITIONAL_RELATIVE_POSITION_ERROR_IN_CASE_OF_EQUALITY;
            }
            else
            {
                (this->CurrentInputParameters->CurrentPositionVector->VecData)[i]
                +=  FSign((this->CurrentInputParameters->CurrentVelocityVector->VecData)[i])
                    *   RML_ADDITIONAL_ABSOLUTE_POSITION_ERROR_IN_CASE_OF_EQUALITY;
            }
        }
    }

    return;
}
