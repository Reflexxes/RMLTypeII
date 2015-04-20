//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLVelocity.cpp
//!
//! \brief
//! Main implementation file for the velocity-based Type II On-Line
//! Trajectory Generation algorithm
//!
//! \details
//! For further information, please refer to the file TypeIIRMLVelocity.h.
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


#include <TypeIIRMLVelocity.h>
#include <TypeIIRMLMath.h>
#include <TypeIIRMLDecisions.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLVector.h>
#include <ReflexxesAPI.h>
#include <RMLVelocityFlags.h>



using namespace TypeIIRMLMath;


//****************************************************************************
// TypeIIRMLVelocity()

TypeIIRMLVelocity::TypeIIRMLVelocity(       const unsigned int  &DegreesOfFreedom
                                       ,    const double        &CycleTimeInSeconds)
{
    this->CurrentTrajectoryIsPhaseSynchronized          =   false                                               ;
    this->CalculatePositionalExtremsFlag                =   false                                               ;
    this->CurrentTrajectoryIsNotSynchronized            =   false                                               ;

    this->ReturnValue                                   =   ReflexxesAPI::RML_ERROR                             ;
    this->DOFWithGreatestExecutionTime                  =   0                                                   ;

    this->NumberOfDOFs                                  =   DegreesOfFreedom                                    ;
    this->CycleTime                                     =   CycleTimeInSeconds                                  ;
    this->InternalClockInSeconds                        =   0.0                                                 ;
    this->SynchronizationTime                           =   0.0                                                 ;

    this->PhaseSyncSelectionVector                      =   new RMLBoolVector               (this->NumberOfDOFs);

    this->ExecutionTimes                                =   new RMLDoubleVector             (this->NumberOfDOFs);
    this->PhaseSynchronizationReferenceVector           =   new RMLDoubleVector             (this->NumberOfDOFs);
    this->PhaseSynchronizationCurrentVelocityVector     =   new RMLDoubleVector             (this->NumberOfDOFs);
    this->PhaseSynchronizationTargetVelocityVector      =   new RMLDoubleVector             (this->NumberOfDOFs);
    this->PhaseSynchronizationMaxAccelerationVector     =   new RMLDoubleVector             (this->NumberOfDOFs);

    this->OldInputParameters                            =   new RMLVelocityInputParameters  (this->NumberOfDOFs);
    this->CurrentInputParameters                        =   new RMLVelocityInputParameters  (this->NumberOfDOFs);

    this->OutputParameters                              =   new RMLVelocityOutputParameters (this->NumberOfDOFs);

    this->Polynomials                                   =   new MotionPolynomials           [this->NumberOfDOFs];
}


//****************************************************************************
// ~TypeIIRMLVelocity()

TypeIIRMLVelocity::~TypeIIRMLVelocity(void)
{
    delete      this->PhaseSyncSelectionVector                      ;

    delete      this->ExecutionTimes                                ;
    delete      this->PhaseSynchronizationReferenceVector           ;
    delete      this->PhaseSynchronizationCurrentVelocityVector     ;
    delete      this->PhaseSynchronizationTargetVelocityVector      ;
    delete      this->PhaseSynchronizationMaxAccelerationVector     ;

    delete      this->OldInputParameters                            ;
    delete      this->CurrentInputParameters                        ;
    delete      this->OutputParameters                              ;

    delete[]    this->Polynomials                                   ;

    this->PhaseSyncSelectionVector                      =   NULL    ;
    this->ExecutionTimes                                =   NULL    ;
    this->PhaseSynchronizationReferenceVector           =   NULL    ;
    this->PhaseSynchronizationCurrentVelocityVector     =   NULL    ;
    this->PhaseSynchronizationTargetVelocityVector      =   NULL    ;
    this->PhaseSynchronizationMaxAccelerationVector     =   NULL    ;

    this->OldInputParameters                            =   NULL    ;
    this->CurrentInputParameters                        =   NULL    ;
    this->OutputParameters                              =   NULL    ;

    this->Polynomials                                   =   NULL    ;
}


//****************************************************************************
// GetNextStateOfMotion()

int TypeIIRMLVelocity::GetNextStateOfMotion(    const RMLVelocityInputParameters    &InputValues
                                             ,  RMLVelocityOutputParameters         *OutputValues
                                             ,  const RMLVelocityFlags              &Flags)
{
    bool                    ErroneousInputValues                =   false
                        ,   StartANewCalculation                =   false   ;

    unsigned int            i                                   =   0       ;

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
        this->ReturnValue   =   ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS;
        return(this->ReturnValue);
    }

    this->CalculatePositionalExtremsFlag    =   Flags.EnableTheCalculationOfTheExtremumMotionStates;

    *(this->CurrentInputParameters) =   InputValues;

    if (Flags   !=  this->OldFlags)
    {
        StartANewCalculation    =   true;
    }

    // check whether parameters have changed
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
                    if (!(      IsInputEpsilonEqual(
                                        (this->CurrentInputParameters->CurrentVelocityVector->VecData)[i]
                                    ,   (this->OutputParameters->NewVelocityVector->VecData)[i])
                            &&  IsInputEpsilonEqual(
                                        (this->CurrentInputParameters->MaxAccelerationVector->VecData)[i]
                                    ,   (this->OldInputParameters->MaxAccelerationVector->VecData)[i])
                            &&  IsInputEpsilonEqual(
                                        (this->CurrentInputParameters->TargetVelocityVector->VecData)[i]
                                    ,   (this->OldInputParameters->TargetVelocityVector->VecData)[i])
                            &&  IsInputEpsilonEqual(
                                        (this->CurrentInputParameters->CurrentPositionVector->VecData)[i]
                                    ,   (this->OutputParameters->NewPositionVector->VecData)[i])))
                    {
                        StartANewCalculation    =   true;
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
        this->InternalClockInSeconds                            =   this->CycleTime;
        StartANewCalculation                                    =   true;
        // if the values have changed, we have to start a
        // trajectory computation

        this->SynchronizationTime                               =   0.0;
    }
    else
    {
        this->InternalClockInSeconds            +=  this->CycleTime;
        this->SynchronizationTime               -= this->CycleTime;

        if (this->SynchronizationTime < 0.0)
        {
            this->SynchronizationTime = 0.0;
        }
    }

    *(this->OldInputParameters)     =   InputValues ;
    this->OldFlags                  =   Flags       ;

    if (StartANewCalculation)
    {
        this->CurrentTrajectoryIsPhaseSynchronized  =       (Flags.SynchronizationBehavior == RMLFlags::ONLY_PHASE_SYNCHRONIZATION)
                                                        ||  (Flags.SynchronizationBehavior == RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE);

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if ((this->CurrentInputParameters->SelectionVector->VecData)[i])
            {
                if (this->CurrentInputParameters->MaxAccelerationVector->VecData[i] <=  0.0)
                {
                    ErroneousInputValues    =   true;
                }
            }
        }

        if (ErroneousInputValues)
        {
            this->FallBackStrategy(     *(this->CurrentInputParameters)
                                    ,   this->OutputParameters      );

            *OutputValues       =   *(this->OutputParameters);
            this->ReturnValue   =   ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES;
            return(this->ReturnValue);
        }

        this->CurrentTrajectoryIsNotSynchronized    =       (   Flags.SynchronizationBehavior == RMLFlags::NO_SYNCHRONIZATION                   );

        this->CurrentTrajectoryIsPhaseSynchronized  =       ((  Flags.SynchronizationBehavior == RMLFlags::ONLY_PHASE_SYNCHRONIZATION           )
                                                        ||  (   Flags.SynchronizationBehavior == RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE    ));


        this->CalculateExecutionTimes();

        this->SynchronizationTime   =   0.0;

        for(i = 0; i < this->NumberOfDOFs; i++)
        {
            if(this->CurrentInputParameters->SelectionVector->VecData[i])
            {
                if ( (this->ExecutionTimes->VecData)[i] > this->SynchronizationTime )
                {
                    this->SynchronizationTime           =   (this->ExecutionTimes->VecData)[i];
                    this->DOFWithGreatestExecutionTime  =   i;
                }
            }
        }

        if  (   (Flags.SynchronizationBehavior          !=  RMLFlags::NO_SYNCHRONIZATION)
            &&  (InputValues.MinimumSynchronizationTime >   this->SynchronizationTime   ))
        {
            this->SynchronizationTime   =   InputValues.MinimumSynchronizationTime;
        }

        if (this->CurrentTrajectoryIsPhaseSynchronized)
        {
            this->ComputePhaseSynchronizationParameters();
        }

        if (    (!this->CurrentTrajectoryIsPhaseSynchronized                            )
            &&  (Flags.SynchronizationBehavior == RMLFlags::ONLY_PHASE_SYNCHRONIZATION  )   )
        {
            this->FallBackStrategy(     *(this->CurrentInputParameters)
                                    ,   this->OutputParameters          );

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

        if (    (Flags.SynchronizationBehavior              ==  RMLFlags::ONLY_TIME_SYNCHRONIZATION         )
            ||  ((Flags.SynchronizationBehavior             ==  RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE )
            &&  (this->CurrentTrajectoryIsPhaseSynchronized ==  false                                       )   )   )
        {
            for (i = 0; i < this->NumberOfDOFs; i++)
            {
                this->CurrentInputParameters->MaxAccelerationVector->VecData[i] =
                    fabs(       (this->CurrentInputParameters->CurrentVelocityVector->VecData   )[i]
                            -   (this->CurrentInputParameters->TargetVelocityVector->VecData    )[i])
                    /   this->SynchronizationTime;
            }
        }

        this->ComputeTrajectoryParameters();
    }

    this->OutputParameters->ANewCalculationWasPerformed =    StartANewCalculation;

    this->ReturnValue   =   this->ComputeAndSetOutputParameters(    this->InternalClockInSeconds
                                                                ,   this->OutputParameters      );

    this->OutputParameters->TrajectoryIsPhaseSynchronized   =   this->CurrentTrajectoryIsPhaseSynchronized;

    if (this->CurrentTrajectoryIsNotSynchronized)
    {
        this->OutputParameters->SynchronizationTime             =   0.0;
        this->OutputParameters->DOFWithTheGreatestExecutionTime =   this->DOFWithGreatestExecutionTime;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if (this->CurrentInputParameters->SelectionVector->VecData[i])
            {
                this->OutputParameters->ExecutionTimes->VecData[i]  =   (this->ExecutionTimes->VecData)[i]
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

    *OutputValues   =   *(this->OutputParameters);

    return(this->ReturnValue);
}


//****************************************************************************
// GetNextStateOfMotionAtTime()

int TypeIIRMLVelocity::GetNextStateOfMotionAtTime(      const double                        &TimeValueInSeconds
                                                    ,   RMLVelocityOutputParameters         *OutputValues       ) const
{
    unsigned int            i                           =   0;

    int                     ReturnValueOfThisMethod     =   ReflexxesAPI::RML_FINAL_STATE_REACHED;

    double                  InternalTime                =       TimeValueInSeconds
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

    ReturnValueOfThisMethod =   this->ComputeAndSetOutputParameters(    InternalTime
                                                                    ,   OutputValues    );

    OutputValues->TrajectoryIsPhaseSynchronized =   this->CurrentTrajectoryIsPhaseSynchronized;

    if (this->CurrentTrajectoryIsNotSynchronized)
    {
        OutputValues->SynchronizationTime               =   0.0;
        OutputValues->DOFWithTheGreatestExecutionTime   =   this->DOFWithGreatestExecutionTime;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if (this->CurrentInputParameters->SelectionVector->VecData[i])
            {
                OutputValues->ExecutionTimes->VecData[i]    =   (this->ExecutionTimes->VecData)[i]
                                                                -   this->InternalClockInSeconds
                                                                +   this->CycleTime
                                                                -   InternalTime;

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
        OutputValues->SynchronizationTime               =   this->SynchronizationTime - InternalTime;
        OutputValues->DOFWithTheGreatestExecutionTime   =   this->DOFWithGreatestExecutionTime;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if (this->CurrentInputParameters->SelectionVector->VecData[i])
            {
                OutputValues->ExecutionTimes->VecData[i]    =   this->SynchronizationTime
                                                                -   InternalTime;

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
                                            ,   OutputValues);
    }
    else
    {
        this->SetPositionalExtremsToZero(OutputValues);
    }

    return(ReturnValueOfThisMethod);
}


//****************************************************************************
// operator = ()

TypeIIRMLVelocity &TypeIIRMLVelocity::operator = (const TypeIIRMLVelocity &TypeIIRMLObject)
{
    unsigned int        i   =   0;

    this->CurrentTrajectoryIsPhaseSynchronized                  =   TypeIIRMLObject.CurrentTrajectoryIsPhaseSynchronized                ;
    this->CalculatePositionalExtremsFlag                        =   TypeIIRMLObject.CalculatePositionalExtremsFlag                      ;
    this->CurrentTrajectoryIsNotSynchronized                    =   TypeIIRMLObject.CurrentTrajectoryIsNotSynchronized                  ;

    this->ReturnValue                                           =   TypeIIRMLObject.ReturnValue                                         ;
    this->DOFWithGreatestExecutionTime                          =   TypeIIRMLObject.DOFWithGreatestExecutionTime                        ;

    this->NumberOfDOFs                                          =   TypeIIRMLObject.NumberOfDOFs                                        ;
    this->CycleTime                                             =   TypeIIRMLObject.CycleTime                                           ;
    this->InternalClockInSeconds                                =   TypeIIRMLObject.InternalClockInSeconds                              ;
    this->SynchronizationTime                                   =   TypeIIRMLObject.SynchronizationTime                                 ;

    *(this->PhaseSyncSelectionVector                        )   =   *(TypeIIRMLObject.PhaseSyncSelectionVector                      )   ;

    *(this->ExecutionTimes                                  )   =   *(TypeIIRMLObject.ExecutionTimes                                )   ;
    *(this->PhaseSynchronizationReferenceVector             )   =   *(TypeIIRMLObject.PhaseSynchronizationReferenceVector           )   ;
    *(this->PhaseSynchronizationCurrentVelocityVector       )   =   *(TypeIIRMLObject.PhaseSynchronizationCurrentVelocityVector     )   ;
    *(this->PhaseSynchronizationTargetVelocityVector        )   =   *(TypeIIRMLObject.PhaseSynchronizationTargetVelocityVector      )   ;
    *(this->PhaseSynchronizationMaxAccelerationVector       )   =   *(TypeIIRMLObject.PhaseSynchronizationMaxAccelerationVector     )   ;

    *(this->OldInputParameters                              )   =   *(TypeIIRMLObject.OldInputParameters                            )   ;
    *(this->CurrentInputParameters                          )   =   *(TypeIIRMLObject.CurrentInputParameters                        )   ;

    *(this->OutputParameters                                )   =   *(TypeIIRMLObject.OutputParameters                              )   ;

    for ( i = 0; i < this->NumberOfDOFs; i++)
    {
        this->Polynomials[i]                                    =   TypeIIRMLObject.Polynomials[i];
    }

    return(*this);
}


//****************************************************************************
// TypeIIRMLVelocity()


TypeIIRMLVelocity::TypeIIRMLVelocity(const TypeIIRMLVelocity &TypeIIRMLObject)
{
    this->PhaseSyncSelectionVector                      =   new RMLBoolVector               (TypeIIRMLObject.NumberOfDOFs)  ;

    this->ExecutionTimes                                =   new RMLDoubleVector             (TypeIIRMLObject.NumberOfDOFs)  ;
    this->PhaseSynchronizationReferenceVector           =   new RMLDoubleVector             (TypeIIRMLObject.NumberOfDOFs)  ;
    this->PhaseSynchronizationCurrentVelocityVector     =   new RMLDoubleVector             (TypeIIRMLObject.NumberOfDOFs)  ;
    this->PhaseSynchronizationTargetVelocityVector      =   new RMLDoubleVector             (TypeIIRMLObject.NumberOfDOFs)  ;
    this->PhaseSynchronizationMaxAccelerationVector     =   new RMLDoubleVector             (TypeIIRMLObject.NumberOfDOFs)  ;

    this->OldInputParameters                            =   new RMLVelocityInputParameters  (TypeIIRMLObject.NumberOfDOFs)  ;
    this->CurrentInputParameters                        =   new RMLVelocityInputParameters  (TypeIIRMLObject.NumberOfDOFs)  ;

    this->OutputParameters                              =   new RMLVelocityOutputParameters (TypeIIRMLObject.NumberOfDOFs)  ;

    this->Polynomials                                   =   new MotionPolynomials           [TypeIIRMLObject.NumberOfDOFs]  ;

    *this                                               =   TypeIIRMLObject                                                 ;
}
