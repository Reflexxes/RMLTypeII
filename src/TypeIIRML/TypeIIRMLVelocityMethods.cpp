//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLVelocityMethods.cpp
//!
//! \brief
//! Implementation file for specific tools used for the velocity-based
//! Type II On-Line Trajectory Generation algorithm
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
#include <math.h>


using namespace TypeIIRMLMath;


//****************************************************************************
// CalculateExecutionTimes()

void TypeIIRMLVelocity::CalculateExecutionTimes(void)
{
    unsigned int            i                                   =   0;

    for (i = 0; i < this->NumberOfDOFs; i++)
    {
        Polynomials[i].ValidPolynomials             =   0;

        if ((this->CurrentInputParameters->SelectionVector->VecData)[i])
        {
            (this->ExecutionTimes->VecData)[i]  =   fabs(       (this->CurrentInputParameters->CurrentVelocityVector->VecData   )[i]
                                                            -   (this->CurrentInputParameters->TargetVelocityVector->VecData    )[i])
                                                    /   this->CurrentInputParameters->MaxAccelerationVector->VecData[i];
        }
    }
    return;
}


//****************************************************************************
// ComputeTrajectoryParameters()

void TypeIIRMLVelocity::ComputeTrajectoryParameters(void)
{
    unsigned int            i                                   =   0;

    double                  TimeForFirstSegment                 =   0.0;

    for (i = 0; i < this->NumberOfDOFs; i++)
    {
        if ((this->CurrentInputParameters->SelectionVector->VecData)[i])
        {
            (this->Polynomials)[i].ValidPolynomials =   0       ;

            TimeForFirstSegment                     =   fabs(       (this->CurrentInputParameters->CurrentVelocityVector->VecData   )[i]
                                                                -   (this->CurrentInputParameters->TargetVelocityVector->VecData    )[i])
                                                        /   this->CurrentInputParameters->MaxAccelerationVector->VecData[i];

            // if vi <= vtrgt
            if (Decision_V___001(       (this->CurrentInputParameters->CurrentVelocityVector->VecData   )[i]
                                    ,   (this->CurrentInputParameters->TargetVelocityVector->VecData    )[i]))
            {
                (this->Polynomials)[i].PositionPolynomial[(this->Polynomials)[i].ValidPolynomials].SetCoefficients((0.5 * (this->CurrentInputParameters->MaxAccelerationVector->VecData)[i]), (this->CurrentInputParameters->CurrentVelocityVector->VecData)[i], (this->CurrentInputParameters->CurrentPositionVector->VecData)[i], 0.0);
                (this->Polynomials)[i].VelocityPolynomial[(this->Polynomials)[i].ValidPolynomials].SetCoefficients(0.0, (this->CurrentInputParameters->MaxAccelerationVector->VecData)[i], (this->CurrentInputParameters->CurrentVelocityVector->VecData)[i], 0.0);
                (this->Polynomials)[i].AccelerationPolynomial[(this->Polynomials)[i].ValidPolynomials].SetCoefficients(0.0, 0.0, (this->CurrentInputParameters->MaxAccelerationVector->VecData)[i], 0.0);

                (this->CurrentInputParameters->CurrentPositionVector->VecData)[i]       +=  (this->CurrentInputParameters->CurrentVelocityVector->VecData   )[i]
                                                                                            *   TimeForFirstSegment
                                                                                            +   0.5
                                                                                            *   (this->CurrentInputParameters->MaxAccelerationVector->VecData)[i]
                                                                                            *   pow2(TimeForFirstSegment);

                (this->CurrentInputParameters->CurrentVelocityVector->VecData)[i]       +=  (this->CurrentInputParameters->MaxAccelerationVector->VecData)[i]
                                                                                            *   TimeForFirstSegment;
            }
            else
            {
                (this->Polynomials)[i].PositionPolynomial[(this->Polynomials)[i].ValidPolynomials].SetCoefficients((-0.5 * (this->CurrentInputParameters->MaxAccelerationVector->VecData)[i]), (this->CurrentInputParameters->CurrentVelocityVector->VecData)[i], (this->CurrentInputParameters->CurrentPositionVector->VecData)[i], 0.0);
                (this->Polynomials)[i].VelocityPolynomial[(this->Polynomials)[i].ValidPolynomials].SetCoefficients(0.0, -(this->CurrentInputParameters->MaxAccelerationVector->VecData)[i], (this->CurrentInputParameters->CurrentVelocityVector->VecData)[i], 0.0);
                (this->Polynomials)[i].AccelerationPolynomial[(this->Polynomials)[i].ValidPolynomials].SetCoefficients(0.0, 0.0, -(this->CurrentInputParameters->MaxAccelerationVector->VecData)[i], 0.0);

                (this->CurrentInputParameters->CurrentPositionVector->VecData)[i]       +=  (this->CurrentInputParameters->CurrentVelocityVector->VecData   )[i]
                                                                                            *   TimeForFirstSegment
                                                                                            -   0.5
                                                                                        *   (this->CurrentInputParameters->MaxAccelerationVector->VecData)[i]
                                                                                            *   pow2(TimeForFirstSegment);

                (this->CurrentInputParameters->CurrentVelocityVector->VecData)[i]       +=  -(this->CurrentInputParameters->MaxAccelerationVector->VecData)[i]
                                                                                            *   TimeForFirstSegment;
            }

            (this->Polynomials)[i].PolynomialTimes[(this->Polynomials)[i].ValidPolynomials] = TimeForFirstSegment;
            (this->Polynomials)[i].ValidPolynomials++;

            (this->OutputParameters->ExecutionTimes->VecData)[i]                            =   TimeForFirstSegment;

            (this->OutputParameters->PositionValuesAtTargetVelocity->VecData)[i]            =   (this->CurrentInputParameters->CurrentPositionVector->VecData)[i];

            if (TimeForFirstSegment > this->SynchronizationTime)
            {
                this->OutputParameters->DOFWithTheGreatestExecutionTime                     =   i;
                this->SynchronizationTime                                                   =   TimeForFirstSegment;
            }

            // final segment to hold the velocity

            (this->Polynomials)[i].PositionPolynomial[(this->Polynomials)[i].ValidPolynomials].SetCoefficients(0.0, (this->CurrentInputParameters->CurrentVelocityVector->VecData)[i], (this->CurrentInputParameters->CurrentPositionVector->VecData)[i], TimeForFirstSegment);
            (this->Polynomials)[i].VelocityPolynomial[(this->Polynomials)[i].ValidPolynomials].SetCoefficients(0.0, 0.0, (this->CurrentInputParameters->CurrentVelocityVector->VecData)[i], TimeForFirstSegment);
            (this->Polynomials)[i].AccelerationPolynomial[(this->Polynomials)[i].ValidPolynomials].SetCoefficients(0.0, 0.0, 0.0, TimeForFirstSegment);

            (this->Polynomials)[i].PolynomialTimes[(this->Polynomials)[i].ValidPolynomials] = TimeForFirstSegment + RML_INFINITY;
            (this->Polynomials)[i].ValidPolynomials++;

            // now, all polynomials are set up, and the desired values can be calculated.

        }
        else
        {
            (this->Polynomials)[i].ValidPolynomials                                     =   0;
        }
    }

    return;
}


//****************************************************************************
// ComputePhaseSynchronizationParameters()

void TypeIIRMLVelocity::ComputePhaseSynchronizationParameters(void)
{
    unsigned int            i                                   =   0;

    double                  VectorStretchFactorMaxAcceleration  =   0.0
                        ,   PhaseSyncTimeAverage                =   0.0
                        ,   PhaseSyncDOFCounter                 =   0.0;

    this->SetupPhaseSyncSelectionVector();

    if (this->CurrentTrajectoryIsPhaseSynchronized)
    {
        this->CurrentTrajectoryIsPhaseSynchronized  = this->IsPhaseSynchronizationPossible();

        if ( (this->CurrentTrajectoryIsPhaseSynchronized)
            &&  (fabs((this->PhaseSynchronizationReferenceVector->VecData)[
                    this->OutputParameters->DOFWithTheGreatestExecutionTime])
             > ABSOLUTE_PHASE_SYNC_EPSILON) )
        {
            VectorStretchFactorMaxAcceleration  =   (this->CurrentInputParameters->MaxAccelerationVector->VecData)[this->OutputParameters->DOFWithTheGreatestExecutionTime]
                                                    / fabs((this->PhaseSynchronizationReferenceVector->VecData)[this->OutputParameters->DOFWithTheGreatestExecutionTime]);

            for(i = 0; i < this->NumberOfDOFs; i++)
            {
                if((this->PhaseSyncSelectionVector->VecData)[i])
                {
                    (this->ExecutionTimes->VecData)[i]                              =   0.0;

                    (this->PhaseSynchronizationMaxAccelerationVector->VecData)[i]   =   fabs(VectorStretchFactorMaxAcceleration
                                                                                                * (this->PhaseSynchronizationReferenceVector->VecData)[i]);
                    if ( (this->PhaseSynchronizationMaxAccelerationVector->VecData)[i] <= 0.0 )
                    {
                        (this->PhaseSynchronizationMaxAccelerationVector->VecData)[i]   =   POSITIVE_ZERO;
                    }

                    if  ( (this->PhaseSynchronizationMaxAccelerationVector->VecData)[i]
                        > ( (this->CurrentInputParameters->MaxAccelerationVector->VecData)[i] * ( 1.0 + RELATIVE_PHASE_SYNC_EPSILON ) + ABSOLUTE_PHASE_SYNC_EPSILON ) )
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
            for(i = 0; i < this->NumberOfDOFs; i++)
            {
                if((this->PhaseSyncSelectionVector->VecData)[i])
                {
                    (this->ExecutionTimes->VecData)[i]  =   fabs(   (this->CurrentInputParameters->CurrentVelocityVector->VecData)[i]
                                                                    -   (this->CurrentInputParameters->TargetVelocityVector->VecData)[i])
                                                            /   (this->PhaseSynchronizationMaxAccelerationVector->VecData)[i];
                }
            }

            PhaseSyncTimeAverage    =   0.0;
            PhaseSyncDOFCounter     =   0.0;

            for(i = 0; i < this->NumberOfDOFs; i++)
            {
                if((this->PhaseSyncSelectionVector->VecData)[i])
                {
                    PhaseSyncTimeAverage    +=  (this->ExecutionTimes->VecData)[i];
                    PhaseSyncDOFCounter     +=  1.0;
                }
            }

            if (PhaseSyncDOFCounter > 0.0)
            {
                PhaseSyncTimeAverage /= PhaseSyncDOFCounter;

                for(i = 0; i < this->NumberOfDOFs; i++)
                {
                    if ((this->PhaseSyncSelectionVector->VecData)[i])
                    {
                        if ( fabs((this->ExecutionTimes->VecData)[i] - PhaseSyncTimeAverage)
                            > (ABSOLUTE_PHASE_SYNC_EPSILON + RELATIVE_PHASE_SYNC_EPSILON * PhaseSyncTimeAverage) )
                        {
                                this->CurrentTrajectoryIsPhaseSynchronized = false;
                                break;
                        }
                    }
                }
            }
        }
    }

    if (this->CurrentTrajectoryIsPhaseSynchronized)
    {
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if ((this->PhaseSyncSelectionVector->VecData)[i])
            {
                (this->CurrentInputParameters->MaxAccelerationVector->VecData)[i]
                    =   (this->PhaseSynchronizationMaxAccelerationVector->VecData)[i];
            }
        }
    }

    return;
}


//****************************************************************************
// ComputeAndSetOutputParameters()

int TypeIIRMLVelocity::ComputeAndSetOutputParameters(       const double                    &TimeValueInSeconds
                                                        ,   RMLVelocityOutputParameters     *OP                 ) const
{
    unsigned int            i                           =   0;

    int                     j                           =   0
                        ,   ReturnValueForThisMethod    =   ReflexxesAPI::RML_FINAL_STATE_REACHED;

    // calculate the new state of motion

    for (i = 0; i < this->NumberOfDOFs; i++)
    {
        if ((this->CurrentInputParameters->SelectionVector->VecData)[i])
        {
            j = 0;

            while ( (TimeValueInSeconds > (((this->Polynomials)[i].PolynomialTimes)[j])) && (j < MAXIMAL_NO_OF_POLYNOMIALS))
            {
                j++;
            }

            (OP->NewPositionVector->VecData)[i]
                =   (this->Polynomials)[i].PositionPolynomial[j].CalculateValue(TimeValueInSeconds);
            (OP->NewVelocityVector->VecData)[i]
                =   (this->Polynomials)[i].VelocityPolynomial[j].CalculateValue(TimeValueInSeconds);
            (OP->NewAccelerationVector->VecData)[i]
                =   (this->Polynomials)[i].AccelerationPolynomial[j].CalculateValue(TimeValueInSeconds);

            if ( j < ((this->Polynomials)[i].ValidPolynomials) - 1)
            {
                ReturnValueForThisMethod = ReflexxesAPI::RML_WORKING;
            }

            (OP->PositionValuesAtTargetVelocity->VecData)[i]
                =   (this->Polynomials)[i].PositionPolynomial[(this->Polynomials)[i].ValidPolynomials - 1].a0;
        }
        else
        {
            (OP->NewPositionVector->VecData)[i]
                =   (this->CurrentInputParameters->CurrentPositionVector->VecData)[i];
            (OP->NewVelocityVector->VecData)[i]
                =   (this->CurrentInputParameters->CurrentVelocityVector->VecData)[i];
            (OP->NewAccelerationVector->VecData)[i]
                =   (this->CurrentInputParameters->CurrentAccelerationVector->VecData)[i];
            (OP->PositionValuesAtTargetVelocity->VecData)[i]
                =   (this->CurrentInputParameters->CurrentPositionVector->VecData)[i];
        }
    }

    return(ReturnValueForThisMethod);
}
