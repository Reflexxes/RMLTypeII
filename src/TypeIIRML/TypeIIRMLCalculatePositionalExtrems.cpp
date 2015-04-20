//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLCalculatePositionalExtrems.cpp
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
#include <TypeIIRMLPolynomial.h>
#include <RMLPositionOutputParameters.h>
#include <RMLPositionInputParameters.h>



using namespace TypeIIRMLMath;


//*******************************************************************************************
// CalculatePositionalExtrems()

void TypeIIRMLPosition::CalculatePositionalExtrems(     const double                &TimeValueInSeconds
                                                    ,   RMLPositionOutputParameters *OP                 ) const
{
    unsigned int            i                           =   0
                        ,   NumberOfRoots               =   0
                        ,   k                           =   0
                        ,   l                           =   0;

    int                     j                           =   0;

    double                  AnalizedPosition            =   0.0
                        ,   TimeOfZeroVelocity1         =   0.0
                        ,   TimeOfZeroVelocity2         =   0.0
                        ,   TimeValueAtExtremumPosition =   0.0;

    for (i = 0; i < this->NumberOfDOFs; i++)
    {
        if ((this->ModifiedSelectionVector->VecData)[i])
        {
            // Initially, set all extrema values to the current position values.
            (OP->MinPosExtremaPositionVectorOnly->VecData)[i]
                =   (OP->NewPositionVector->VecData)[i];
            (OP->MaxPosExtremaPositionVectorOnly->VecData)[i]
                =   (OP->NewPositionVector->VecData)[i];

            // Check all root of the velocity polynomials.
            for (j = 0; j < ((this->Polynomials)[i].ValidPolynomials - 1); j++)
            {
                if ((this->Polynomials)[i].PolynomialTimes[j] > TimeValueInSeconds)
                {
                    if (    Sign(       (j == 0)
                                    ?   ((this->Polynomials)[i].VelocityPolynomial[j].CalculateValue(0.0))
                                    :   ((this->Polynomials)[i].VelocityPolynomial[j].CalculateValue(
                                        (this->Polynomials)[i].PolynomialTimes[j - 1])))
                            !=
                            Sign(       (this->Polynomials)[i].VelocityPolynomial[j].CalculateValue(
                                        (this->Polynomials)[i].PolynomialTimes[j])) )
                    {
                        (this->Polynomials)[i].VelocityPolynomial[j].CalculateRealRoots(    &NumberOfRoots
                                                                                        ,   &TimeOfZeroVelocity1
                                                                                        ,   &TimeOfZeroVelocity2);
                         if ((NumberOfRoots == 1) &&    (TimeOfZeroVelocity1 > TimeValueInSeconds))
                        {
                            AnalizedPosition = (this->Polynomials)[i].PositionPolynomial[j].CalculateValue(TimeOfZeroVelocity1);
                            TimeValueAtExtremumPosition =   TimeOfZeroVelocity1;
                        }
                        else
                        {
                            continue;
                        }

                        if (    (AnalizedPosition > (OP->MaxPosExtremaPositionVectorOnly->VecData)[i])
                            &&  (TimeValueAtExtremumPosition > TimeValueInSeconds)  )
                        {
                            (OP->MaxPosExtremaPositionVectorOnly->VecData)[i]   =   AnalizedPosition;
                            (OP->MaxExtremaTimesVector->VecData)[i] =   TimeValueAtExtremumPosition;
                        }

                        if (    (AnalizedPosition < (OP->MinPosExtremaPositionVectorOnly->VecData)[i])
                            &&  (TimeValueAtExtremumPosition > TimeValueInSeconds)  )
                        {
                            (OP->MinPosExtremaPositionVectorOnly->VecData)[i]   =   AnalizedPosition;
                            (OP->MinExtremaTimesVector->VecData)[i] =   TimeValueAtExtremumPosition;
                        }
                    }
                }
            }

            // Check the target state of motion.
            if ((this->Polynomials)[i].PolynomialTimes[(this->Polynomials)[i].ValidPolynomials - 2]
                 > TimeValueInSeconds)
            {
                if (    ((this->CurrentInputParameters->TargetPositionVector->VecData)[i]
                        >
                        (OP->MaxPosExtremaPositionVectorOnly->VecData)[i])
                        &&
                        (TimeValueInSeconds
                        <=
                        (this->Polynomials)[i].PolynomialTimes[(this->Polynomials)[i].ValidPolynomials - 2]))
                {
                    (OP->MaxPosExtremaPositionVectorOnly->VecData)[i]   =   (this->CurrentInputParameters->TargetPositionVector->VecData)[i];

                    if (this->CurrentTrajectoryIsNotSynchronized)
                    {
                        (OP->MaxExtremaTimesVector->VecData)[i] =   this->MinimumExecutionTimes->VecData[i];
                    }
                    else
                    {
                        (OP->MaxExtremaTimesVector->VecData)[i] =   this->SynchronizationTime + this->InternalClockInSeconds - this->CycleTime;
                    }
                }

                if (    ((this->CurrentInputParameters->TargetPositionVector->VecData)[i]
                        <
                        (OP->MinPosExtremaPositionVectorOnly->VecData)[i])
                        &&
                        (TimeValueInSeconds
                        <=
                        (this->Polynomials)[i].PolynomialTimes[(this->Polynomials)[i].ValidPolynomials - 2]))
                {
                    (OP->MinPosExtremaPositionVectorOnly->VecData)[i]   =   (this->CurrentInputParameters->TargetPositionVector->VecData)[i];

                    if (this->CurrentTrajectoryIsNotSynchronized)
                    {
                        (OP->MinExtremaTimesVector->VecData)[i] =   this->MinimumExecutionTimes->VecData[i];
                    }
                    else
                    {
                        (OP->MinExtremaTimesVector->VecData)[i] =   this->SynchronizationTime + this->InternalClockInSeconds - this->CycleTime;
                    }
                }
            }


            for (k = 0; k < this->NumberOfDOFs; k++)
            {
                if ((this->ModifiedSelectionVector->VecData)[k])
                {
                    for (l = 0; l < MAXIMAL_NO_OF_POLYNOMIALS; l++)
                    {
                        if (    (this->Polynomials)[k].PolynomialTimes[l]
                                >=
                                (OP->MinExtremaTimesVector->VecData)[i])
                        {
                            break;
                        }
                    }

                    (((OP->MinPosExtremaPositionVectorArray)[i])->VecData)[k]
                        =   (this->Polynomials)[k].PositionPolynomial[l].CalculateValue(
                                (OP->MinExtremaTimesVector->VecData)[i]);
                    (((OP->MinPosExtremaVelocityVectorArray)[i])->VecData)[k]
                        =   (this->Polynomials)[k].VelocityPolynomial[l].CalculateValue(
                                (OP->MinExtremaTimesVector->VecData)[i]);
                    (((OP->MinPosExtremaAccelerationVectorArray)[i])->VecData)[k]
                        =   (this->Polynomials)[k].AccelerationPolynomial[l].CalculateValue(
                                (OP->MinExtremaTimesVector->VecData)[i]);

                    (((OP->MinPosExtremaPositionVectorArray)[i])->VecData)[k]
                        =   (this->CurrentInputParameters->TargetPositionVector->VecData)[k]
                                -   ((this->StoredTargetPosition->VecData)[k]
                                -   (((OP->MinPosExtremaPositionVectorArray)[i])->VecData)[k] );

                    for (l = 0; l < MAXIMAL_NO_OF_POLYNOMIALS; l++)
                    {
                        if (    (this->Polynomials)[k].PolynomialTimes[l]
                                >=
                                (OP->MaxExtremaTimesVector->VecData)[i])
                        {
                            break;
                        }
                    }

                    (((OP->MaxPosExtremaPositionVectorArray)[i])->VecData)[k]
                        =   (this->Polynomials)[k].PositionPolynomial[l].CalculateValue(
                                (OP->MaxExtremaTimesVector->VecData)[i]);
                    (((OP->MaxPosExtremaVelocityVectorArray)[i])->VecData)[k]
                        =   (this->Polynomials)[k].VelocityPolynomial[l].CalculateValue(
                                (OP->MaxExtremaTimesVector->VecData)[i]);
                    (((OP->MaxPosExtremaAccelerationVectorArray)[i])->VecData)[k]
                        =   (this->Polynomials)[k].AccelerationPolynomial[l].CalculateValue(
                                (OP->MaxExtremaTimesVector->VecData)[i]);

                    // Correct the position values (in order to cope with varying input values for the current
                    // position and the target position while the difference between them remains constant)
                    (((OP->MaxPosExtremaPositionVectorArray)[i])->VecData)[k]
                        =   (this->CurrentInputParameters->TargetPositionVector->VecData)[k]
                                -   ((this->StoredTargetPosition->VecData)[k]
                                -   (((OP->MaxPosExtremaPositionVectorArray)[i])->VecData)[k] );
                }
                else
                {
                    (((OP->MinPosExtremaPositionVectorArray)[i])->VecData)[k]
                        =   (this->CurrentInputParameters->CurrentPositionVector->VecData)[k];
                    (((OP->MinPosExtremaVelocityVectorArray)[i])->VecData)[k]
                        =   (this->CurrentInputParameters->CurrentVelocityVector->VecData)[k];
                    (((OP->MinPosExtremaAccelerationVectorArray)[i])->VecData)[k]
                        =   (this->CurrentInputParameters->CurrentAccelerationVector->VecData)[k];

                    (((OP->MaxPosExtremaPositionVectorArray)[i])->VecData)[k]
                        =   (this->CurrentInputParameters->CurrentPositionVector->VecData)[k];
                    (((OP->MaxPosExtremaVelocityVectorArray)[i])->VecData)[k]
                        =   (this->CurrentInputParameters->CurrentVelocityVector->VecData)[k];
                    (((OP->MaxPosExtremaAccelerationVectorArray)[i])->VecData)[k]
                        =   (this->CurrentInputParameters->CurrentAccelerationVector->VecData)[k];
                }
            }

            OP->MaxExtremaTimesVector->VecData[i] -= TimeValueInSeconds;
            if (OP->MaxExtremaTimesVector->VecData[i] < 0.0)
            {
                OP->MaxExtremaTimesVector->VecData[i] = 0.0;
            }

            OP->MinExtremaTimesVector->VecData[i] -= TimeValueInSeconds;
            if (OP->MinExtremaTimesVector->VecData[i] < 0.0)
            {
                OP->MinExtremaTimesVector->VecData[i] = 0.0;
            }
        }
        else
        {
            // Set all non-selected DOFs to its current values.

            (OP->MinPosExtremaPositionVectorOnly->VecData)[i]   =   (this->CurrentInputParameters->CurrentPositionVector->VecData)[i];
            (OP->MaxPosExtremaPositionVectorOnly->VecData)[i]   =   (this->CurrentInputParameters->CurrentPositionVector->VecData)[i];
            (OP->MinExtremaTimesVector->VecData)[i]             =   0.0;
            (OP->MaxExtremaTimesVector->VecData)[i]             =   0.0;

            for (k = 0; k < this->NumberOfDOFs; k++)
            {
                (((OP->MinPosExtremaPositionVectorArray)[i])->VecData)[k]
                    =   (this->CurrentInputParameters->CurrentPositionVector->VecData)[k];
                (((OP->MinPosExtremaVelocityVectorArray)[i])->VecData)[k]
                    =   (this->CurrentInputParameters->CurrentVelocityVector->VecData)[k];
                (((OP->MinPosExtremaAccelerationVectorArray)[i])->VecData)[k]
                    =   (this->CurrentInputParameters->CurrentAccelerationVector->VecData)[k];

                (((OP->MaxPosExtremaPositionVectorArray)[i])->VecData)[k]
                    =   (this->CurrentInputParameters->CurrentPositionVector->VecData)[k];
                (((OP->MaxPosExtremaVelocityVectorArray)[i])->VecData)[k]
                    =   (this->CurrentInputParameters->CurrentVelocityVector->VecData)[k];
                (((OP->MaxPosExtremaAccelerationVectorArray)[i])->VecData)[k]
                    =   (this->CurrentInputParameters->CurrentAccelerationVector->VecData)[k];
            }
        }
    }

    return;
}


//*******************************************************************************************
// SetPositionalExtremsToZero()

void TypeIIRMLPosition::SetPositionalExtremsToZero(RMLPositionOutputParameters *OP) const
{
    unsigned int            i                           =   0
                        ,   k                           =   0;

    for (i = 0; i < this->NumberOfDOFs; i++)
    {
        for (k = 0; k < this->NumberOfDOFs; k++)
        {
            (((OP->MinPosExtremaPositionVectorArray)[i])->VecData)[k]
            =   0.0;
            (((OP->MinPosExtremaVelocityVectorArray)[i])->VecData)[k]
            =   0.0;
            (((OP->MinPosExtremaAccelerationVectorArray)[i])->VecData)[k]
            =   0.0;

            (((OP->MaxPosExtremaPositionVectorArray)[i])->VecData)[k]
            =   0.0;
            (((OP->MaxPosExtremaVelocityVectorArray)[i])->VecData)[k]
            =   0.0;
            (((OP->MaxPosExtremaAccelerationVectorArray)[i])->VecData)[k]
            =   0.0;
        }

        (OP->MinPosExtremaPositionVectorOnly->VecData)[i]
        =   0.0;
        (OP->MaxPosExtremaPositionVectorOnly->VecData)[i]
        =   0.0;
        (OP->MinExtremaTimesVector->VecData)[i]
        =   0.0;
        (OP->MaxExtremaTimesVector->VecData)[i]
        =   0.0;
    }
}
