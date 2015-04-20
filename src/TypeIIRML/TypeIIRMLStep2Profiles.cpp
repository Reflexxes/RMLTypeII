//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLStep2Profiles.cpp
//!
//! \brief
//! Implementation file for the calculation of all Step 2 motion
//! profiles for the Type II On-Line Trajectory Generation algorithm
//!
//! \details
//! For further information, please refer to the file
//! TypeIIRMLStep2Profiles.h
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


#include <TypeIIRMLStep2Profiles.h>
#include <TypeIIRMLPolynomial.h>
#include <TypeIIRMLMath.h>
#include <math.h>


//************************************************************************************
// ProfileStep2PosLinHldNegLin()

void TypeIIRMLMath::ProfileStep2PosLinHldNegLin(    const double        &CurrentTime
                                                ,   const double        &SynchronizationTime
                                                ,   const double        &CurrentPosition
                                                ,   const double        &CurrentVelocity
                                                ,   const double        &TargetPosition
                                                ,   const double        &TargetVelocity
                                                ,   const double        &MaxAcceleration
                                                ,   MotionPolynomials   *PolynomialsLocal
                                                ,   const bool          &Inverted               )
{
    double      HoldVelocity            =   0.0
            ,   TimeForCurrentStep      =   0.0
            ,   ThisCurrentPosition     =   CurrentPosition
            ,   ThisCurrentVelocity     =   CurrentVelocity
            ,   TimeDifference          =   SynchronizationTime - CurrentTime
            ,   ThisCurrentTime         =   CurrentTime;

    HoldVelocity    =   0.5 * (TimeDifference * MaxAcceleration
                        +   ThisCurrentVelocity + TargetVelocity
                        -   RMLSqrt(pow2(MaxAcceleration)
                        *   pow2(TimeDifference) - pow2(ThisCurrentVelocity
                        -   TargetVelocity) + 2.0 * MaxAcceleration * (2.0
                        *   (ThisCurrentPosition - TargetPosition)
                        +   TimeDifference * (ThisCurrentVelocity
                        +   TargetVelocity)))                               );

    if (HoldVelocity < ThisCurrentVelocity)
    {
        HoldVelocity    =   ThisCurrentVelocity;
    }

    // ********************************************************************
    // Compute polynomial parameters for the time interval t01 (PosLin)

    TimeForCurrentStep  =   (HoldVelocity - ThisCurrentVelocity) / MaxAcceleration;

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * (-MaxAcceleration)), (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-MaxAcceleration) , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-MaxAcceleration), ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * MaxAcceleration), ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, MaxAcceleration , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , MaxAcceleration, ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  0.5 * (ThisCurrentVelocity + HoldVelocity)
                            *   TimeForCurrentStep;
    ThisCurrentVelocity =   HoldVelocity;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time interval t12 (Hld)

    if (HoldVelocity > TargetVelocity)
    {
        TimeForCurrentStep  =   SynchronizationTime - ThisCurrentTime
                                -   (HoldVelocity - TargetVelocity)
                                /   MaxAcceleration;
    }
    else
    {
        TimeForCurrentStep  =   SynchronizationTime - ThisCurrentTime;
    }

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  ThisCurrentVelocity * TimeForCurrentStep;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time interval t23 (NegLin)

    if (ThisCurrentVelocity > TargetVelocity)
    {
        TimeForCurrentStep  =   (ThisCurrentVelocity - TargetVelocity)
                                /   MaxAcceleration;
    }
    else
    {
        TimeForCurrentStep  =   0.0;
    }

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * MaxAcceleration), (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, MaxAcceleration , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , MaxAcceleration, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * (-MaxAcceleration)), ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-MaxAcceleration) , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-MaxAcceleration), ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  0.5 * (TargetVelocity + ThisCurrentVelocity)
                            *   TimeForCurrentStep;
    ThisCurrentVelocity -=  TimeForCurrentStep * MaxAcceleration;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time, after ptrgt and vtrgt
    // have been reached.

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + RML_INFINITY;
    PolynomialsLocal->ValidPolynomials++;

    return;
}


//************************************************************************************
// ProfileStep2PosLinHldPosLin()

void TypeIIRMLMath::ProfileStep2PosLinHldPosLin(    const double        &CurrentTime
                                                ,   const double        &SynchronizationTime
                                                ,   const double        &CurrentPosition
                                                ,   const double        &CurrentVelocity
                                                ,   const double        &TargetPosition
                                                ,   const double        &TargetVelocity
                                                ,   const double        &MaxAcceleration
                                                ,   MotionPolynomials   *PolynomialsLocal
                                                ,   const bool          &Inverted               )
{
    double      HoldVelocity            =   0.0
            ,   TimeForCurrentStep      =   0.0
            ,   ThisCurrentPosition     =   CurrentPosition
            ,   ThisCurrentVelocity     =   CurrentVelocity
            ,   TimeDifference          =   SynchronizationTime - CurrentTime
            ,   ThisCurrentTime         =   CurrentTime
            ,   Denominator             =   MaxAcceleration * TimeDifference + CurrentVelocity
                                            -   TargetVelocity;

    if (fabs(Denominator) > RML_DENOMINATOR_EPSILON)
    {
        HoldVelocity    =   (0.5 * (2.0 * MaxAcceleration * (TargetPosition
                            -   CurrentPosition) + pow2(CurrentVelocity)
                            -   pow2(TargetVelocity))) / Denominator;
    }
    else
    {
        HoldVelocity    =    TargetVelocity;
    }

    if (HoldVelocity < ThisCurrentVelocity)
    {
        HoldVelocity    =   ThisCurrentVelocity;
    }

    // ********************************************************************
    // Compute polynomial parameters for the time interval t01 (PosLin)

    TimeForCurrentStep  =   (HoldVelocity - ThisCurrentVelocity) / MaxAcceleration;

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * (-MaxAcceleration)), (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-MaxAcceleration) , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-MaxAcceleration), ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * MaxAcceleration), ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, MaxAcceleration , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , MaxAcceleration, ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  0.5 * (ThisCurrentVelocity + HoldVelocity)
                            *   TimeForCurrentStep;
    ThisCurrentVelocity =   HoldVelocity;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time interval t12 (Hld)

    if (TargetVelocity > HoldVelocity)
    {
        TimeForCurrentStep  =   SynchronizationTime - ThisCurrentTime
                                -   (TargetVelocity - HoldVelocity)
                                /   MaxAcceleration;
    }
    else
    {
        TimeForCurrentStep  =   SynchronizationTime - ThisCurrentTime;
    }

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  ThisCurrentVelocity * TimeForCurrentStep;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time interval t23 (PosLin)

    if (TargetVelocity > ThisCurrentVelocity)
    {
        TimeForCurrentStep  =   (TargetVelocity - ThisCurrentVelocity)
                                /   MaxAcceleration;
    }
    else
    {
        TimeForCurrentStep  =   0.0;
    }

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * (-MaxAcceleration)), (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-MaxAcceleration) , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-MaxAcceleration), ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * MaxAcceleration), ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, MaxAcceleration , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , MaxAcceleration, ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  0.5 * (TargetVelocity + ThisCurrentVelocity)
                            *   TimeForCurrentStep;
    ThisCurrentVelocity +=  TimeForCurrentStep * MaxAcceleration;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time, after ptrgt and vtrgt
    // have been reached.

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + RML_INFINITY;
    PolynomialsLocal->ValidPolynomials++;

    return;
}


//************************************************************************************
// ProfileStep2NegLinHldPosLin()

void TypeIIRMLMath::ProfileStep2NegLinHldPosLin(    const double        &CurrentTime
                                                ,   const double        &SynchronizationTime
                                                ,   const double        &CurrentPosition
                                                ,   const double        &CurrentVelocity
                                                ,   const double        &TargetPosition
                                                ,   const double        &TargetVelocity
                                                ,   const double        &MaxAcceleration
                                                ,   MotionPolynomials   *PolynomialsLocal
                                                ,   const bool          &Inverted               )
{
    double      HoldVelocity            =   0.0
            ,   TimeForCurrentStep      =   0.0
            ,   ThisCurrentPosition     =   CurrentPosition
            ,   ThisCurrentVelocity     =   CurrentVelocity
            ,   TimeDifference          =   SynchronizationTime - CurrentTime
            ,   ThisCurrentTime         =   CurrentTime;

    HoldVelocity    =   0.5 * (CurrentVelocity + TargetVelocity
                        -   TimeDifference * MaxAcceleration
                        +   RMLSqrt(pow2(MaxAcceleration)
                        *   pow2(TimeDifference) - pow2(CurrentVelocity
                        -   TargetVelocity) - 2.0 * MaxAcceleration * (2.0
                        *   (CurrentPosition - TargetPosition)
                        +   TimeDifference * (CurrentVelocity
                        +   TargetVelocity))));

    if (HoldVelocity > ThisCurrentVelocity)
    {
        HoldVelocity    =   ThisCurrentVelocity;
    }

    // ********************************************************************
    // Compute polynomial parameters for the time interval t01 (NegLin)

    TimeForCurrentStep  =   (ThisCurrentVelocity - HoldVelocity) / MaxAcceleration;

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * MaxAcceleration), (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, MaxAcceleration , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , MaxAcceleration, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * (-MaxAcceleration)), ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-MaxAcceleration) , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-MaxAcceleration), ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  0.5 * (ThisCurrentVelocity + HoldVelocity)
                            *   TimeForCurrentStep;
    ThisCurrentVelocity =   HoldVelocity;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time interval t12 (Hld)

    if (TargetVelocity > HoldVelocity)
    {
        TimeForCurrentStep  =   SynchronizationTime - ThisCurrentTime
                                -   (TargetVelocity - HoldVelocity)
                                /   MaxAcceleration;
    }
    else
    {
        TimeForCurrentStep  =   SynchronizationTime - ThisCurrentTime;
    }

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  ThisCurrentVelocity * TimeForCurrentStep;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time interval t23 (PosLin)

    if (TargetVelocity > ThisCurrentVelocity)
    {
        TimeForCurrentStep  =   (TargetVelocity - ThisCurrentVelocity)
                                /   MaxAcceleration;
    }
    else
    {
        TimeForCurrentStep  =   0.0;
    }

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * (-MaxAcceleration)), (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-MaxAcceleration) , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-MaxAcceleration), ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * MaxAcceleration), ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, MaxAcceleration , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , MaxAcceleration, ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  0.5 * (TargetVelocity + ThisCurrentVelocity)
                            *   TimeForCurrentStep;
    ThisCurrentVelocity +=  TimeForCurrentStep * MaxAcceleration;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time, after ptrgt and vtrgt
    // have been reached.

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + RML_INFINITY;
    PolynomialsLocal->ValidPolynomials++;

    return;
}


//************************************************************************************
// ProfileStep2NegLinHldNegLin()

void TypeIIRMLMath::ProfileStep2NegLinHldNegLin(    const double        &CurrentTime
                                                ,   const double        &SynchronizationTime
                                                ,   const double        &CurrentPosition
                                                ,   const double        &CurrentVelocity
                                                ,   const double        &TargetPosition
                                                ,   const double        &TargetVelocity
                                                ,   const double        &MaxAcceleration
                                                ,   MotionPolynomials   *PolynomialsLocal
                                                ,   const bool          &Inverted               )
{
    double      HoldVelocity            =   0.0
            ,   TimeForCurrentStep      =   0.0
            ,   ThisCurrentPosition     =   CurrentPosition
            ,   ThisCurrentVelocity     =   CurrentVelocity
            ,   TimeDifference          =   SynchronizationTime - CurrentTime
            ,   ThisCurrentTime         =   CurrentTime
            ,    Denominator            =   MaxAcceleration * TimeDifference - CurrentVelocity
                                            +   TargetVelocity;

    if (fabs(Denominator) > RML_DENOMINATOR_EPSILON)
    {
        HoldVelocity    =   (0.5 * (2.0 * MaxAcceleration * (TargetPosition
                            -   CurrentPosition) - pow2(CurrentVelocity)
                            +   pow2(TargetVelocity))) / Denominator;
    }
    else
    {
        HoldVelocity    =   TargetVelocity;
    }

    if (HoldVelocity > ThisCurrentVelocity)
    {
        HoldVelocity    =   ThisCurrentVelocity;
    }

    // ********************************************************************
    // Compute polynomial parameters for the time interval t01 (NegLin)

    TimeForCurrentStep  =   (ThisCurrentVelocity - HoldVelocity) / MaxAcceleration;

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * MaxAcceleration), (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, MaxAcceleration , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , MaxAcceleration, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * (-MaxAcceleration)), ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-MaxAcceleration) , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-MaxAcceleration), ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  0.5 * (ThisCurrentVelocity + HoldVelocity)
                            *   TimeForCurrentStep;
    ThisCurrentVelocity =   HoldVelocity;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time interval t12 (Hld)

    if (HoldVelocity > TargetVelocity)
    {
        TimeForCurrentStep  =   SynchronizationTime - ThisCurrentTime
                                -   (HoldVelocity - TargetVelocity)
                                /   MaxAcceleration;
    }
    else
    {
        TimeForCurrentStep  =   SynchronizationTime - ThisCurrentTime;
    }

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  ThisCurrentVelocity * TimeForCurrentStep;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time interval t23 (NegLin)

    if (ThisCurrentVelocity > TargetVelocity)
    {
        TimeForCurrentStep  =   (ThisCurrentVelocity - TargetVelocity)
                                /   MaxAcceleration;
    }
    else
    {
        TimeForCurrentStep  =   0.0;
    }

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * MaxAcceleration), (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, MaxAcceleration , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , MaxAcceleration, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * (-MaxAcceleration)), ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-MaxAcceleration) , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-MaxAcceleration), ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  0.5 * (TargetVelocity + ThisCurrentVelocity)
                            *   TimeForCurrentStep;
    ThisCurrentVelocity -=  TimeForCurrentStep * MaxAcceleration;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time, after ptrgt and vtrgt
    // have been reached.

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + RML_INFINITY;
    PolynomialsLocal->ValidPolynomials++;

    return;
}


//************************************************************************************
// ProfileStep2PosTrapNegLin()

void TypeIIRMLMath::ProfileStep2PosTrapNegLin(      const double        &CurrentTime
                                                ,   const double        &SynchronizationTime
                                                ,   const double        &CurrentPosition
                                                ,   const double        &CurrentVelocity
                                                ,   const double        &TargetPosition
                                                ,   const double        &TargetVelocity
                                                ,   const double        &MaxAcceleration
                                                ,   MotionPolynomials   *PolynomialsLocal
                                                ,   const bool          &Inverted               )
{
    double      HoldVelocity            =   0.0
            ,   TimeForCurrentStep      =   0.0
            ,   ThisCurrentPosition     =   CurrentPosition
            ,   ThisCurrentVelocity     =   CurrentVelocity
            ,   TimeDifference          =   SynchronizationTime - CurrentTime
            ,   ThisCurrentTime         =   CurrentTime;

    HoldVelocity    =   0.5 * (MaxAcceleration * TimeDifference
                        +   CurrentVelocity + TargetVelocity
                        -   RMLSqrt(pow2(MaxAcceleration)
                        *   pow2(TimeDifference) - pow2(CurrentVelocity
                        -   TargetVelocity)+ 2.0 * MaxAcceleration * (2.0
                        *   (CurrentPosition - TargetPosition)
                        +   TimeDifference * (CurrentVelocity
                        +   TargetVelocity))));

    if (HoldVelocity < ThisCurrentVelocity)
    {
        HoldVelocity    =   ThisCurrentVelocity;
    }

    // ********************************************************************
    // Compute polynomial parameters for the time interval t01 (PosLin)

    TimeForCurrentStep  =   (HoldVelocity - ThisCurrentVelocity) / MaxAcceleration;

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * (-MaxAcceleration)), (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-MaxAcceleration) , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-MaxAcceleration), ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * MaxAcceleration), ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, MaxAcceleration , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , MaxAcceleration, ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  0.5 * (ThisCurrentVelocity + HoldVelocity)
                            *   TimeForCurrentStep;
    ThisCurrentVelocity =   HoldVelocity;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time interval t12 (Hld)

    TimeForCurrentStep  =   SynchronizationTime - ThisCurrentTime
                            -   (HoldVelocity - TargetVelocity)
                            /   MaxAcceleration;

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  ThisCurrentVelocity * TimeForCurrentStep;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time interval t23 (NegLin)

    TimeForCurrentStep  =   ThisCurrentVelocity / MaxAcceleration;

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * MaxAcceleration), (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, MaxAcceleration , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , MaxAcceleration, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * (-MaxAcceleration)), ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-MaxAcceleration) , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-MaxAcceleration), ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  0.5 * ThisCurrentVelocity * TimeForCurrentStep;
    ThisCurrentVelocity -=  TimeForCurrentStep * MaxAcceleration;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time interval t34 (NegLin)

    TimeForCurrentStep  =   -TargetVelocity / MaxAcceleration;

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * MaxAcceleration), (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, MaxAcceleration , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , MaxAcceleration, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * (-MaxAcceleration)), ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-MaxAcceleration) , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-MaxAcceleration), ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  0.5 * TargetVelocity * TimeForCurrentStep;
    ThisCurrentVelocity -=  TimeForCurrentStep * MaxAcceleration;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time, after ptrgt and vtrgt
    // have been reached.

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + RML_INFINITY;
    PolynomialsLocal->ValidPolynomials++;

    return;
}


//************************************************************************************
// ProfileStep2NegLinHldNegLinNegLin()

void TypeIIRMLMath::ProfileStep2NegLinHldNegLinNegLin(      const double        &CurrentTime
                                                        ,   const double        &SynchronizationTime
                                                        ,   const double        &CurrentPosition
                                                        ,   const double        &CurrentVelocity
                                                        ,   const double        &TargetPosition
                                                        ,   const double        &TargetVelocity
                                                        ,   const double        &MaxAcceleration
                                                        ,   MotionPolynomials   *PolynomialsLocal
                                                        ,   const bool          &Inverted               )
{
    double      HoldVelocity            =   0.0
            ,   TimeForCurrentStep      =   0.0
            ,   ThisCurrentPosition     =   CurrentPosition
            ,   ThisCurrentVelocity     =   CurrentVelocity
            ,   TimeDifference          =   SynchronizationTime - CurrentTime
            ,   ThisCurrentTime         =   CurrentTime
            ,    Denominator            =   MaxAcceleration * TimeDifference - CurrentVelocity
                                            +   TargetVelocity;

    if (fabs(Denominator) > RML_DENOMINATOR_EPSILON)
    {
        HoldVelocity    =   (0.5 * (2.0 * MaxAcceleration * (TargetPosition
                            -   CurrentPosition) - pow2(CurrentVelocity)
                            +   pow2(TargetVelocity))) / Denominator;
    }
    else
    {
        HoldVelocity    =   TargetVelocity;
    }

    if (HoldVelocity > ThisCurrentVelocity)
    {
        HoldVelocity    =   ThisCurrentVelocity;
    }

    if (HoldVelocity < 0.0)
    {
        HoldVelocity    =   0.0;
    }

    // ********************************************************************
    // Compute polynomial parameters for the time interval t01 (NegLin)

    TimeForCurrentStep  =   (ThisCurrentVelocity - HoldVelocity) / MaxAcceleration;

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * MaxAcceleration), (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, MaxAcceleration , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , MaxAcceleration, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * (-MaxAcceleration)), ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-MaxAcceleration) , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-MaxAcceleration), ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  0.5 * (ThisCurrentVelocity + HoldVelocity)
                            *   TimeForCurrentStep;
    ThisCurrentVelocity =   HoldVelocity;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time interval t12 (Hld)

    TimeForCurrentStep  =   SynchronizationTime - ThisCurrentTime
                            -   (HoldVelocity - TargetVelocity)
                            /   MaxAcceleration;

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  ThisCurrentVelocity * TimeForCurrentStep;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time interval t23 (NegLin)

    TimeForCurrentStep  =   ThisCurrentVelocity / MaxAcceleration;

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * MaxAcceleration), (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, MaxAcceleration , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , MaxAcceleration, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * (-MaxAcceleration)), ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-MaxAcceleration) , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-MaxAcceleration), ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  0.5 * ThisCurrentVelocity * TimeForCurrentStep;
    ThisCurrentVelocity -=  TimeForCurrentStep * MaxAcceleration;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time interval t34 (NegLin)

    TimeForCurrentStep  =   -TargetVelocity / MaxAcceleration;

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * MaxAcceleration), (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, MaxAcceleration , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , MaxAcceleration, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * (-MaxAcceleration)), ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-MaxAcceleration) , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-MaxAcceleration), ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++;

    ThisCurrentPosition +=  0.5 * TargetVelocity * TimeForCurrentStep;
    ThisCurrentVelocity -=  TimeForCurrentStep * MaxAcceleration;
    ThisCurrentTime     +=  TimeForCurrentStep;

    // ********************************************************************
    // Compute polynomial parameters for the time, after ptrgt and vtrgt
    // have been reached.

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-ThisCurrentVelocity), (-ThisCurrentPosition), ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-ThisCurrentVelocity), ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, ThisCurrentVelocity, ThisCurrentPosition, ThisCurrentTime);
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , ThisCurrentVelocity, ThisCurrentTime);
        PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , 0.0, ThisCurrentTime);
    }

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = ThisCurrentTime + RML_INFINITY;
    PolynomialsLocal->ValidPolynomials++;

    return;
}
