//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLStep1IntermediateProfiles.h
//!
//! \brief
//! Header file for intermediate profile segments of Step 1
//!
//! \details
//! Header file for trajectory execution time calculations
//! of intermediate velocity profiles in the first step
//! of the Type II On-Line Trajectory Generation algorithm.
//! All profile functions are part of
//! the namespace TypeIIRMLMath.
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



#ifndef __TypeIIRMLStep1IntermediateProfiles__
#define __TypeIIRMLStep1IntermediateProfiles__

#include <TypeIIRMLMath.h>


namespace TypeIIRMLMath
{


//  ---------------------- Doxygen info ----------------------
//! \fn void NegateStep1(double *ThisCurrentPosition, double *ThisCurrentVelocity, double *ThisTargetPosition, double *ThisTargetVelocity)
//!
//! \brief
//! Negate input values during Step 1
//!
//! \param ThisCurrentPosition
//! Pointer to a \c double value: Current position value for DOF \f$ k \f$
//! at instant \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$.
//! The sign of this value will be flipped.
//!
//! \param ThisCurrentVelocity
//! Pointer to a \c double value: Current velocity value for DOF \f$ k \f$
//! at instant \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//! The sign of this value will be flipped.
//!
//! \param ThisTargetPosition
//! Pointer to a \c double value: Target position value for DOF \f$ k \f$
//! at instant \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//! The sign of this value will be flipped.
//!
//! \param ThisTargetVelocity
//! Pointer to a \c double value: Target velocity value for DOF \f$ k \f$
//! at instant \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,trgt} \f$
//! The sign of this value will be flipped.
//!
//! \note
//! This function is used in the Step 1 decision trees 1A, 1B, and 1C.
//!
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1A()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1B()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1C()
//! \sa TypeIIRMLMath::NegateStep2()
//  ----------------------------------------------------------
void NegateStep1(       double          *ThisCurrentPosition
                    ,   double          *ThisCurrentVelocity
                    ,   double          *ThisTargetPosition
                    ,   double          *ThisTargetVelocity         );


//  ---------------------- Doxygen info ----------------------
//! \fn void VToVMaxStep1(double *TotalTime, double *ThisCurrentPosition, double *ThisCurrentVelocity, const double &MaxVelocity, const double &MaxAcceleration)
//!
//! \brief
//! One intermediate Step 1 trajectory segment: v -> +vmax (NegLin)
//!
//! \param TotalTime
//! Pointer to a \c double value: Time in seconds required for preceding
//! trajectory segments. This value
//! will be increased by the amount of time required time for this
//! trajectory segment.
//!
//! \param ThisCurrentPosition
//! Pointer to a \c double value: Current position value for DOF \f$ k \f$
//! at instant \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$.
//! This value will be changed by the function in order to intermediately
//! reach an acceleration value of zero.
//!
//! \param ThisCurrentVelocity
//! Pointer to a \c double value: Current velocity value for DOF \f$ k \f$
//! at instant \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//! This value will be increased in order to intermediately reach an
//! acceleration value of \f$\ _{k}A_{i}^{\,max} \f$.
//!
//! \param MaxVelocity
//! Maximum allowed velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,max} \f$.
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$.
//!
//!
//! \note
//! This function is used in the Step 1 decision trees 1A, 1B, and 1C.
//!
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1A()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1B()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1C()
//! \sa TypeIIRMLMath::VToVMaxStep2()
//  ----------------------------------------------------------
void VToVMaxStep1(      double          *TotalTime
                    ,   double          *ThisCurrentPosition
                    ,   double          *ThisCurrentVelocity
                    ,   const double    &MaxVelocity
                    ,   const double    &MaxAcceleration        );


//  ---------------------- Doxygen info ----------------------
//! \fn void VToZeroStep1(double *TotalTime, double *ThisCurrentPosition, double *ThisCurrentVelocity, const double &MaxAcceleration)
//!
//! \brief
//! One intermediate Step 1 trajectory segment: v -> 0 (NegLin)
//!
//! \param TotalTime
//! Pointer to a \c double value: Time in seconds required for preceding
//! trajectory segments. This value
//! will be increased by the amount of time required time for this
//! trajectory segment.
//!
//! \param ThisCurrentPosition
//! Pointer to a \c double value: Current position value for DOF \f$ k \f$
//! at instant \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$.
//! This value will be changed by the function in order to intermediately
//! reach an acceleration value of zero.
//!
//! \param ThisCurrentVelocity
//! Pointer to a \c double value: Current velocity value for DOF \f$ k \f$
//! at instant \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//! This value will be increased in order to intermediately reach an
//! acceleration value of \f$\ _{k}A_{i}^{\,max} \f$.
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$.
//!
//! \note
//! This function is used in the Step 1 decision trees 1A, 1B, and 1C.
//!
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1A()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1B()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1C()
//! \sa TypeIIRMLMath::VToZeroStep2()
//  ----------------------------------------------------------
void VToZeroStep1(      double          *TotalTime
                    ,   double          *ThisCurrentPosition
                    ,   double          *ThisCurrentVelocity
                    ,   const double    &MaxAcceleration        );


}   // namespace TypeIIRMLMath

#endif
