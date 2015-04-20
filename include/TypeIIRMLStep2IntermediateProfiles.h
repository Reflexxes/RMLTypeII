//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLStep2IntermediateProfiles.h
//!
//! \brief
//! Header file for intermediate profile segments of Step 2
//!
//! \details
//! Header file for trajectory execution time calculations
//! of intermediate velocity profiles in the second step
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



#ifndef __TypeIIRMLStep2IntermediateTimeProfiles__
#define __TypeIIRMLStep2IntermediateTimeProfiles__


#include <TypeIIRMLPolynomial.h>
#include <TypeIIRMLMath.h>


namespace TypeIIRMLMath
{


//  ---------------------- Doxygen info ----------------------
//! \fn void NegateStep2(double *ThisCurrentPosition, double *ThisCurrentVelocity, double *ThisTargetPosition, double *ThisTargetVelocity, bool *Inverted)
//!
//! \brief
//! Negate input values
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
//! \param Inverted
//! Pointer to \c bool value: This bit will be flipped; it indicates,
//! whether the input values have been flipped even or odd times, such
//! that succeeding functions can set-up the trajectory parameters
//! correspondingly.
//!
//! \note
//! This function is used in the Step 2 decision tree.
//!
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree2()
//! \sa TypeIIRMLMath::NegateStep1()
//  ----------------------------------------------------------
void NegateStep2(       double              *ThisCurrentPosition
                    ,   double              *ThisCurrentVelocity
                    ,   double              *ThisTargetPosition
                    ,   double              *ThisTargetVelocity
                    ,   bool                *Inverted               );



//  ---------------------- Doxygen info ----------------------
//! \fn void VToVMaxStep2(double *ThisCurrentTime, double *ThisCurrentPosition, double *ThisCurrentVelocity, const double &MaxVelocity, const double &MaxAcceleration, MotionPolynomials *PolynomialsLocal, const bool &Inverted)
//!
//! \brief
//! One intermediate Step 2 trajectory segment: v -> vmax (NegLin)
//!
//! \param ThisCurrentTime
//! Pointer to a \c double value: Time in seconds required for preceding
//! trajectory segments. This value
//! will be increased by the amount of time required time for this
//! trajectory segment.
//!
//! \param ThisCurrentPosition
//! Pointer to a \c double value: Current position value for DOF \f$ k \f$
//! at instant \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$.
//! This value will be changed by the function in order to intermediately
//! reach the maximum velocity value, \f$\ _{k}V_{i}^{\,max} \f$.
//!
//! \param ThisCurrentVelocity
//! Pointer to a \c double value: Current velocity value for DOF \f$ k \f$
//! at instant \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//! This value will be increased in order to intermediately reach the
//! maximum velocity value, \f$\ _{k}V_{i}^{\,max} \f$.
//!
//! \param MaxVelocity
//! Maximum allowed velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,max} \f$.
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$.
//!
//! \param PolynomialsLocal
//! Pointer to a \c MotionPolynomials object, which contains the
//! complete trajectory for one single DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$ _k{\cal M}_{i}(t) \f$. This function will
//! add one further trajectory segment to this object.
//!
//! \param Inverted
//! A \c bool value that indicates whether the input values have been
//! flipped even or odd times prior to the execution of this function
//! (cf. TypeIIRMLMath::NegatePink()). The trajectory parameters in
//! \c PolynomialsLocal will be set-up correspondingly.
//!
//! \note
//! This function is used in the Step 2 decision tree.
//!
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree2()
//! \sa TypeIIRMLMath::VToVMaxStep1()
//  ----------------------------------------------------------
void VToVMaxStep2(      double              *ThisCurrentTime
                    ,   double              *ThisCurrentPosition
                    ,   double              *ThisCurrentVelocity
                    ,   const double        &MaxVelocity
                    ,   const double        &MaxAcceleration
                    ,   MotionPolynomials   *PolynomialsLocal
                    ,   const bool          &Inverted               );


//  ---------------------- Doxygen info ----------------------
//! \fn void VToZeroStep2(double *ThisCurrentTime, double *ThisCurrentPosition, double *ThisCurrentVelocity, const double &MaxAcceleration, MotionPolynomials *PolynomialsLocal, const bool &Inverted)
//!
//! \brief
//! One intermediate Step 2 trajectory segment: v -> 0 (NegLin)
//!
//! \param ThisCurrentTime
//! Pointer to a \c double value: Time in seconds required for preceding
//! trajectory segments. This value
//! will be increased by the amount of time required time for this
//! trajectory segment.
//!
//! \param ThisCurrentPosition
//! Pointer to a \c double value: Current position value for DOF \f$ k \f$
//! at instant \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$.
//! This value will be changed by the function in order to intermediately
//! reach a velocity value of zero.
//!
//! \param ThisCurrentVelocity
//! Pointer to a \c double value: Current velocity value for DOF \f$ k \f$
//! at instant \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//! This value will be increased in order to intermediately reach a
//! velocity value of zero.
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$.
//!
//! \param PolynomialsLocal
//! Pointer to a \c MotionPolynomials object, which contains the
//! complete trajectory for one single DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$ _k{\cal M}_{i}(t) \f$. This function will
//! add one further trajectory segment to this object.
//!
//! \param Inverted
//! A \c bool value that indicates whether the input values have been
//! flipped even or odd times prior to the execution of this function
//! (cf. TypeIIRMLMath::NegatePink()). The trajectory parameters in
//! \c PolynomialsLocal will be set-up correspondingly.
//!
//! \note
//! This function is used in the Step 2 decision tree.
//!
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree2()
//! \sa TypeIIRMLMath::VToZeroStep1()
//  ----------------------------------------------------------
void VToZeroStep2(      double              *ThisCurrentTime
                    ,   double              *ThisCurrentPosition
                    ,   double              *ThisCurrentVelocity
                    ,   const double        &MaxAcceleration
                    ,   MotionPolynomials   *PolynomialsLocal
                    ,   const bool          &Inverted               );

}   // namespace TypeIIRMLMath

#endif
