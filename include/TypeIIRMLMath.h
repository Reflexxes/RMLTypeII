//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLMath.h
//!
//! \brief
//! Header file for functions and definitions of constant values and macros
//!
//! \details
//! Header file for definitions of constant values and macros to be used
//! for within in the library of the Type II On-Line Trajectory Algorithm.
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


#ifndef __TypeIIRMLMath__
#define __TypeIIRMLMath__


//*******************************************************************************************
// Include files

#include <math.h>


namespace TypeIIRMLMath
{

//*******************************************************************************************
// Definitions and macros




//  ---------------------- Doxygen info ----------------------
//! \def OTG_INFINITY
//!
//! \brief
//! A value for infinity \f$ \infty = 10^{100} \f$
//  ----------------------------------------------------------
#define OTG_INFINITY ((double)1.0e100)


//  ---------------------- Doxygen info ----------------------
//! \def RML_INPUT_VALUE_EPSILON
//!
//! \brief
//! Positive threshold value to compare current and former input values
//!
//! \details
//! Positive threshold value to determine, whether the input values of the
//! OTG algorithm remained constant. This value is used in the macro
//! IsInputEpsilonEqual().
//!
//! \sa IsInputEpsilonEqual(A,B)
//  ----------------------------------------------------------
#define RML_INPUT_VALUE_EPSILON ((double)1.0e-10)


//  ---------------------- Doxygen info ----------------------
//! \def RML_VALID_SOLUTION_EPSILON
//!
//! \brief
//! Positive threshold value used during the check, whether a valid
//! solution for a given profile is possible
//!
//! \sa TypeIIRMLMath::IsSolutionForProfile_PosTriNegLin_Possible()
//  ----------------------------------------------------------
#define RML_VALID_SOLUTION_EPSILON ((double)1.0e-10)


//  ---------------------- Doxygen info ----------------------
//! \def MAXIMAL_NO_OF_POLYNOMIALS
//!
//! \brief
//! The maximum number of polynomials
//!
//! \details
//! The maximum number of polynomials to be used for the Type II
//! On-Line Trajectory Generation algorithm. This number is set up
//! w.r.t. the 6 trajectory segments of the Step 2 Decision Tree plus
//! one further segment for the time after the desired state of motion
//! has been reached.
//!
//! \sa TypeIIRMLPolynomial.h
//  ----------------------------------------------------------
#define MAXIMAL_NO_OF_POLYNOMIALS  7


//  ---------------------- Doxygen info ----------------------
//! \def RML_MAX_EXECUTION_TIME
//!
//! \brief
//! Maximum value for the for the minimum trajectory execution time
//! \f$t_i^{\,min}\f$
//!
//! \details
//! This value is required to ensure numerical stability.
//!
//! \sa ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG
//  ----------------------------------------------------------
#define RML_MAX_EXECUTION_TIME ((double)1e10)


//  ---------------------- Doxygen info ----------------------
//! \def RML_ADDITIONAL_RELATIVE_POSITION_ERROR_IN_CASE_OF_EQUALITY
//!
//! \brief
//! If the initial state of motion exactly equals the target state of motion,
//! a negligible position error is created
//!
//! \sa TypeIIRMLPosition::CompareAndEventuallyAdaptInitialAndTargetStateofMotion()
//  ----------------------------------------------------------
#define RML_ADDITIONAL_RELATIVE_POSITION_ERROR_IN_CASE_OF_EQUALITY ((double)1e-7)


//  ---------------------- Doxygen info ----------------------
//! \def RML_ADDITIONAL_ABSOLUTE_POSITION_ERROR_IN_CASE_OF_EQUALITY
//!
//! \brief
//! If the initial state of motion exactly equals the target state of motion,
//! a negligible position error is created
//!
//! \sa TypeIIRMLPosition::CompareAndEventuallyAdaptInitialAndTargetStateofMotion()
//  ----------------------------------------------------------
#define RML_ADDITIONAL_ABSOLUTE_POSITION_ERROR_IN_CASE_OF_EQUALITY ((double)1e-7)


//  ---------------------- Doxygen info ----------------------
//! \def POSITIVE_ZERO
//!
//! \brief
//! To prevent from numerical errors, a value for a "positive"
//! value of zero is required for deterministic behavior
//  ----------------------------------------------------------
#define POSITIVE_ZERO ((double)1.0e-50)


//  ---------------------- Doxygen info ----------------------
//! \def ABSOLUTE_PHASE_SYNC_EPSILON
//!
//! \brief
//! Absolute epsilon to check whether all required vectors
//! are collinear
//!
//! \sa TypeIIRMLPosition::IsPhaseSynchronizationPossible()
//! \sa TypeIIRMLPosition::Step1()
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa TypeIIRMLVelocity::IsPhaseSynchronizationPossible()
//  ----------------------------------------------------------
#define ABSOLUTE_PHASE_SYNC_EPSILON ((double)1.0e-6)


//  ---------------------- Doxygen info ----------------------
//! \def RELATIVE_PHASE_SYNC_EPSILON
//!
//! \brief
//! Relative epsilon to check whether all required vectors
//! are collinear
//!
//! \sa TypeIIRMLPosition::Step1()
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//  ----------------------------------------------------------
#define RELATIVE_PHASE_SYNC_EPSILON ((double)1.0e-3)


//  ---------------------- Doxygen info ----------------------
//! \def RML_INFINITY
//!
//! \brief
//! A value for infinity \f$ \infty = 10^{100} \f$
//  ----------------------------------------------------------
#define RML_INFINITY ((double)1.0e100)


//  ---------------------- Doxygen info ----------------------
//! \def RML_POSITION_EXTREMS_TIME_EPSILON
//!
//! \brief
//! Time value in seconds to increase the time intervals, in which a
//! trajectory segment is valid in order to robustly calculate
//! positional extremes
//!
//!
//! \sa TypeIIRMLPosition::CalculatePositionalExtrems()
//! \sa TypeIIRMLVelocity::CalculatePositionalExtrems()
//  ----------------------------------------------------------
#define RML_POSITION_EXTREMS_TIME_EPSILON ((double)1.0e-4)


//  ---------------------- Doxygen info ----------------------
//! \def RML_DENOMINATOR_EPSILON
//!
//! \brief
//! Epsilon value to prevent from division by zero
//  ----------------------------------------------------------
#define RML_DENOMINATOR_EPSILON ((double)1.0e-6)


//  ---------------------- Doxygen info ----------------------
//! \def PHASE_SYNC_COLLINEARITY_REL_EPSILON
//!
//! \brief
//! Relative value to check for collinearity during the check for phase
//! synchronization
//!
//! \sa TypeIIRMLPosition::IsPhaseSynchronizationPossible()
//  ----------------------------------------------------------
#define PHASE_SYNC_COLLINEARITY_REL_EPSILON ((double)1.0e-2)


//  ---------------------- Doxygen info ----------------------
//! \def Sign(A)
//!
//! \brief
//! Sign macro (integer)
//!
//! \param A
//! Value, whose sign is returned.
//!
//! \return
//! Integer value: -1 or 1
//  ----------------------------------------------------------
#define Sign(A) ( ((double)(A) < 0.0)?(-1):(1) )


//  ---------------------- Doxygen info ----------------------
//! \def FSign(A)
//!
//! \brief
//! Sign macro (floating point)
//!
//! \param A
//! Value, whose sign is returned.
//!
//! \return
//! Floating point value: -1.0 or 1.0
//  ----------------------------------------------------------
#define FSign(A) ( ((double)(A) < 0.0)?(-1.0):(1.0) )


//  ---------------------- Doxygen info ----------------------
//! \def pow2(A)
//!
//! \brief
//! A to the power of 2
//!
//! \param A
//! Basis
//!
//! \return
//! Result value
//  ----------------------------------------------------------
#define pow2(A)                         ((A)*(A))


//  ---------------------- Doxygen info ----------------------
//! \fn inline double RMLSqrt(const double &Value)
//!
//! \brief
//! Calculates the real square root of a given value. If the value is
//! negative a value of almost zero will be returned.
//!
//! \param Value
//! Square root radicand
//!
//! \return
//! Square root value (real).
//!
//! \sa POSITIVE_ZERO
//  ----------------------------------------------------------
inline double RMLSqrt(const double &Value)
{
    return( ( Value <= 0.0 ) ? ( POSITIVE_ZERO ) : ( sqrt( Value ) ) );
}


//  ---------------------- Doxygen info ----------------------
//! \def IsInputEpsilonEqual(A,B)
//!
//! \brief
//! A macro that checks, whether the difference between the values
//! 'A' and 'B' is less than RML_INPUT_VALUE_EPSILON.
//  ----------------------------------------------------------
#define IsInputEpsilonEqual(A,B) ((bool)((fabs((double)A - (double)B) <= RML_INPUT_VALUE_EPSILON)?(true):(false)))


//  ---------------------- Doxygen info ----------------------
//! \fn inline bool IsEpsilonEquality(const double &Value1, const double &Value2, const double &Epsilon)
//!
//! \brief
//! Checks epsilon equality for two values
//!
//! \details
//! Returns \f$ \left(\left|Value1\,-\,Value2\right|\ <=\ Epsilon\right)\f$,
//! that if epsilon quality holds, \c true will be returned, otherwise
//! \c false.
//!
//! \param Value1
//! First value to be compared
//!
//! \param Value2
//! Second value to be compared
//!
//! \param Epsilon
//! Value for Epsilon
//!
//! \return
//! \c true if both values are epsilon equal, otherwise \c false
//  ----------------------------------------------------------
inline bool IsEpsilonEquality(      const double &Value1
                                ,   const double &Value2
                                ,   const double &Epsilon)
{
    return(fabs(Value1 - Value2) <= Epsilon);
}


}   // namespace TypeIIRMLMath

#endif
