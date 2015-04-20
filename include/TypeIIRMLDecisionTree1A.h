//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLDecisionTree1A.h
//!
//! \brief
//! Header file for the Step 1 decision tree 1A of the Type II On-Line
//! Trajectory Generation algorithm
//!
//! \details
//! Header file for the function TypeIIRMLMath::TypeIIRMLDecisionTree1A(),
//! which contains the first decision tree (1A) of the Step 1 of the On-
//! Line Trajectory Generation algorithm. It calculates the minimum possible
//! trajectory execution time for one single degree of freedom. Details
//! on this methodology may be found in\n
//! \n
//! <b>T. Kroeger.</b>\n
//! <b>On-Line Trajectory Generation in Robotic Systems.</b>\n
//! <b>Springer Tracts in Advanced Robotics, Vol. 58, Springer, January 2010.</b>\n
//! <b><a href="http://www.springer.com/978-3-642-05174-6" target="_blank" title="You may order your personal copy at www.springer.com.">http://www.springer.com/978-3-642-05174-6</a></b>\n
//! \n
//! The function is part of the namespace TypeIIRMLMath.
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


#ifndef __TypeIIRMLDecisionTree1A__
#define __TypeIIRMLDecisionTree1A__

#include <TypeIIRMLStep1Profiles.h>


namespace TypeIIRMLMath
{

//  ---------------------- Doxygen info ----------------------
//! \fn void TypeIIRMLDecisionTree1A(const double&CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxVelocity, const double &MaxAcceleration, Step1_Profile *AppliedProfile, double *MinimalExecutionTime)
//!
//! \brief
//! This function contains the decision tree 1A of the Step 1 of the
//! Type II On-Line Trajectory Generation algorithm
//!
//! \details
//! This function calculates the minimum possible trajectory execution
//! time \f$\ _{k}t_{i}^{\,min} \f$ for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$.
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant \f$ T_{i} \f$,
//! \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant \f$ T_{i} \f$,
//! \f$\ _{k}V_{i} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant \f$ T_{i} \f$,
//! \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \param TargetVelocity
//! Target velocity value for DOF \f$ k \f$ at instant \f$ T_{i} \f$,
//! \f$\ _{k}V_{i}^{\,trgt} \f$
//!
//! \param MaxVelocity
//! Maximum allowed velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,max} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \param AppliedProfile
//! A pointer to an \c int value. The index of the motion profile, which
//! is used to achieve the minimum execution time (cf.
//! TypeIIRMLMath::Step1_Profile) will be copied to this variable.
//!
//! \param MinimalExecutionTime
//! Pointer to a \c double value: The actual result of the of this
//! function, that is, minimum possible execution time
//! \f$\ _{k}t_{i}^{\,min} \f$ will be copied to this variable.
//!
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1B()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1C()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree2()
//  ----------------------------------------------------------
void TypeIIRMLDecisionTree1A(       const double    &CurrentPosition
                                ,   const double    &CurrentVelocity
                                ,   const double    &TargetPosition
                                ,   const double    &TargetVelocity
                                ,   const double    &MaxVelocity
                                ,   const double    &MaxAcceleration
                                ,   Step1_Profile   *AppliedProfile
                                ,   double          *MinimalExecutionTime   );


}   // namespace TypeIIRMLMath

#endif

