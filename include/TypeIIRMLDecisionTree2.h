//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLDecisionTree2.h
//!
//! \brief
//! Header file for the Step 2 decision tree of the Type II On-Line
//! Trajectory Generation algorithm (time-synchronized case)
//!
//! \details
//! Header file for the function TypeIIRMLMath::TypeIIRMLDecisionTree2(),
//! which contains the decision tree of the Step 2 of the On-Line
//! Trajectory Generation algorithm. It synchronizes all selected
//! degrees of freedom to the synchronization time (cf. Step 1). Details
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


#ifndef __TypeIIRMLDecisionTree2__
#define __TypeIIRMLDecisionTree2__


#include <TypeIIRMLPolynomial.h>


namespace TypeIIRMLMath
{

//  ---------------------- Doxygen info ----------------------
//! \fn void TypeIIRMLDecisionTree2(const double&CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxVelocity, const double &MaxAcceleration, const double &SynchronizationTime, MotionPolynomials *PolynomialsInternal)
//!
//! \brief
//! This function contains the decision tree of the Step 2 of the
//! Type II On-Line Trajectory Generation algorithm
//!
//! \details
//! This function synchronizes the trajectory for one single degree of
//! freedom to the synchronization time \f$ t_{i}^{\,sync} \f$ and
//! calculates all trajectory parameters.
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
//! \param SynchronizationTime
//! The synchronization time \f$ t_{i}^{\,sync} \f$ that was calculated
//! in Step 1 of the On-Line Trajectory Generation algorithm (in
//! seconds).
//!
//! \param PolynomialsInternal
//! A pointer to a \c MotionPolynomials object (cf.
//! TypeIIRMLMath::MotionPolynomials). All trajectory parameters of the
//! synchronized Type II trajectory will be written into this object.
//!
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1A()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1B()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1C()
//  ----------------------------------------------------------
    void TypeIIRMLDecisionTree2(        const double        &CurrentPosition
                                    ,   const double        &CurrentVelocity
                                    ,   const double        &TargetPosition
                                    ,   const double        &TargetVelocity
                                    ,   const double        &MaxVelocity
                                    ,   const double        &MaxAcceleration
                                    ,   const double        &SynchronizationTime
                                    ,   MotionPolynomials   *PolynomialsInternal);

}   // namespace TypeIIRMLMath

#endif

