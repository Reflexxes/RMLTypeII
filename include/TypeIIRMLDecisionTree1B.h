//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLDecisionTree1B.h
//!
//! \brief
//! Header file for the Step 1 decision tree 1B of the Type II On-Line
//! Trajectory Generation algorithm
//!
//! \details
//! Header file for the function TypeIIRMLMath::TypeIIRMLDecisionTree1B(),
//! which contains a part of the second decision tree (1B) of the Step 1
//! of the On-Line Trajectory Generation algorithm. It calculates the
//! beginning of a possible inoperative time interval. Details on this
//! methodology may be found in\n
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


#ifndef __TypeIIRMLDecisionTree1B__
#define __TypeIIRMLDecisionTree1B__


namespace TypeIIRMLMath
{

//  ---------------------- Doxygen info ----------------------
//! \fn void TypeIIRMLDecisionTree1B(const double&CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxVelocity, const double &MaxAcceleration, double *MaximalExecutionTime)
//!
//! \brief
//! This function contains the decision tree 1B of the Step 1 of the
//! Type II On-Line Trajectory Generation algorithm
//!
//! \details
//! This function calculates the beginning of possible inoperative time
//! interval, that is, the time \f$\ _{k}t_{i}^{\,begin} \f$ for DOF
//! \f$ k \f$ at instant \f$ T_{i} \f$.
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
//! \param MaximalExecutionTime
//! Pointer to a \c double value: The actual result of the of this
//! function, that is, the value of the beginning of possible inoperative
//! time interval, that is, the time \f$\ _{k}t_{i}^{\,begin} \f$ for DOF
//! \f$ k \f$ at instant \f$ T_{i} \f$. If no inoperative time interval
//! is existent, a value of \c RML_INFINITY will be written to this
//! variable.
//!
//! \return
//! For the case, an error during the calculations of one motion profile
//! happens, the return value will be \c true, otherwise \c false.
//!
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1A()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1C()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree2()
//  ----------------------------------------------------------
    void TypeIIRMLDecisionTree1B(       const double    &CurrentPosition
                                    ,   const double    &CurrentVelocity
                                    ,   const double    &TargetPosition
                                    ,   const double    &TargetVelocity
                                    ,   const double    &MaxVelocity
                                    ,   const double    &MaxAcceleration
                                    ,   double          *MaximalExecutionTime);


}   // namespace TypeIIRMLMath

#endif

