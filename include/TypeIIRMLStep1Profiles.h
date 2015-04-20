//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLStep1Profiles.h
//!
//! \brief
//! Header file for the Step 1 motion profiles
//!
//! \details
//! Header file for the enumeration of all possible Step 1 motion
//! profiles used for within in the library of the Type II On-Line
//! Trajectory Generation algorithm. Further more, this file contains all
//! calculation functions of the Step 1 motion
//! profiles, that is, the leaves of the Step 1 decision trees 1A, 1B, and
//! 1C. These functions calculate the minimum or maximum trajectory
//! execution time for a specific motion profile.
//!
//! The enumeration as well as all profile functions are part of the
//! namespace TypeIIRMLMath.
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


#ifndef __TypeIIRMLStep1Profiles__
#define __TypeIIRMLStep1Profiles__


#include <TypeIIRMLMath.h>


namespace TypeIIRMLMath
{
//  ---------------------- Doxygen info ----------------------
//! \enum Step1_Profile
//!
//! \brief
//! Enumeration of all possible profiles of Step 1 (A, B, and C).
//!
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1A()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1B()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1C()
//  ----------------------------------------------------------
enum Step1_Profile
{
    //! \brief The profile is undefined
    Step1_Undefined                 =    0,
    //! \brief The profile is \em PosLinHldNegLin
    Step1_Profile_PosLinHldNegLin   =    1,
    //! \brief The profile is \em PosLinNegLin
    Step1_Profile_PosLinNegLin      =    2,
    //! \brief The profile is \em PosTriNegLin
    Step1_Profile_PosTriNegLin      =    3,
    //! \brief The profile is \em PosTrapNegLin
    Step1_Profile_PosTrapNegLin     =    4,
    //! \brief The profile is \em NegLinHldPosLin
    Step1_Profile_NegLinHldPosLin   =    5,
    //! \brief The profile is \em NegLinPosLin
    Step1_Profile_NegLinPosLin      =    6,
    //! \brief The profile is \em NegTriPosLin
    Step1_Profile_NegTriPosLin      =    7,
    //! \brief The profile is \em NegTrapPosLin
    Step1_Profile_NegTrapPosLin     =    8
};


//  ---------------------- Doxygen info ----------------------
//! \fn double ProfileStep1PosLinHldNegLin(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxVelocity, const double &MaxAcceleration)
//!
//! \brief
//! Calculates the execution time of the \em PosLinHldNegLin velocity profile
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \param TargetVelocity
//! Target velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,trgt} \f$
//!
//! \param MaxVelocity
//! Maximum allowed velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,max} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \return
//! Execution time for this profile in seconds
//!
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1A()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1B()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1C()
//  ----------------------------------------------------------
double ProfileStep1PosLinHldNegLin(     const double &CurrentPosition
                                    ,   const double &CurrentVelocity
                                    ,   const double &TargetPosition
                                    ,   const double &TargetVelocity
                                    ,   const double &MaxVelocity
                                    ,   const double &MaxAcceleration   );


//  ---------------------- Doxygen info ----------------------
//! \fn double ProfileStep1PosLinNegLin(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxAcceleration)
//!
//! \brief
//! Calculates the execution time of the \em PosLinNegLin velocity profile
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \param TargetVelocity
//! Target velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,trgt} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \return
//! Execution time for this profile in seconds
//!
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1A()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1B()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1C()
//  ----------------------------------------------------------
double ProfileStep1PosLinNegLin(        const double &CurrentPosition
                                    ,   const double &CurrentVelocity
                                    ,   const double &TargetPosition
                                    ,   const double &TargetVelocity
                                    ,   const double &MaxAcceleration   );


//  ---------------------- Doxygen info ----------------------
//! \fn double ProfileStep1PosTriNegLin(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxAcceleration)
//!
//! \brief
//! Calculates the execution time of the \em PosTriNegLin velocity profile
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \param TargetVelocity
//! Target velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,trgt} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \return
//! Execution time for this profile in seconds
//!
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1A()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1B()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1C()
//  ----------------------------------------------------------
double ProfileStep1PosTriNegLin(        const double &CurrentPosition
                                    ,   const double &CurrentVelocity
                                    ,   const double &TargetPosition
                                    ,   const double &TargetVelocity
                                    ,   const double &MaxAcceleration   );


//  ---------------------- Doxygen info ----------------------
//! \fn double ProfileStep1PosTrapNegLin(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxVelocity, const double &MaxAcceleration)
//!
//! \brief
//! Calculates the execution time of the \em PosTrapNegLin velocity profile
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \param TargetVelocity
//! Target velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,trgt} \f$
//!
//! \param MaxVelocity
//! Maximum allowed velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,max} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \return
//! Execution time for this profile in seconds
//!
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1A()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1B()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1C()
//  ----------------------------------------------------------
double ProfileStep1PosTrapNegLin(       const double &CurrentPosition
                                    ,   const double &CurrentVelocity
                                    ,   const double &TargetPosition
                                    ,   const double &TargetVelocity
                                    ,   const double &MaxVelocity
                                    ,   const double &MaxAcceleration   );


//  ---------------------- Doxygen info ----------------------
//! \fn double ProfileStep1NegLinPosLin(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxAcceleration)
//!
//! \brief
//! Calculates the execution time of the \em NegLinPosLin velocity profile
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \param TargetVelocity
//! Target velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,trgt} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \return
//! Execution time for this profile in seconds
//!
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1A()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1B()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1C()
//  ----------------------------------------------------------
double ProfileStep1NegLinPosLin(        const double &CurrentPosition
                                    ,   const double &CurrentVelocity
                                    ,   const double &TargetPosition
                                    ,   const double &TargetVelocity
                                    ,   const double &MaxAcceleration   );


//  ---------------------- Doxygen info ----------------------
//! \fn bool IsSolutionForProfile_PosLinHldNegLin_Possible(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxVelocity, const double &MaxAcceleration)
//!
//! \brief
//! Checks whether a valid solution for the \em PosLinHldNegLin velocity profile
//! would be possible
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \param TargetVelocity
//! Target velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,trgt} \f$
//!
//! \param MaxVelocity
//! Maximum allowed velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,max} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \return
//!  - \c true if a valid solution would be possible
//!  - \c false otherwise
//!
//! \sa TypeIIRMLPosition::Step1()
//  ----------------------------------------------------------
bool IsSolutionForProfile_PosLinHldNegLin_Possible(     const double &CurrentPosition
                                                    ,   const double &CurrentVelocity
                                                    ,   const double &TargetPosition
                                                    ,   const double &TargetVelocity
                                                    ,   const double &MaxVelocity
                                                    ,   const double &MaxAcceleration   );


//  ---------------------- Doxygen info ----------------------
//! \fn bool IsSolutionForProfile_PosLinNegLin_Possible(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxVelocity, const double &MaxAcceleration)
//!
//! \brief
//! Checks whether a valid solution for the \em PosLinNegLin velocity profile
//! would be possible
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \param TargetVelocity
//! Target velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,trgt} \f$
//!
//! \param MaxVelocity
//! Maximum allowed velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,max} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \return
//!  - \c true if a valid solution would be possible
//!  - \c false otherwise
//!
//! \sa TypeIIRMLPosition::Step1()
//  ----------------------------------------------------------
bool IsSolutionForProfile_PosLinNegLin_Possible(    const double &CurrentPosition
                                                ,   const double &CurrentVelocity
                                                ,   const double &TargetPosition
                                                ,   const double &TargetVelocity
                                                ,   const double &MaxVelocity
                                                ,   const double &MaxAcceleration   );


//  ---------------------- Doxygen info ----------------------
//! \fn bool IsSolutionForProfile_PosTriNegLin_Possible(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxVelocity, const double &MaxAcceleration)
//!
//! \brief
//! Checks whether a valid solution for the \em PosTriNegLin velocity profile
//! would be possible
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \param TargetVelocity
//! Target velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,trgt} \f$
//!
//! \param MaxVelocity
//! Maximum allowed velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,max} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \return
//!  - \c true if a valid solution would be possible
//!  - \c false otherwise
//!
//! \sa TypeIIRMLPosition::Step1()
//  ----------------------------------------------------------
bool IsSolutionForProfile_PosTriNegLin_Possible(    const double &CurrentPosition
                                                ,   const double &CurrentVelocity
                                                ,   const double &TargetPosition
                                                ,   const double &TargetVelocity
                                                ,   const double &MaxVelocity
                                                ,   const double &MaxAcceleration   );


//  ---------------------- Doxygen info ----------------------
//! \fn bool IsSolutionForProfile_PosTrapNegLin_Possible(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxVelocity, const double &MaxAcceleration)
//!
//! \brief
//! Checks whether a valid solution for the \em PosTrapNegLin velocity profile
//! would be possible
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \param TargetVelocity
//! Target velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,trgt} \f$
//!
//! \param MaxVelocity
//! Maximum allowed velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,max} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \return
//!  - \c true if a valid solution would be possible
//!  - \c false otherwise
//!
//! \sa TypeIIRMLPosition::Step1()
//  ----------------------------------------------------------
bool IsSolutionForProfile_PosTrapNegLin_Possible(       const double &CurrentPosition
                                                    ,   const double &CurrentVelocity
                                                    ,   const double &TargetPosition
                                                    ,   const double &TargetVelocity
                                                    ,   const double &MaxVelocity
                                                    ,   const double &MaxAcceleration   );

}   // namespace TypeIIRMLMath

#endif

