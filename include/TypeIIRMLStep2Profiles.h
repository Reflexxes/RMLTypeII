//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLStep2Profiles.h
//!
//! \brief
//! Header file for the Step 2 motion profiles
//!
//! \details
//! Header file for the enumeration of all possible Step 2 motion
//! profiles used for within in the library of the Type II On-Line
//! Trajectory Generation algorithm. Further more, this file contains all
//! calculation functions of the Step 2 motion
//! profiles. These functions calculate the all trajectory parameters for a
//! specific motion profile.
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


#ifndef __TypeIIRMLStep2Profiles__
#define __TypeIIRMLStep2Profiles__


#include <TypeIIRMLPolynomial.h>
#include <TypeIIRMLMath.h>


namespace TypeIIRMLMath
{
//  ---------------------- Doxygen info ----------------------
//! \enum Step2_Profile
//!
//! \brief
//! Enumeration of all possible profiles of Step 2.
//!
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree2()
//  ----------------------------------------------------------
enum Step2_Profile
{
    //! \brief The profile is undefined
    Step2_Undefined                         =    0,
    //! \brief The profile is \em PosLinHldNegLin
    Step2_Profile_PosLinHldNegLin           =    1,
    //! \brief The profile is \em PosLinHldPosLin
    Step2_Profile_PosLinHldPosLin           =    2,
    //! \brief The profile is \em NegLinHldPosLin
    Step2_Profile_NegLinHldPosLin           =    3,
    //! \brief The profile is \em NegLinHldNegLin
    Step2_Profile_NegLinHldNegLin           =    4,
    //! \brief The profile is \em PosTrapNegLin
    Step2_Profile_PosTrapNegLin             =    5,
    //! \brief The profile is \em NegLinHldNegLinNegLin
    Step2_Profile_NegLinHldNegLinNegLin     =    6,
};


//  ---------------------- Doxygen info ----------------------
//! \fn void ProfileStep2PosLinHldNegLin(const double&CurrentTime, const double&SynchronizationTime, const double&CurrentPosition, const double&CurrentVelocity, const double&TargetPosition, const double&TargetVelocity, const double&MaxAcceleration, MotionPolynomials*PolynomialsLocal, const bool&Inverted)
//!
//! \brief
//! Parameterization of the Step 2 velocity profile \em PosLinHldNegLin
//! (leaf of the Step 2 decision tree)
//!
//! \param CurrentTime
//! Time in seconds required for preceding intermediate trajectory
//! segments, \f$ ^{\left(\Lambda+1\right)}_{\ \ \ \ \ k}t_i \f$, where
//! \f$ \Lambda \f$ is the number of preceding trajectory segments.
//!
//! \param SynchronizationTime
//! Synchronization time in seconds, \f$ t_i^{\,sync} \f$
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
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \param PolynomialsLocal
//! Pointer to a \c MotionPolynomials object, which contains the
//! complete trajectory for one single DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$ _k{\cal M}_{i}(t) \f$. This function will
//! set-up all remaining trajectory parameters.
//!
//! \param Inverted
//! A \c bool value that indicates whether the input values have been
//! flipped even or odd times prior to the execution of this function
//! (cf. TypeIIRMLMath::NegateStep2()). The trajectory parameters in
//! \c PolynomialsLocal will be set-up correspondingly.
//!
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree2()
//  ----------------------------------------------------------
void ProfileStep2PosLinHldNegLin(       const double        &CurrentTime
                                    ,   const double        &SynchronizationTime
                                    ,   const double        &CurrentPosition
                                    ,   const double        &CurrentVelocity
                                    ,   const double        &TargetPosition
                                    ,   const double        &TargetVelocity
                                    ,   const double        &MaxAcceleration
                                    ,   MotionPolynomials   *PolynomialsLocal
                                    ,   const bool          &Inverted               );


//  ---------------------- Doxygen info ----------------------
//! \fn void ProfileStep2PosLinHldPosLin(const double&CurrentTime, const double&SynchronizationTime, const double&CurrentPosition, const double&CurrentVelocity, const double&TargetPosition, const double&TargetVelocity, const double&MaxAcceleration, MotionPolynomials*PolynomialsLocal, const bool&Inverted)
//!
//! \brief
//! Parameterization of the Step 2 velocity profile \em PosLinHldNegLin
//! (leaf of the Step 2 decision tree)
//!
//! \copydetails TypeIIRMLMath::ProfileStep2PosLinHldNegLin()
//  ----------------------------------------------------------
void ProfileStep2PosLinHldPosLin(       const double        &CurrentTime
                                    ,   const double        &SynchronizationTime
                                    ,   const double        &CurrentPosition
                                    ,   const double        &CurrentVelocity
                                    ,   const double        &TargetPosition
                                    ,   const double        &TargetVelocity
                                    ,   const double        &MaxAcceleration
                                    ,   MotionPolynomials   *PolynomialsLocal
                                    ,   const bool          &Inverted               );


//  ---------------------- Doxygen info ----------------------
//! \fn void ProfileStep2NegLinHldPosLin(const double&CurrentTime, const double&SynchronizationTime, const double&CurrentPosition, const double&CurrentVelocity, const double&TargetPosition, const double&TargetVelocity, const double&MaxAcceleration, MotionPolynomials*PolynomialsLocal, const bool&Inverted)
//!
//! \brief
//! Parameterization of the Step 2 velocity profile \em NegLinHldPosLin
//! (leaf of the Step 2 decision tree)
//!
//! \copydetails TypeIIRMLMath::ProfileStep2PosLinHldNegLin()
//  ----------------------------------------------------------
void ProfileStep2NegLinHldPosLin(       const double        &CurrentTime
                                    ,   const double        &SynchronizationTime
                                    ,   const double        &CurrentPosition
                                    ,   const double        &CurrentVelocity
                                    ,   const double        &TargetPosition
                                    ,   const double        &TargetVelocity
                                    ,   const double        &MaxAcceleration
                                    ,   MotionPolynomials   *PolynomialsLocal
                                    ,   const bool          &Inverted               );


//  ---------------------- Doxygen info ----------------------
//! \fn void ProfileStep2NegLinHldNegLin(const double&CurrentTime, const double&SynchronizationTime, const double&CurrentPosition, const double&CurrentVelocity, const double&TargetPosition, const double&TargetVelocity, const double&MaxAcceleration, MotionPolynomials*PolynomialsLocal, const bool&Inverted)
//!
//! \brief
//! Parameterization of the Step 2 velocity profile \em NegLinHldNegLin
//! (leaf of the Step 2 decision tree)
//!
//! \copydetails TypeIIRMLMath::ProfileStep2PosLinHldNegLin()
//  ----------------------------------------------------------
void ProfileStep2NegLinHldNegLin(       const double        &CurrentTime
                                    ,   const double        &SynchronizationTime
                                    ,   const double        &CurrentPosition
                                    ,   const double        &CurrentVelocity
                                    ,   const double        &TargetPosition
                                    ,   const double        &TargetVelocity
                                    ,   const double        &MaxAcceleration
                                    ,   MotionPolynomials   *PolynomialsLocal
                                    ,   const bool          &Inverted               );


//  ---------------------- Doxygen info ----------------------
//! \fn void ProfileStep2PosTrapNegLin(const double&CurrentTime, const double&SynchronizationTime, const double&CurrentPosition, const double&CurrentVelocity, const double&TargetPosition, const double&TargetVelocity, const double&MaxAcceleration, MotionPolynomials*PolynomialsLocal, const bool&Inverted)
//!
//! \brief
//! Parameterization of the Step 2 velocity profile \em PosTrapNegLin
//! (leaf of the Step 2 decision tree)
//!
//! \copydetails TypeIIRMLMath::ProfileStep2PosLinHldNegLin()
//  ----------------------------------------------------------
void ProfileStep2PosTrapNegLin(         const double        &CurrentTime
                                    ,   const double        &SynchronizationTime
                                    ,   const double        &CurrentPosition
                                    ,   const double        &CurrentVelocity
                                    ,   const double        &TargetPosition
                                    ,   const double        &TargetVelocity
                                    ,   const double        &MaxAcceleration
                                    ,   MotionPolynomials   *PolynomialsLocal
                                    ,   const bool          &Inverted               );


//  ---------------------- Doxygen info ----------------------
//! \fn void ProfileStep2NegLinHldNegLinNegLin(const double&CurrentTime, const double&SynchronizationTime, const double&CurrentPosition, const double&CurrentVelocity, const double&TargetPosition, const double&TargetVelocity, const double&MaxAcceleration, MotionPolynomials*PolynomialsLocal, const bool&Inverted)
//!
//! \brief
//! Parameterization of the Step 2 velocity profile \em NegLinHldNegLinNegLin
//! (leaf of the Step 2 decision tree)
//!
//! \copydetails TypeIIRMLMath::ProfileStep2PosLinHldNegLin()
//  ----------------------------------------------------------
void ProfileStep2NegLinHldNegLinNegLin(     const double        &CurrentTime
                                        ,   const double        &SynchronizationTime
                                        ,   const double        &CurrentPosition
                                        ,   const double        &CurrentVelocity
                                        ,   const double        &TargetPosition
                                        ,   const double        &TargetVelocity
                                        ,   const double        &MaxAcceleration
                                        ,   MotionPolynomials   *PolynomialsLocal
                                        ,   const bool          &Inverted               );

}   // namespace TypeIIRMLMath

#endif

