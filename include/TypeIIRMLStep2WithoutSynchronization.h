// ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLStep2WithoutSynchronization.h
//!
//! \brief
//! Header file file for the declaration of the function
//! TypeIIRMLMath::Step2WithoutSynchronization().
//!
//! \details
//! Header file for the function TypeIIRMLMath::Step2WithoutSynchronization(),
//! which sets up all polynomial parameters in the case of non-synchronized
//! trajectories (cf. RMLFlags::NO_SYNCHRONIZATION).
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


#ifndef __TypeIIRMLStep2WithoutSynchronization__
#define __TypeIIRMLStep2WithoutSynchronization__


#include <TypeIIRMLMath.h>
#include <TypeIIRMLStep1Profiles.h>
#include <TypeIIRMLPolynomial.h>


namespace TypeIIRMLMath
{

// ---------------------- Doxygen info ----------------------
//! \fn void Step2WithoutSynchronization(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxVelocity, const double &MaxAcceleration, const TypeIIRMLMath::Step1_Profile &UsedProfile, const double &MinimumExecutionTime, MotionPolynomials *PolynomialsInternal)
//!
//! \brief
//! This function contains sets of trajectory parameters (i.e., all
//! polynomial coefficients) in the case of non-synchronized trajectories.
//!
//! \details
//! Based on all input values for one selected degree of freedom and on the
//! velocity profile that was applied in Step 1A, all trajectory
//! parameters, that is, all polynomial coefficients, are set-up. This
//! method is only applied in the case of non-synchronized trajectories
//! (cf. RMLFlags::NO_SYNCHRONIZATION).
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
//! \param UsedProfile
//! The ID of the used acceleration profile in Step 1A (cf.
//! RMLMath::Step1_Profile).
//!
//! \param MinimumExecutionTime
//! Minimum execution time \f$ t_{i}^{\,min} \f$ for the current degree of
//! freedom \f$ k \f$ (i.e., the result of profiles of the Step 1A
//! 1A decision tree.
//!
//! \param PolynomialsInternal
//! A pointer to a \c MotionPolynomials object (cf.
//! TypeIIRMLMath::MotionPolynomials). All trajectory parameters of the
//! synchronized Type II trajectory will be written into this object.
//!
//! \sa RMLFlags::NO_SYNCHRONIZATION
// ----------------------------------------------------------
 void Step2WithoutSynchronization(      const double                        &CurrentPosition
                                    ,   const double                        &CurrentVelocity
                                    ,   const double                        &TargetPosition
                                    ,   const double                        &TargetVelocity
                                    ,   const double                        &MaxVelocity
                                    ,   const double                        &MaxAcceleration
                                    ,   const TypeIIRMLMath::Step1_Profile  &UsedProfile
                                    ,   const double                        &MinimumExecutionTime
                                    ,   MotionPolynomials                   *PolynomialsInternal);

}   // namespace TypeIIRMLMath

#endif

