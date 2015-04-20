//  ---------------------- Doxygen info ----------------------
//! \file RMLPositionFlags.h
//!
//! \brief
//! Header file for the class RMLPositionFlags
//!
//! \details
//! Flags to parameterize the position-based On-Line Trajectory Generation
//! algorithm.
//!
//! \sa RMLFlags.h
//! \sa RMLVelocityFlags.h
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


#ifndef __RMLPositionFlags__
#define __RMLPositionFlags__


#include <RMLFlags.h>


//  ---------------------- Doxygen info ----------------------
//! \class RMLPositionFlags
//!
//! \brief
//! Data structure containing flags to parameterize the execution of the
//! position-based On-Line Trajectory Generation algorithm
//!
//! \sa RMLFlags
//! \sa RMLVelocityFlags
//  ----------------------------------------------------------
class RMLPositionFlags : public RMLFlags
{
public:


//  ---------------------- Doxygen info ----------------------
//! \fn RMLPositionFlags(void)
//!
//! \brief
//! Constructor of the class
//!
//! \details
//! Sets the default values:
//!  - Synchronization behavior: phase-synchronization if possible (RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE)
//!  - Behavior after the target state of motion is reached: keep target velocity (RMLPositionFlags::KEEP_TARGET_VELOCITY)
//!  - Calculation of extremum motion states enabled
//!  - In case the fall back strategy becomes active: velocity to predefined values (default: zero)
//  ----------------------------------------------------------
    RMLPositionFlags(void)
    {
        this->SynchronizationBehavior                       =   RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE ;
        this->BehaviorAfterFinalStateOfMotionIsReached      =   RMLPositionFlags::KEEP_TARGET_VELOCITY      ;
        this->EnableTheCalculationOfTheExtremumMotionStates =   true                                        ;
        this->KeepCurrentVelocityInCaseOfFallbackStrategy   =   false                                       ;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLPositionFlags(void)
//!
//! \brief
//! Destructor of the class RMLPositionFlags
//  ----------------------------------------------------------
    ~RMLPositionFlags(void)
    {
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline bool operator == (const RMLPositionFlags &Flags) const
//!
//! \brief
//! Equal operator
//!
//! \return
//!\c true if all attributes of both objects are equal; \c false otherwise
//  ----------------------------------------------------------
    inline bool operator == (const RMLPositionFlags &Flags) const
    {
        return(     (RMLFlags::operator==(Flags))
                &&  (this->BehaviorAfterFinalStateOfMotionIsReached
                        ==  Flags.BehaviorAfterFinalStateOfMotionIsReached)
                &&  (this->KeepCurrentVelocityInCaseOfFallbackStrategy
                        ==  Flags.KeepCurrentVelocityInCaseOfFallbackStrategy));
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline bool operator != (const RMLPositionFlags &Flags) const
//!
//! \brief
//! Unequal operator
//!
//! \return
//!\c false if all attributes of both objects are equal; \c true otherwise
//  ----------------------------------------------------------
    inline bool operator != (const RMLPositionFlags &Flags) const
    {
        return(!(*this == Flags));
    }


//  ---------------------- Doxygen info ----------------------
//! \enum FinalMotionBehaviorEnum
//!
//! \brief
//! Enumeration whose values specify the behavior after the final state
//! of motion is reached.
//!
//! \sa BehaviorAfterFinalStateOfMotionIsReached
//  ----------------------------------------------------------
    enum FinalMotionBehaviorEnum
    {
        //! \brief
        //! The desired velocity of the target state of motion will be
        //! kept at zero acceleration.
        KEEP_TARGET_VELOCITY    =   0   ,
        //! \brief
        //! After the final state of motion is reached, a new trajectory
        //! will be computed, such that the desired state of motion will
        //! be reached again (and again, and again, etc.).
        RECOMPUTE_TRAJECTORY    =   1
    };


//  ---------------------- Doxygen info ----------------------
//! \var int BehaviorAfterFinalStateOfMotionIsReached
//!
//! \brief
//! This flag defines the behavior for the time \em after the final
//! state of motion was reached.
//!
//! \details
//! Two values are possible:
//!
//!  - RMLPositionFlags::KEEP_TARGET_VELOCITY or
//!  - RMLPositionFlags::RECOMPUTE_TRAJECTORY.
//!
//! \note
//! If the desired target velocity vector is zero, both values will
//! lead to the same behavior.
//!
//! \sa FinalMotionBehaviorEnum
//! \sa \ref page_FinalStateReachedBehavior
//  ----------------------------------------------------------
    int             BehaviorAfterFinalStateOfMotionIsReached;


//  ---------------------- Doxygen info ----------------------
//! \var bool KeepCurrentVelocityInCaseOfFallbackStrategy
//!
//! \brief
//! If true, RMLPositionInputParameters::AlternativeTargetVelocityVector
//! will be used in TypeIVRMLPosition::FallBackStrategy()
//!
//! \details
//! In case, the second safety layer becomes activated by calling
//! TypeIVRMLPosition::FallBackStrategy(), this flag can optionally be used
//! to set the current velocity vector \f$ \vec{V}_i \f$ as the
//! alternative target velocity vector for the velocity-based On-Line
//! Trajectory Generation algorithm.
//!
//! \note
//! By default, this flag is set to \c false, and in most cases,
//! is not necessary to change this. Depending on the
//! application, it may be reasonable to keep the current velocity
//! in case of an error.
//!
//! \sa RMLPositionInputParameters::AlternativeTargetVelocityVector
//! \sa \ref page_ErrorHandling
//  ----------------------------------------------------------
    bool                KeepCurrentVelocityInCaseOfFallbackStrategy;


};// class RMLPositionFlags



#endif


