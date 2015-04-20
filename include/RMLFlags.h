//  ---------------------- Doxygen info ----------------------
//! \file RMLFlags.h
//!
//! \brief
//! Header file for the class RMLFlags
//!
//! \details
//! Flags to parameterize the On-Line Trajectory Generation algorithms.
//! This basis data structure is inherited to the structures
//! RMLPositionFlags and RMLVelocityFlags
//!
//! \sa RMLPositionFlags
//! \sa RMLVelocityFlags
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


#ifndef __RMLFlags__
#define __RMLFlags__


//  ---------------------- Doxygen info ----------------------
//! \class RMLFlags
//!
//! \brief
//! Data structure containing flags to parameterize the execution of the
//! On-Line Trajectory Generation algorithm
//  ----------------------------------------------------------
class RMLFlags
{

protected:

//  ---------------------- Doxygen info ----------------------
//! \fn RMLFlags(void)
//!
//! \brief
//! Constructor of the class
//!
//! \note
//! This is only the base class for the classes\n\n
//! <ul>
//!  <li>RMLPositionFlags and</li>
//!  <li>RMLVelocityFlags,\n\n</li>
//! </ul>
//! such that the constructor is declared \c protected.
//  ----------------------------------------------------------
    RMLFlags(void)
    {
    }

public:


//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLFlags(void)
//!
//! \brief
//! Destructor of the class RMLFlags
//  ----------------------------------------------------------
    ~RMLFlags(void)
    {
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline bool operator == (const RMLFlags &Flags) const
//!
//! \brief
//! Equal operator
//!
//! \return
//!\c true if all attributes of both objects are equal; \c false otherwise
//  ----------------------------------------------------------
    inline bool operator == (const RMLFlags &Flags) const
    {
        return (    (this->SynchronizationBehavior
                        ==  Flags.SynchronizationBehavior)
                &&  (this->EnableTheCalculationOfTheExtremumMotionStates
                        ==  Flags.EnableTheCalculationOfTheExtremumMotionStates)    );
    }

//  ---------------------- Doxygen info ----------------------
//! \fn inline bool operator != (const RMLFlags &Flags) const
//!
//! \brief
//! Unequal operator
//!
//! \return
//!\c false if all attributes of both objects are equal; \c true otherwise
//  ----------------------------------------------------------
    inline bool operator != (const RMLFlags &Flags) const
    {
        return(!(*this == Flags));
    }


//  ---------------------- Doxygen info ----------------------
//! \enum SyncBehaviorEnum
//!
//! \brief
//! Enumeration whose values specify the synchronization method of the
//! On-Line Trajectory Generation algorithm
//!
//! \sa RMLFlags::SynchronizationBehavior
//! \sa \ref page_SynchronizationBehavior
//! \sa \ref page_PSIfPossible
//  ----------------------------------------------------------
    enum SyncBehaviorEnum
    {
        //! \brief
        //! This is the default value. If it is possible to calculate a
        //! phase-synchronized (i.e., homothetic) trajectory, the algorithm
        //! will generate it. A more detailed description of this flag can
        //!  be found on the page \ref page_PSIfPossible.
        PHASE_SYNCHRONIZATION_IF_POSSIBLE   =   0   ,
        //! \brief
        //! Even if it is possible to calculate a phase-synchronized
        //! trajectory, only a time-synchronized trajectory will be
        //! provided. More information can be found on page
        //! \ref page_SynchronizationBehavior.
        ONLY_TIME_SYNCHRONIZATION           =   1   ,
        //! \brief
        //! Only phase-synchronized trajectories are allowed. If it is not
        //! possible calculate a phase-synchronized trajectory, an error
        //! will be returned (but feasible, steady, and continuous
        //! output values will still be computed). More information can be
        //! found on page \ref page_SynchronizationBehavior.
        ONLY_PHASE_SYNCHRONIZATION          =   2   ,
        //! \brief
        //! No synchronization will be performed, and all selected degrees
        //! of freedom are treated independently.
        NO_SYNCHRONIZATION                  =   3
    };


//  ---------------------- Doxygen info ----------------------
//! \var unsigned char SynchronizationBehavior
//!
//! \brief
//! This value specifies the desired synchronization behavior
//!
//! \details
//! The following values are possible:
//!
//! - RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE
//! - RMLFlags::ONLY_TIME_SYNCHRONIZATION
//! - RMLFlags::ONLY_PHASE_SYNCHRONIZATION
//! - RMLFlags::NO_SYNCHRONIZATION
//!
//! Further details about time- and phase-synchronization can be found
//! in the section on \ref page_SynchronizationBehavior and at
//! RMLFlags::SyncBehaviorEnum.
//!
//! \sa RMLPositionFlags
//! \sa RMLVelocityFlags
//! \sa RMLPositionOutputParameters::IsTrajectoryPhaseSynchronized()
//! \sa RMLVelocityOutputParameters::IsTrajectoryPhaseSynchronized()
//! \sa \ref page_SynchronizationBehavior
//! \sa \ref page_PSIfPossible
//  ----------------------------------------------------------
    unsigned char       SynchronizationBehavior;


//  ---------------------- Doxygen info ----------------------
//! \var bool EnableTheCalculationOfTheExtremumMotionStates
//!
//! \brief
//! A flag to enable or disable the calculation of the extremum states of motion of the
//! currently calculated trajectory
//!
//! \details
//! If this flag is set, a call of
//!
//! - ReflexxesAPI::RMLPosition() or
//! - ReflexxesAPI::RMLVelocity()
//!
//! will not only calculate the desired trajectory, but also the motion states
//! at the extremum positions. These values will be accessible through the methods
//! of the classes
//!
//! - RMLPositionOutputParameters and
//! - RMLVelocityOutputParameters
//!
//! \remark
//! In average, the calculation of the extremum states of motion takes about five percent
//! the overall computational effort of the OTG algorithm, such that the maximum
//! execution time of the algorithm will be decreased by disabling this flag.
//!
//! \note
//! The default value for this flag is \c true.
//!
//! \sa \ref page_OutputValues
//! \sa \ref page_FinalStateReachedBehavior
//! \sa \ref page_PSIfPossible\n\n
//  ----------------------------------------------------------
    bool                EnableTheCalculationOfTheExtremumMotionStates;

};// class RMLFlags



#endif


