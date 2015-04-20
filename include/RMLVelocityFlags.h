//  ---------------------- Doxygen info ----------------------
//! \file RMLVelocityFlags.h
//!
//! \brief
//! Header file for the class RMLVelocityFlags
//!
//! \details
//! Flags to parameterize the velocity-based On-Line Trajectory Generation
//! algorithm.
//!
//! \sa RMLFlags.h
//! \sa RMLPositionFlags.h
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


#ifndef __RMLVelocityFlags__
#define __RMLVelocityFlags__


#include <RMLFlags.h>


//  ---------------------- Doxygen info ----------------------
//! \class RMLVelocityFlags
//!
//! \brief
//! Data structure containing flags to parameterize the execution of the
//! velocity-based On-Line Trajectory Generation algorithm
//!
//! \sa RMLFlags
//! \sa RMLPositionFlags
//  ----------------------------------------------------------
class RMLVelocityFlags : public RMLFlags
{
public:


//  ---------------------- Doxygen info ----------------------
//! \fn RMLVelocityFlags(void)
//!
//! \brief
//! Constructor of the class
//!
//! \details
//! Sets the default values:
//!  - Synchronization behavior:  no synchronization (RMLFlags::NO_SYNCHRONIZATION)
//!  - Calculation of extremum motion states enabled
//  ----------------------------------------------------------
    RMLVelocityFlags(void)
    {
        this->SynchronizationBehavior                       =   RMLFlags::NO_SYNCHRONIZATION    ;
        this->EnableTheCalculationOfTheExtremumMotionStates =   true                            ;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLVelocityFlags(void)
//!
//! \brief
//! Destructor of the class RMLVelocityFlags
//  ----------------------------------------------------------
    ~RMLVelocityFlags(void)
    {
    }

//  ---------------------- Doxygen info ----------------------
//! \fn inline bool operator == (const RMLVelocityFlags &Flags) const
//!
//! \brief
//! Equal operator
//!
//! \return
//!\c true if all attributes of both objects are equal; \c false otherwise
//  ----------------------------------------------------------
    inline bool operator == (const RMLVelocityFlags &Flags) const
    {
        return(RMLFlags::operator==(Flags));
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline bool operator != (const RMLVelocityFlags &Flags) const
//!
//! \brief
//! Unequal operator
//!
//! \return
//!\c false if all attributes of both objects are equal; \c true otherwise
//  ----------------------------------------------------------
    inline bool operator != (const RMLVelocityFlags &Flags) const
    {
        return(!(*this == Flags));
    }

};// class RMLVelocityFlags



#endif


