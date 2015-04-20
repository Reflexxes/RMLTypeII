//  ---------------------- Doxygen info ----------------------
//! \file RMLPositionOutputParameters.h
//!
//! \brief
//! Header file for the class RMLPositionOutputParameters
//!
//! \details
//! The class RMLPositionOutputParameters is derived from the class
//! RMLOutputParameters and constitutes a part of the interface for the
//! position-based On-Line Trajectory Generation algorithm.
//!
//! \sa RMLInputParameters
//! \sa RMLPositionInputParameters
//! \sa RMLVelocityOutputParameters
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


#ifndef __RMLPositionOutputParameters__
#define __RMLPositionOutputParameters__


#include <RMLOutputParameters.h>


//  ---------------------- Doxygen info ----------------------
//! \class RMLPositionOutputParameters
//!
//! \brief
//! Class for the output parameters of the position-based On-Line
//! Trajectory Generation algorithm
//!
//! \details
//! The class RMLPositionOutputParameters is derived from the class
//! RMLOutputParameters and constitutes a part of the interface for the
//! position-based On-Line Trajectory Generation algorithm.
//!
//! \sa ReflexxesAPI
//! \sa RMLOutputParameters
//! \sa RMLVelocityOutputParameters
//! \sa RMLPositionInputParameters
//  ----------------------------------------------------------
class RMLPositionOutputParameters : public RMLOutputParameters
{

public:

//  ---------------------- Doxygen info ----------------------
//! \fn RMLPositionOutputParameters(const unsigned int DegreesOfFreedom)
//!
//! \brief
//! Constructor of class RMLPositionOutputParameters
//!
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//!
//! \param DegreesOfFreedom
//! Specifies the number of degrees of freedom
//  ----------------------------------------------------------
    RMLPositionOutputParameters(const unsigned int DegreesOfFreedom) : RMLOutputParameters(DegreesOfFreedom)
    {
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLPositionOutputParameters(const RMLPositionOutputParameters &OP)
//!
//! \brief
//! Copy constructor of class RMLPositionOutputParameters
//!
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//!
//! \param OP
//! Object to be copied
//  ----------------------------------------------------------
    RMLPositionOutputParameters(const RMLPositionOutputParameters &OP) : RMLOutputParameters(OP)
    {
    }


//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLPositionOutputParameters(void)
//!
//! \brief
//! Destructor of class RMLPositionOutputParameters
//  ----------------------------------------------------------
    ~RMLPositionOutputParameters(void)
    {
    }


//  ---------------------- Doxygen info ----------------------
//! \fn void Echo(FILE* FileHandler = stdout) const
//!
//! \brief
//! \copybrief RMLOutputParameters::Echo()
//!
//! \details
//! \copydetails RMLOutputParameters::Echo()
//  ----------------------------------------------------------
    void Echo(FILE* FileHandler = stdout) const
    {
        if (FileHandler == NULL)
        {
            return;
        }

        RMLOutputParameters::Echo(FileHandler);

        return;
    }


};// class RMLPositionOutputParameters



#endif


