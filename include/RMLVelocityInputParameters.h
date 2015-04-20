//  ---------------------- Doxygen info ----------------------
//! \file RMLVelocityInputParameters.h
//!
//! \brief
//! Header file for the class RMLVelocityInputParameters
//!
//! \details
//! The class RMLVelocityInputParameters is derived from the class
//! RMLInputParameters and constitutes a part of the interface for the
//! velocity-based On-Line Trajectory Generation algorithm.
//!
//! \sa RMLInputParameters
//! \sa RMLVelocityOutputParameters
//! \sa RMLPositionInputParameters
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


#ifndef __RMLVelocityInputParameters__
#define __RMLVelocityInputParameters__


#include <RMLInputParameters.h>
#include <math.h>


//  ---------------------- Doxygen info ----------------------
//! \class RMLVelocityInputParameters
//!
//! \brief
//! Class for the input parameters of the velocity-based On-Line
//! Trajectory Generation algorithm
//!
//! \details
//! The class RMLVelocityInputParameters is derived from the class
//! RMLInputParameters and constitutes a part of the interface for the
//! velocity-based On-Line Trajectory Generation algorithm.\n\n
//!
//! A detailed description can be found at \ref page_InputValues.
//!
//! \sa ReflexxesAPI
//! \sa RMLInputParameters
//! \sa RMLPositionInputParameters
//! \sa RMLVelocityOutputParameters
//! \sa \ref page_InputValues
//  ----------------------------------------------------------
class RMLVelocityInputParameters : public RMLInputParameters
{
public:


//  ---------------------- Doxygen info ----------------------
//! \fn RMLVelocityInputParameters(const unsigned int DegreesOfFreedom)
//!
//! \brief
//! Constructor of class RMLVelocityInputParameters
//!
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//!
//! \param DegreesOfFreedom
//! Specifies the number of degrees of freedom
//  ----------------------------------------------------------
    RMLVelocityInputParameters(const unsigned int DegreesOfFreedom) : RMLInputParameters(DegreesOfFreedom)
    {
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLVelocityInputParameters(const RMLVelocityInputParameters& IP)
//!
//! \brief
//! Copy constructor of class RMLVelocityInputParameters
//!
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//!
//! \param IP
//! Object to be copied
//  ----------------------------------------------------------
    RMLVelocityInputParameters(const RMLVelocityInputParameters &IP) : RMLInputParameters(IP)
    {
    }


//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLVelocityInputParameters(void)
//!
//! \brief
//! Destructor of class RMLVelocityInputParameters
//  ----------------------------------------------------------
    ~RMLVelocityInputParameters(void)
    {
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLVelocityInputParameters &operator = (const RMLVelocityInputParameters &IP)
//!
//! \brief
//! Copy operator
//!
//! \param IP
//! RMLVelocityInputParameters object to be copied
//  ----------------------------------------------------------
    inline RMLVelocityInputParameters &operator = (const RMLVelocityInputParameters &IP)
    {
        RMLInputParameters::operator=(IP);

        return(*this);
    }



// #############################################################################


//  ---------------------- Doxygen info ----------------------
//! \fn bool CheckForValidity(void) const
//!
//! \brief
//! Checks the input parameters for validity
//!
//! \details
//! \copydetails RMLPositionInputParameters::CheckForValidity()
//  ----------------------------------------------------------
    bool CheckForValidity(void) const
    {
        unsigned int        i                           =   0;

        double              MinimumOrderOfMagnitude     =   0.0
                        ,   MaximumOrderOfMagnitude     =   0.0;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if ((this->SelectionVector->VecData)[i])
            {
                if (    ((this->MaxAccelerationVector->VecData)[i]  >=  (this->MaxJerkVector->VecData)                  [i] )
                    &&  ((this->MaxAccelerationVector->VecData)[i]  >=  fabs((this->TargetVelocityVector->VecData)      [i]))
                    &&  ((this->MaxAccelerationVector->VecData)[i]  >=  fabs((this->CurrentPositionVector->VecData)     [i]))
                    &&  ((this->MaxAccelerationVector->VecData)[i]  >=  fabs((this->CurrentVelocityVector->VecData)     [i]))
                    &&  ((this->MaxAccelerationVector->VecData)[i]  >=  fabs((this->CurrentAccelerationVector->VecData) [i])))
                {
                    MaximumOrderOfMagnitude =   (this->MaxAccelerationVector->VecData)[i];
                }
                else
                {
                    if (    ((this->MaxJerkVector->VecData)[i]  >=  fabs((this->TargetVelocityVector->VecData)      [i]))
                        &&  ((this->MaxJerkVector->VecData)[i]  >=  fabs((this->CurrentPositionVector->VecData)     [i]))
                        &&  ((this->MaxJerkVector->VecData)[i]  >=  fabs((this->CurrentVelocityVector->VecData)     [i]))
                        &&  ((this->MaxJerkVector->VecData)[i]  >=  fabs((this->CurrentAccelerationVector->VecData) [i])))
                    {
                        MaximumOrderOfMagnitude =   (this->MaxJerkVector->VecData)[i];
                    }
                    else
                    {
                        if (    (fabs((this->TargetVelocityVector->VecData)[i]) >=  fabs((this->CurrentPositionVector->VecData)     [i]))
                            &&  (fabs((this->TargetVelocityVector->VecData)[i]) >=  fabs((this->CurrentVelocityVector->VecData)     [i]))
                            &&  (fabs((this->TargetVelocityVector->VecData)[i]) >=  fabs((this->CurrentAccelerationVector->VecData) [i])))
                        {
                            MaximumOrderOfMagnitude =   fabs((this->TargetVelocityVector->VecData)[i]);
                        }
                        else
                        {
                            if (    (fabs((this->CurrentPositionVector->VecData)[i])    >=  fabs((this->CurrentVelocityVector->VecData)     [i]))
                                &&  (fabs((this->CurrentPositionVector->VecData)[i])    >=  fabs((this->CurrentAccelerationVector->VecData) [i])))
                            {
                                MaximumOrderOfMagnitude =   fabs((this->CurrentPositionVector->VecData)[i]);
                            }
                            else
                            {
                                MaximumOrderOfMagnitude =   fabs((this->CurrentAccelerationVector->VecData)[i]);
                            }
                        }
                    }
                }

                if ((this->MaxAccelerationVector->VecData)[i]   <=  (this->MaxJerkVector->VecData)[i])
                {
                    MinimumOrderOfMagnitude =   (this->MaxAccelerationVector->VecData)[i];
                }
                else
                {
                    MinimumOrderOfMagnitude =   (this->MaxJerkVector->VecData)[i];
                }

                // The value of MinimumOrderOfMagnitude is greater than
                // zero:
                if (    (MaximumOrderOfMagnitude / MinimumOrderOfMagnitude)
                    >   (double)pow((float)10, (int)(RMLVelocityInputParameters::MAXIMUM_MAGNITUDE_RANGE)))
                {
                    return(false);
                }
            }
        }

        if (this->MinimumSynchronizationTime > 1e10)
        {
            return(false);
        }

        return(true);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn void Echo(FILE* FileHandler = stdout) const
//!
//! \brief
//! \copybrief RMLInputParameters::Echo()
//!
//! \details
//! \copydetails RMLInputParameters::Echo()
//  ----------------------------------------------------------
    void Echo(FILE* FileHandler = stdout) const
    {
        RMLInputParameters::Echo(FileHandler);
        return;
    }

protected:


    enum
    {
//  ---------------------- Doxygen info ----------------------
//! \brief
//! Specifies the maximum allowed range for the orders of magnitude of
//! the input values.
//  ----------------------------------------------------------
        MAXIMUM_MAGNITUDE_RANGE =   10
    };



};// class RMLVelocityInputParameters



#endif


