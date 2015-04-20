//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLStep3.cpp
//!
//! \brief
//! Implementation file for the Type II On-Line Trajectory
//! Generation algorithm
//!
//! \details
//! For further information, please refer to the file TypeIIRMLPosition.h.
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


#include <TypeIIRMLPosition.h>
#include <TypeIIRMLMath.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <ReflexxesAPI.h>


//*******************************************************************************************
// Step3

int TypeIIRMLPosition::Step3(       const double                    &TimeValueInSeconds
                                ,   RMLPositionOutputParameters     *OP                 ) const
{
    unsigned int            i                       =   0;

    int                     j                       =   0
                        ,   ReturnValueForThisMethod = ReflexxesAPI::RML_FINAL_STATE_REACHED;

    for (i = 0; i < this->NumberOfDOFs; i++)
    {
        if ( (this->ModifiedSelectionVector->VecData)[i] )
        {
            j = 0;

            while ( (TimeValueInSeconds > (this->Polynomials)[i].PolynomialTimes[j]) && (j < MAXIMAL_NO_OF_POLYNOMIALS - 1))
            {
                j++;
            }

            (OP->NewPositionVector->VecData)    [i]
                =   (this->Polynomials)[i].PositionPolynomial[j].CalculateValue(TimeValueInSeconds);
            (OP->NewVelocityVector->VecData)    [i]
                =   (this->Polynomials)[i].VelocityPolynomial[j].CalculateValue(TimeValueInSeconds);
            (OP->NewAccelerationVector->VecData)[i]
                =   (this->Polynomials)[i].AccelerationPolynomial[j].CalculateValue(TimeValueInSeconds);

            if ( j < ((this->Polynomials)[i].ValidPolynomials) - 1)
            {
                ReturnValueForThisMethod = ReflexxesAPI::RML_WORKING;
            }
        }
        else
        {
            (OP->NewPositionVector->VecData)    [i]
                =   (this->CurrentInputParameters->CurrentPositionVector->VecData)[i];
            (OP->NewVelocityVector->VecData)    [i]
                =   (this->CurrentInputParameters->CurrentVelocityVector->VecData)[i];
            (OP->NewAccelerationVector->VecData)[i]
                =   (this->CurrentInputParameters->CurrentAccelerationVector->VecData)[i];
        }
    }

    return(ReturnValueForThisMethod);
}
