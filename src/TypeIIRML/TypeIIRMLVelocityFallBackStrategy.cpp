//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLVelocityFallBackStrategy.cpp
//!
//! \brief
//! Implementation file for the Type II On-Line Trajectory
//! Generation algorithm
//!
//! \details
//! For further information, please refer to the file TypeIIRMLVelocity.h.
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


#include <TypeIIRMLVelocity.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>


using namespace TypeIIRMLMath;


//****************************************************************************
// FallBackStrategy()

void TypeIIRMLVelocity::FallBackStrategy(       const RMLVelocityInputParameters    &InputValues
                                            ,   RMLVelocityOutputParameters         *OutputValues   )
{
    unsigned int        i   =   0;

    for (i = 0; i < this->NumberOfDOFs; i++)
    {
        if (InputValues.SelectionVector->VecData[i])
        {
            (OutputValues->NewPositionVector->VecData)[i]
                =   InputValues.CurrentPositionVector->VecData[i]
                    +   this->CycleTime
                    *   (InputValues.CurrentVelocityVector->VecData)[i];

            (OutputValues->NewVelocityVector->VecData)[i]
                =   InputValues.CurrentVelocityVector->VecData[i];
        }
        else
        {
            (OutputValues->NewPositionVector->VecData)[i]
                =   InputValues.CurrentPositionVector->VecData[i];

            (OutputValues->NewVelocityVector->VecData)[i]
                =   InputValues.CurrentVelocityVector->VecData[i];
        }

        OutputValues->ExecutionTimes->VecData[i]    =   0.0;

        OutputValues->PositionValuesAtTargetVelocity->VecData[i]
                =   InputValues.CurrentPositionVector->VecData[i];
    }

    this->SetPositionalExtremsToZero(OutputValues);

    OutputValues->TrajectoryIsPhaseSynchronized     =   false;
    OutputValues->SynchronizationTime               =   0.0;
    OutputValues->DOFWithTheGreatestExecutionTime   =   0;

    return;
}
