//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLSetupModifiedSelectionVector.cpp
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
#include <RMLPositionInputParameters.h>
#include <RMLVector.h>


//****************************************************************************
// SetupModifiedSelectionVector()

void TypeIIRMLPosition::SetupModifiedSelectionVector(void)
{
    unsigned int                i                               =   0;

    *(this->ModifiedSelectionVector)    =   *(this->CurrentInputParameters->SelectionVector);

    for (i = 0; i < this->NumberOfDOFs; i++)
    {
        if ((this->CurrentInputParameters->SelectionVector->VecData)[i])
        {
            if  (   ((this->CurrentInputParameters->TargetVelocityVector->VecData)[i] == 0.0)
                &&  ((this->MinimumExecutionTimes->VecData)[i] <= this->CycleTime))
            {
                (this->ModifiedSelectionVector->VecData)[i] =   false;

                // This degree of freedom has already reached its target state of motion, which
                // is steady, that is, it consists of a zero velocity and zero acceleration value
                // right in the target position. To save CPU time (irrelevant for real-time capability)
                // and to enable a simple and correct calculation of phase-synchronous trajectories
                // the selection vector is modified, and the current state of motion is set to the
                // target state of motion.

                (this->CurrentInputParameters->CurrentPositionVector->VecData)          [i]
                    =   (this->CurrentInputParameters->TargetPositionVector->VecData)   [i];
                (this->CurrentInputParameters->CurrentVelocityVector->VecData)          [i] =   0.0;
                (this->CurrentInputParameters->CurrentAccelerationVector->VecData)      [i] =   0.0;
            }
        }
    }
}
