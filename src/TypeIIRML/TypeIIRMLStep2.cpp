//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLStep2.cpp
//!
//! \brief
//! Implementation file for the method TypeIIRMLPosition::Step2()
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
#include <TypeIIRMLDecisionTree2.h>
#include <TypeIIRMLStep2WithoutSynchronization.h>
#include <RMLPositionInputParameters.h>
#include <ReflexxesAPI.h>


using namespace TypeIIRMLMath;


//*******************************************************************************************
// Step2

void TypeIIRMLPosition::Step2(void)
{
    unsigned int        i                   =   0;

    if (this->CurrentTrajectoryIsPhaseSynchronized)
    {
        // As we only calculate the trajectory for one DOF, we do not use multiple threads for the
        // Step 2 calculation of the trajectory.

        this->Step2PhaseSynchronization();
    }
    else
    {
        if (this->CurrentTrajectoryIsNotSynchronized)
        {
            for (i = 0; i < this->NumberOfDOFs; i++)
            {
                if((this->ModifiedSelectionVector->VecData)[i])
                {
                    Step2WithoutSynchronization(        (this->CurrentInputParameters->CurrentPositionVector->VecData)      [i]
                                                    ,   (this->CurrentInputParameters->CurrentVelocityVector->VecData)      [i]
                                                    ,   (this->CurrentInputParameters->TargetPositionVector->VecData)       [i]
                                                    ,   (this->CurrentInputParameters->TargetVelocityVector->VecData)       [i]
                                                    ,   (this->CurrentInputParameters->MaxVelocityVector->VecData)          [i]
                                                    ,   (this->CurrentInputParameters->MaxAccelerationVector->VecData)      [i]
                                                    ,   (this->UsedStep1AProfiles->VecData)                                 [i]
                                                    ,   (this->MinimumExecutionTimes->VecData)                              [i]
                                                    ,   &((this->Polynomials)[i])   );
                }
            }
        }
        else
        {
            for (i = 0; i < this->NumberOfDOFs; i++)
            {
                if((this->ModifiedSelectionVector->VecData)[i])
                {
                    TypeIIRMLDecisionTree2(     (this->CurrentInputParameters->CurrentPositionVector->VecData)      [i]
                                            ,   (this->CurrentInputParameters->CurrentVelocityVector->VecData)      [i]
                                            ,   (this->CurrentInputParameters->TargetPositionVector->VecData)       [i]
                                            ,   (this->CurrentInputParameters->TargetVelocityVector->VecData)       [i]
                                            ,   (this->CurrentInputParameters->MaxVelocityVector->VecData)          [i]
                                            ,   (this->CurrentInputParameters->MaxAccelerationVector->VecData)      [i]
                                            ,   this->SynchronizationTime
                                            ,   &((this->Polynomials)[i])   );
                }
            }
        }
    }

    return;
}
