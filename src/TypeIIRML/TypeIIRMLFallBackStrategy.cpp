//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLFallBackStrategy.cpp
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
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <RMLVector.h>
#include <RMLVelocityFlags.h>
#include <TypeIIRMLVelocity.h>


//*******************************************************************************************
// FallBackStrategy()

void TypeIIRMLPosition::FallBackStrategy(       const RMLPositionInputParameters    &InputValues
                                            ,   RMLPositionOutputParameters         *OutputValues
                                            ,   const RMLPositionFlags              &InputsFlags)
{
    unsigned int        i   =   0;

    *(this->VelocityInputParameters->SelectionVector)
        =   *(InputValues.SelectionVector);
    *(this->VelocityInputParameters->CurrentPositionVector)
        =   *(InputValues.CurrentPositionVector);
    *(this->VelocityInputParameters->CurrentVelocityVector)
        =   *(InputValues.CurrentVelocityVector);
    *(this->VelocityInputParameters->CurrentAccelerationVector)
        =   *(InputValues.CurrentAccelerationVector);
    *(this->VelocityInputParameters->MaxAccelerationVector)
        =   *(InputValues.MaxAccelerationVector);
    *(this->VelocityInputParameters->MaxJerkVector)
        =   *(InputValues.MaxJerkVector);

    if (InputsFlags.KeepCurrentVelocityInCaseOfFallbackStrategy)
    {
        *(this->VelocityInputParameters->TargetVelocityVector)
            =   *(InputValues.CurrentVelocityVector);
    }
    else
    {
        *(this->VelocityInputParameters->TargetVelocityVector)
            =   *(InputValues.AlternativeTargetVelocityVector);
    }

    if (InputsFlags.SynchronizationBehavior ==  RMLFlags::ONLY_PHASE_SYNCHRONIZATION)
    {
        this->VelocityFlags.SynchronizationBehavior =   RMLFlags::ONLY_PHASE_SYNCHRONIZATION;
    }
    else
    {
        this->VelocityFlags.SynchronizationBehavior =   RMLFlags::NO_SYNCHRONIZATION;
    }

    this->RMLVelocityObject->GetNextStateOfMotion(      *(this->VelocityInputParameters)
                                                    ,   this->VelocityOutputParameters
                                                    ,   this->VelocityFlags);

    *(OutputValues->NewPositionVector)
        =   *(this->VelocityOutputParameters->NewPositionVector);
    *(OutputValues->NewVelocityVector)
        =   *(this->VelocityOutputParameters->NewVelocityVector);
    *(OutputValues->NewAccelerationVector)
        =   *(this->VelocityOutputParameters->NewAccelerationVector);

    OutputValues->SynchronizationTime
        =   this->VelocityOutputParameters->GetGreatestExecutionTime();

    OutputValues->TrajectoryIsPhaseSynchronized     =   false;

    OutputValues->ANewCalculationWasPerformed       =   true;

    *(OutputValues->MinPosExtremaPositionVectorOnly)
        =   *(this->VelocityOutputParameters->MinPosExtremaPositionVectorOnly);
    *(OutputValues->MaxPosExtremaPositionVectorOnly)
        =   *(this->VelocityOutputParameters->MaxPosExtremaPositionVectorOnly);

    *(OutputValues->MinExtremaTimesVector)
        =   *(this->VelocityOutputParameters->MinExtremaTimesVector);
    *(OutputValues->MaxExtremaTimesVector)
        =   *(this->VelocityOutputParameters->MaxExtremaTimesVector);

    for (i = 0; i < this->NumberOfDOFs; i++)
    {
        *((OutputValues->MinPosExtremaPositionVectorArray)[i])
            =   *((this->VelocityOutputParameters->MinPosExtremaPositionVectorArray)[i]);
        *((OutputValues->MinPosExtremaVelocityVectorArray)[i])
            =   *((this->VelocityOutputParameters->MinPosExtremaVelocityVectorArray)[i]);
        *((OutputValues->MinPosExtremaAccelerationVectorArray)[i])
            =   *((this->VelocityOutputParameters->MinPosExtremaAccelerationVectorArray)[i]);

        *((OutputValues->MaxPosExtremaPositionVectorArray)[i])
            =   *((this->VelocityOutputParameters->MaxPosExtremaPositionVectorArray)[i]);
        *((OutputValues->MaxPosExtremaVelocityVectorArray)[i])
            =   *((this->VelocityOutputParameters->MaxPosExtremaVelocityVectorArray)[i]);
        *((OutputValues->MaxPosExtremaAccelerationVectorArray)[i])
            =   *((this->VelocityOutputParameters->MaxPosExtremaAccelerationVectorArray)[i]);
    }
}
