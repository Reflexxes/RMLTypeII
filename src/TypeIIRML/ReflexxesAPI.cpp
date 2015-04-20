//  ---------------------- Doxygen info ----------------------
//! \file ReflexxesAPI.cpp
//!
//! \brief
//! Implementation file for the user interface (API)
//!
//! \details
//! Implementation file for all methods of the class ReflexxesAPI, which
//! constitutes the user API for On-Line Trajectory Generation
//! algorithms.
//! For further information, please refer to the file ReflexxesAPI.h.
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


#include <ReflexxesAPI.h>
#include <TypeIIRMLPosition.h>
#include <TypeIIRMLVelocity.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <RMLPositionFlags.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLVelocityFlags.h>


//****************************************************************************
// ReflexxesAPI()

ReflexxesAPI::ReflexxesAPI(     const unsigned int  &DegreesOfFreedom
                           ,    const double        &CycleTimeInSeconds
                           ,    const unsigned int  &NumberOfAdditionalThreads)
{
    this->NumberOfDOFs          =   DegreesOfFreedom            ;
    this->NumberOfOwnThreads    =   NumberOfAdditionalThreads   ;
    this->CycleTime             =   CycleTimeInSeconds          ;

    this->RMLPositionObject     =   (void*) new TypeIIRMLPosition(      DegreesOfFreedom
                                                                    ,   CycleTimeInSeconds);

    this->RMLVelocityObject     =   (void*) new TypeIIRMLVelocity(      DegreesOfFreedom
                                                                    ,   CycleTimeInSeconds);
}


//****************************************************************************
// ~ReflexxesAPI()

ReflexxesAPI::~ReflexxesAPI(void)
{
    delete  (TypeIIRMLVelocity*)this->RMLVelocityObject;
    delete  (TypeIIRMLPosition*)this->RMLPositionObject;

    this->RMLVelocityObject =   NULL;
    this->RMLPositionObject =   NULL;
}


//****************************************************************************
// RMLPosition()

int ReflexxesAPI::RMLPosition(      const RMLPositionInputParameters    &InputValues
                                ,   RMLPositionOutputParameters         *OutputValues
                                ,   const RMLPositionFlags              &Flags)
{
    return(((TypeIIRMLPosition*)(this->RMLPositionObject))->GetNextStateOfMotion(       InputValues
                                                                                    ,   OutputValues
                                                                                    ,   Flags           ));
}


//****************************************************************************
// RMLPositionAtAGivenSampleTime()

int ReflexxesAPI::RMLPositionAtAGivenSampleTime(    const double                        &TimeValueInSeconds
                                                ,   RMLPositionOutputParameters         *OutputValues)
{
    return(((TypeIIRMLPosition*)(this->RMLPositionObject))->GetNextStateOfMotionAtTime(     TimeValueInSeconds
                                                                                        ,   OutputValues        ));
}


//****************************************************************************
// RMLVelocity()

int ReflexxesAPI::RMLVelocity(      const RMLVelocityInputParameters    &InputValues
                                ,   RMLVelocityOutputParameters         *OutputValues
                                ,   const RMLVelocityFlags              &Flags)
{
    return(((TypeIIRMLVelocity*)(this->RMLVelocityObject))->GetNextStateOfMotion(       InputValues
                                                                                    ,   OutputValues
                                                                                    ,   Flags           ));
}


//****************************************************************************
// RMLVelocityAtAGivenSampleTime()

int ReflexxesAPI::RMLVelocityAtAGivenSampleTime(    const double                        &TimeValueInSeconds
                                                ,   RMLVelocityOutputParameters         *OutputValues)
{
    return(((TypeIIRMLVelocity*)(this->RMLVelocityObject))->GetNextStateOfMotionAtTime(     TimeValueInSeconds
                                                                                        ,   OutputValues        ));
}
