//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLDecisionTree1B.cpp
//!
//! \brief
//! Implementation file for the Step 1 decision tree 1B of the Type II
//! On-Line Trajectory Generation algorithm
//!
//! \details
//! For further information, please refer to the file
//! TypeIIRMLDecisionTree1B.h
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


#include <TypeIIRMLDecisionTree1B.h>
#include <TypeIIRMLMath.h>
#include <TypeIIRMLDecisions.h>
#include <TypeIIRMLStep1IntermediateProfiles.h>
#include <TypeIIRMLStep1Profiles.h>


//************************************************************************************
// TypeIIRMLDecisionTree1B()

void TypeIIRMLMath::TypeIIRMLDecisionTree1B(    const double    &CurrentPosition
                                            ,   const double    &CurrentVelocity
                                            ,   const double    &TargetPosition
                                            ,   const double    &TargetVelocity
                                            ,   const double    &MaxVelocity
                                            ,   const double    &MaxAcceleration
                                            ,   double          *MaximalExecutionTime)
{
    double          ThisCurrentPosition     =   CurrentPosition
                ,   ThisCurrentVelocity     =   CurrentVelocity
                ,   ThisTargetPosition      =   TargetPosition
                ,   ThisTargetVelocity      =   TargetVelocity      ;

    *MaximalExecutionTime   =   0.0;

    // ********************************************************************
    if (Decision_1B__001(ThisCurrentVelocity))
    {
        goto MDecision_1B__002;
    }
    else
    {
        NegateStep1(        &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   &ThisTargetPosition
                        ,   &ThisTargetVelocity );

        goto MDecision_1B__002;
    }
    // ********************************************************************
MDecision_1B__002:
    if (Decision_1B__002(       ThisCurrentVelocity
                            ,   MaxVelocity         ))
    {
        goto MDecision_1B__003;
    }
    else
    {
        VToVMaxStep1(       MaximalExecutionTime
                        ,   &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   MaxVelocity
                        ,   MaxAcceleration     );

        goto MDecision_1B__003;
    }
    // ********************************************************************
MDecision_1B__003:
    if (Decision_1B__003(ThisTargetVelocity))
    {
        goto MDecision_1B__004;
    }
    else
    {
        *MaximalExecutionTime   =   RML_INFINITY;
        goto END_OF_THIS_FUNCTION;
    }
    // ********************************************************************
MDecision_1B__004:
    if (Decision_1B__004(       ThisCurrentVelocity
                            ,   ThisTargetVelocity      ))
    {
        goto MDecision_1B__005;
    }
    else
    {
        goto MDecision_1B__007;
    }
    // ********************************************************************
MDecision_1B__005:
    if (Decision_1B__005(       ThisCurrentPosition
                            ,   ThisCurrentVelocity
                            ,   ThisTargetPosition
                            ,   ThisTargetVelocity
                            ,   MaxAcceleration         ))
    {
        goto MDecision_1B__006;
    }
    else
    {
        *MaximalExecutionTime   =   RML_INFINITY;
        goto END_OF_THIS_FUNCTION;
    }
    // ********************************************************************
MDecision_1B__006:
    if (Decision_1B__006(       ThisCurrentPosition
                            ,   ThisCurrentVelocity
                            ,   ThisTargetPosition
                            ,   ThisTargetVelocity
                            ,   MaxAcceleration         ))
    {
        *MaximalExecutionTime   =   RML_INFINITY;
        goto END_OF_THIS_FUNCTION;
    }
    else
    {
        *MaximalExecutionTime   +=  ProfileStep1NegLinPosLin(       ThisCurrentPosition
                                                                ,   ThisCurrentVelocity
                                                                ,   ThisTargetPosition
                                                                ,   ThisTargetVelocity
                                                                ,   MaxAcceleration         );
        goto END_OF_THIS_FUNCTION;
    }
    // ********************************************************************
MDecision_1B__007:
    if (Decision_1B__007(       ThisCurrentPosition
                            ,   ThisCurrentVelocity
                            ,   ThisTargetPosition
                            ,   ThisTargetVelocity
                            ,   MaxAcceleration         ))
    {
        *MaximalExecutionTime   =   RML_INFINITY;
        goto END_OF_THIS_FUNCTION;
    }
    else
    {
        goto MDecision_1B__006;
    }
    // ********************************************************************
END_OF_THIS_FUNCTION:

    return;
}
