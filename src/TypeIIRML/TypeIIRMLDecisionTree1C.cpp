//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLDecisionTree1C.cpp
//!
//! \brief
//! Implementation file for the Step 1 decision tree 1C of the Type II
//! On-Line Trajectory Generation algorithm
//!
//! \details
//! For further information, please refer to the file
//! TypeIIRMLDecisionTree1C.h
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


#include <TypeIIRMLDecisionTree1C.h>
#include <TypeIIRMLStep1Profiles.h>
#include <TypeIIRMLMath.h>
#include <TypeIIRMLDecisions.h>
#include <TypeIIRMLStep1IntermediateProfiles.h>
#include <TypeIIRMLStep1Profiles.h>


//************************************************************************************
// TypeIIRMLDecisionTree1C()

void TypeIIRMLMath::TypeIIRMLDecisionTree1C(    const double    &CurrentPosition
                                            ,   const double    &CurrentVelocity
                                            ,   const double    &TargetPosition
                                            ,   const double    &TargetVelocity
                                            ,   const double    &MaxVelocity
                                            ,   const double    &MaxAcceleration
                                            ,   double          *AlternativeExecutionTime)
{
    double          ThisCurrentPosition     =   CurrentPosition
                ,   ThisCurrentVelocity     =   CurrentVelocity
                ,   ThisTargetPosition      =   TargetPosition
                ,   ThisTargetVelocity      =   TargetVelocity      ;

    *AlternativeExecutionTime   =   0.0;

    // ********************************************************************
    if (Decision_1C__001(ThisCurrentVelocity))
    {
        goto MDecision_1C__002;
    }
    else
    {
        NegateStep1(        &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   &ThisTargetPosition
                        ,   &ThisTargetVelocity );

        goto MDecision_1C__002;
    }
    // ********************************************************************
MDecision_1C__002:
    if (Decision_1C__002(       ThisCurrentVelocity
                            ,   MaxVelocity         ))
    {
        goto MDecision_1C__003;
    }
    else
    {
        VToVMaxStep1(       AlternativeExecutionTime
                        ,   &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   MaxVelocity
                        ,   MaxAcceleration     );

        goto MDecision_1C__003;
    }
    // ********************************************************************
MDecision_1C__003:

    VToZeroStep1(       AlternativeExecutionTime
                    ,   &ThisCurrentPosition
                    ,   &ThisCurrentVelocity
                    ,   MaxAcceleration         );

    NegateStep1(        &ThisCurrentPosition
                    ,   &ThisCurrentVelocity
                    ,   &ThisTargetPosition
                    ,   &ThisTargetVelocity );


    if (Decision_1C__003(       ThisCurrentPosition
                            ,   ThisCurrentVelocity
                            ,   ThisTargetPosition
                            ,   ThisTargetVelocity
                            ,   MaxVelocity
                            ,   MaxAcceleration         ))
    {
        *AlternativeExecutionTime   +=  ProfileStep1PosTriNegLin(       ThisCurrentPosition
                                                                    ,   ThisCurrentVelocity
                                                                    ,   ThisTargetPosition
                                                                    ,   ThisTargetVelocity
                                                                    ,   MaxAcceleration         );
    }
    else
    {
        *AlternativeExecutionTime   +=  ProfileStep1PosTrapNegLin(      ThisCurrentPosition
                                                                    ,   ThisCurrentVelocity
                                                                    ,   ThisTargetPosition
                                                                    ,   ThisTargetVelocity
                                                                    ,   MaxVelocity
                                                                    ,   MaxAcceleration         );
    }
    // ********************************************************************

    return;
}
