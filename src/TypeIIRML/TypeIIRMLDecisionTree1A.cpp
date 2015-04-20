//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLDecisionTree1A.cpp
//!
//! \brief
//! Implementation file for the Step 1 decision tree 1A of the Type II
//! On-Line Trajectory Generation algorithm
//!
//! \details
//! For further information, please refer to the file
//! TypeIIRMLDecisionTree1A.h
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


#include <TypeIIRMLDecisionTree1A.h>
#include <TypeIIRMLStep1Profiles.h>
#include <TypeIIRMLMath.h>
#include <TypeIIRMLDecisions.h>
#include <TypeIIRMLStep1IntermediateProfiles.h>
#include <TypeIIRMLStep1Profiles.h>


//************************************************************************************
// TypeIIRMLDecisionTree1A()

void TypeIIRMLMath::TypeIIRMLDecisionTree1A(    const double    &CurrentPosition
                                            ,   const double    &CurrentVelocity
                                            ,   const double    &TargetPosition
                                            ,   const double    &TargetVelocity
                                            ,   const double    &MaxVelocity
                                            ,   const double    &MaxAcceleration
                                            ,   Step1_Profile   *AppliedProfile
                                            ,   double          *MinimalExecutionTime   )
{
    bool            IntermediateInversion   =   false;

    double          ThisCurrentPosition     =   CurrentPosition
                ,   ThisCurrentVelocity     =   CurrentVelocity
                ,   ThisTargetPosition      =   TargetPosition
                ,   ThisTargetVelocity      =   TargetVelocity      ;

    *MinimalExecutionTime   =   0.0;

    // ********************************************************************
    if (Decision_1A__001(ThisCurrentVelocity))
    {
        goto MDecision_1A__002;
    }
    else
    {
        NegateStep1(        &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   &ThisTargetPosition
                        ,   &ThisTargetVelocity );

        goto MDecision_1A__002;
    }
    // ********************************************************************
MDecision_1A__002:
    if (Decision_1A__002(       ThisCurrentVelocity
                            ,   MaxVelocity         ))
    {
        goto MDecision_1A__003;
    }
    else
    {
        VToVMaxStep1(       MinimalExecutionTime
                        ,   &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   MaxVelocity
                        ,   MaxAcceleration     );

        goto MDecision_1A__003;
    }
    // ********************************************************************
MDecision_1A__003:
    if (Decision_1A__003(       ThisCurrentVelocity
                            ,   ThisTargetVelocity  ))
    {
        goto MDecision_1A__004;
    }
    else
    {
        goto MDecision_1A__006;
    }
    // ********************************************************************
MDecision_1A__004:
    if (Decision_1A__004(       ThisCurrentPosition
                            ,   ThisCurrentVelocity
                            ,   ThisTargetPosition
                            ,   ThisTargetVelocity
                            ,   MaxAcceleration         ))
    {
        goto MDecision_1A__005;
    }
    else
    {
        VToZeroStep1(       MinimalExecutionTime
                        ,   &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   MaxAcceleration     );

        NegateStep1(        &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   &ThisTargetPosition
                        ,   &ThisTargetVelocity );

        IntermediateInversion   =   true;
        goto MDecision_1A__009;
    }
    // ********************************************************************
MDecision_1A__005:
    if (Decision_1A__005(       ThisCurrentPosition
                            ,   ThisCurrentVelocity
                            ,   ThisTargetPosition
                            ,   ThisTargetVelocity
                            ,   MaxVelocity
                            ,   MaxAcceleration         ))
    {
        *MinimalExecutionTime   +=  ProfileStep1PosLinHldNegLin(    ThisCurrentPosition
                                                                ,   ThisCurrentVelocity
                                                                ,   ThisTargetPosition
                                                                ,   ThisTargetVelocity
                                                                ,   MaxVelocity
                                                                ,   MaxAcceleration         );
        if (IntermediateInversion)
        {
            *AppliedProfile =   Step1_Profile_NegLinHldPosLin;
        }
        else
        {
            *AppliedProfile =   Step1_Profile_PosLinHldNegLin;
        }

        goto END_OF_THIS_FUNCTION;
    }
    else
    {
        *MinimalExecutionTime   +=  ProfileStep1PosLinNegLin(       ThisCurrentPosition
                                                                ,   ThisCurrentVelocity
                                                                ,   ThisTargetPosition
                                                                ,   ThisTargetVelocity
                                                                ,   MaxAcceleration         );
        if (IntermediateInversion)
        {
            *AppliedProfile =   Step1_Profile_NegLinPosLin;
        }
        else
        {
            *AppliedProfile =   Step1_Profile_PosLinNegLin;
        }

        goto END_OF_THIS_FUNCTION;
    }
    // ********************************************************************
MDecision_1A__006:
    if (Decision_1A__006(ThisTargetVelocity))
    {
        goto MDecision_1A__007;
    }
    else
    {
        goto MDecision_1A__008;
    }
    // ********************************************************************
MDecision_1A__007:
    if (Decision_1A__007(       ThisCurrentPosition
                            ,   ThisCurrentVelocity
                            ,   ThisTargetPosition
                            ,   ThisTargetVelocity
                            ,   MaxAcceleration         ))
    {
        VToZeroStep1(       MinimalExecutionTime
                        ,   &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   MaxAcceleration     );

        NegateStep1(        &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   &ThisTargetPosition
                        ,   &ThisTargetVelocity );

        IntermediateInversion   =   true;
        goto MDecision_1A__009;
    }
    else
    {
        goto MDecision_1A__005;
    }
    // ********************************************************************
MDecision_1A__008:
    if (Decision_1A__008(       ThisCurrentPosition
                            ,   ThisCurrentVelocity
                            ,   ThisTargetPosition
                            ,   ThisTargetVelocity
                            ,   MaxAcceleration         ))
    {
        VToZeroStep1(       MinimalExecutionTime
                        ,   &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   MaxAcceleration     );

        NegateStep1(        &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   &ThisTargetPosition
                        ,   &ThisTargetVelocity );

        IntermediateInversion   =   true;
        goto MDecision_1A__005;
    }
    else
    {
        goto MDecision_1A__009;
    }
    // ********************************************************************
MDecision_1A__009:
    if (Decision_1A__009(       ThisCurrentPosition
                            ,   ThisCurrentVelocity
                            ,   ThisTargetPosition
                            ,   ThisTargetVelocity
                            ,   MaxVelocity
                            ,   MaxAcceleration         ))
    {
        *MinimalExecutionTime   +=  ProfileStep1PosTriNegLin(       ThisCurrentPosition
                                                                ,   ThisCurrentVelocity
                                                                ,   ThisTargetPosition
                                                                ,   ThisTargetVelocity
                                                                ,   MaxAcceleration         );
        if (IntermediateInversion)
        {
            *AppliedProfile =   Step1_Profile_NegTriPosLin;
        }
        else
        {
            *AppliedProfile =   Step1_Profile_PosTriNegLin;
        }

        goto END_OF_THIS_FUNCTION;
    }
    else
    {
        *MinimalExecutionTime   +=  ProfileStep1PosTrapNegLin(      ThisCurrentPosition
                                                                ,   ThisCurrentVelocity
                                                                ,   ThisTargetPosition
                                                                ,   ThisTargetVelocity
                                                                ,   MaxVelocity
                                                                ,   MaxAcceleration         );
        if (IntermediateInversion)
        {
            *AppliedProfile =   Step1_Profile_NegTrapPosLin;
        }
        else
        {
            *AppliedProfile =   Step1_Profile_PosTrapNegLin;
        }

        goto END_OF_THIS_FUNCTION;
    }
    // ********************************************************************
END_OF_THIS_FUNCTION:

    return;
}
