//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLStep2WithoutSynchronization.cpp
//!
//! \brief
//! Implementation file for the function
//! TypeIIRMLMath::Step2WithoutSynchronization()
//!
//! \details
//! For further information, please refer to the file
//! TypeIIRMLStep2WithoutSynchronization.h.
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


#include <TypeIIRMLStep2WithoutSynchronization.h>
#include <TypeIIRMLStep1Profiles.h>
#include <TypeIIRMLDecisionTree2.h>
#include <TypeIIRMLMath.h>
#include <TypeIIRMLDecisions.h>
#include <TypeIIRMLStep2Profiles.h>
#include <TypeIIRMLStep2IntermediateProfiles.h>


//****************************************************************************
// Step2WithoutSynchronization()

void TypeIIRMLMath::Step2WithoutSynchronization(        const double                        &CurrentPosition
                                                    ,   const double                        &CurrentVelocity
                                                    ,   const double                        &TargetPosition
                                                    ,   const double                        &TargetVelocity
                                                    ,   const double                        &MaxVelocity
                                                    ,   const double                        &MaxAcceleration
                                                    ,   const TypeIIRMLMath::Step1_Profile  &UsedProfile
                                                    ,   const double                        &MinimumExecutionTime
                                                    ,   MotionPolynomials                   *PolynomialsInternal)
{
    bool            Inverted                =   false;

    double          CurrentTime             =   0.0
                ,   ThisCurrentPosition     =   CurrentPosition
                ,   ThisCurrentVelocity     =   CurrentVelocity
                ,   ThisTargetPosition      =   TargetPosition
                ,   ThisTargetVelocity      =   TargetVelocity      ;

    if (!Decision_2___001(ThisCurrentVelocity))
    {
        NegateStep2(        &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   &ThisTargetPosition
                        ,   &ThisTargetVelocity
                        ,   &Inverted               );
    }

    if (!Decision_2___002(      ThisCurrentVelocity
                            ,   MaxVelocity         ))
    {
        VToVMaxStep2(       &CurrentTime
                        ,   &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   MaxVelocity
                        ,   MaxAcceleration
                        ,   &(*PolynomialsInternal)
                        ,   Inverted                );
    }

    if (    (UsedProfile == Step1_Profile_NegLinHldPosLin   )
        ||  (UsedProfile == Step1_Profile_NegLinPosLin      )
        ||  (UsedProfile == Step1_Profile_NegTrapPosLin     )
        ||  (UsedProfile == Step1_Profile_NegTriPosLin      )   )
    {
        VToZeroStep2(       &CurrentTime
                        ,   &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   MaxAcceleration
                        ,   &(*PolynomialsInternal)
                        ,   Inverted                );

        NegateStep2(        &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   &ThisTargetPosition
                        ,   &ThisTargetVelocity
                        ,   &Inverted               );
    }

    if (    (UsedProfile == Step1_Profile_PosLinHldNegLin   )
        ||  (UsedProfile == Step1_Profile_NegLinHldPosLin   )
        ||  (UsedProfile == Step1_Profile_PosLinNegLin      )
        ||  (UsedProfile == Step1_Profile_NegLinPosLin      )   )
    {
        ProfileStep2PosLinHldNegLin(    CurrentTime
                                    ,   MinimumExecutionTime
                                    ,   ThisCurrentPosition
                                    ,   ThisCurrentVelocity
                                    ,   ThisTargetPosition
                                    ,   ThisTargetVelocity
                                    ,   MaxAcceleration
                                    ,   &(*PolynomialsInternal)
                                    ,   Inverted                );
    }
    else
    {
        ProfileStep2PosTrapNegLin(      CurrentTime
                                    ,   MinimumExecutionTime
                                    ,   ThisCurrentPosition
                                    ,   ThisCurrentVelocity
                                    ,   ThisTargetPosition
                                    ,   ThisTargetVelocity
                                    ,   MaxAcceleration
                                    ,   &(*PolynomialsInternal)
                                    ,   Inverted                );
    }

    return;
}

