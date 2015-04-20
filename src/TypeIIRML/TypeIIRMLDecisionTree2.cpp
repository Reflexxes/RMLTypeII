//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLDecisionTree2.cpp
//!
//! \brief
//! Implementation file for the Step 2 decision tree of the Type II On-Line
//! Trajectory Generation algorithm (time-synchronized case)
//!
//! \details
//! For further information, please refer to the file
//! TypeIIRMLDecisionTree2.h
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


#include <TypeIIRMLDecisionTree2.h>
#include <TypeIIRMLStep2Profiles.h>
#include <TypeIIRMLMath.h>
#include <TypeIIRMLDecisions.h>
#include <TypeIIRMLStep2IntermediateProfiles.h>
#include <TypeIIRMLStep2Profiles.h>


//************************************************************************************
// TypeIIRMLDecisionTree2()

void TypeIIRMLMath::TypeIIRMLDecisionTree2(     const double        &CurrentPosition
                                            ,   const double        &CurrentVelocity
                                            ,   const double        &TargetPosition
                                            ,   const double        &TargetVelocity
                                            ,   const double        &MaxVelocity
                                            ,   const double        &MaxAcceleration
                                            ,   const double        &SynchronizationTime
                                            ,   MotionPolynomials   *PolynomialsInternal)
{
    bool            Inverted                =   false               ;

    double          CurrentTime             =   0.0
                ,   ThisCurrentPosition     =   CurrentPosition
                ,   ThisCurrentVelocity     =   CurrentVelocity
                ,   ThisTargetPosition      =   TargetPosition
                ,   ThisTargetVelocity      =   TargetVelocity      ;

    // ********************************************************************
    if (Decision_2___001(ThisCurrentVelocity))
    {
        goto MDecision_2___002;
    }
    else
    {
        NegateStep2(        &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   &ThisTargetPosition
                        ,   &ThisTargetVelocity
                        ,   &Inverted               );

        goto MDecision_2___002;
    }
    // ********************************************************************
MDecision_2___002:
    if (Decision_2___002(       ThisCurrentVelocity
                            ,   MaxVelocity         ))
    {
        goto MDecision_2___003;
    }
    else
    {
        VToVMaxStep2(       &CurrentTime
                        ,   &ThisCurrentPosition
                        ,   &ThisCurrentVelocity
                        ,   MaxVelocity
                        ,   MaxAcceleration
                        ,   &(*PolynomialsInternal)
                        ,   Inverted                );

        goto MDecision_2___003;
    }
    // ********************************************************************
MDecision_2___003:
    if (Decision_2___003(       ThisCurrentVelocity
                            ,   ThisTargetVelocity  ))
    {
        goto MDecision_2___004;
    }
    else
    {
        goto MDecision_2___007;
    }
    // ********************************************************************
MDecision_2___004:
    if (Decision_2___004(       ThisCurrentPosition
                            ,   ThisCurrentVelocity
                            ,   ThisTargetPosition
                            ,   ThisTargetVelocity
                            ,   MaxAcceleration
                            ,   CurrentTime
                            ,   SynchronizationTime     ))
    {
        ProfileStep2PosLinHldNegLin(    CurrentTime
                                    ,   SynchronizationTime
                                    ,   ThisCurrentPosition
                                    ,   ThisCurrentVelocity
                                    ,   ThisTargetPosition
                                    ,   ThisTargetVelocity
                                    ,   MaxAcceleration
                                    ,   &(*PolynomialsInternal)
                                    ,   Inverted                );
        goto END_OF_THIS_FUNCTION;
    }
    else
    {
        goto MDecision_2___005;
    }
    // ********************************************************************
MDecision_2___005:
    if (Decision_2___005(       ThisCurrentPosition
                            ,   ThisCurrentVelocity
                            ,   ThisTargetPosition
                            ,   ThisTargetVelocity
                            ,   MaxAcceleration
                            ,   CurrentTime
                            ,   SynchronizationTime     ))
    {
        ProfileStep2PosLinHldPosLin(    CurrentTime
                                    ,   SynchronizationTime
                                    ,   ThisCurrentPosition
                                    ,   ThisCurrentVelocity
                                    ,   ThisTargetPosition
                                    ,   ThisTargetVelocity
                                    ,   MaxAcceleration
                                    ,   &(*PolynomialsInternal)
                                    ,   Inverted                );
        goto END_OF_THIS_FUNCTION;
    }
    else
    {
        goto MDecision_2___006;
    }
    // ********************************************************************
MDecision_2___006:
    if (Decision_2___006(       CurrentTime
                            ,   SynchronizationTime
                            ,   ThisCurrentPosition
                            ,   ThisCurrentVelocity
                            ,   ThisTargetPosition
                            ,   ThisTargetVelocity
                            ,   MaxAcceleration         ))
    {
        ProfileStep2NegLinHldPosLin(    CurrentTime
                                    ,   SynchronizationTime
                                    ,   ThisCurrentPosition
                                    ,   ThisCurrentVelocity
                                    ,   ThisTargetPosition
                                    ,   ThisTargetVelocity
                                    ,   MaxAcceleration
                                    ,   &(*PolynomialsInternal)
                                    ,   Inverted                );
        goto END_OF_THIS_FUNCTION;
    }
    else
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

        ProfileStep2PosTrapNegLin(      CurrentTime
                                    ,   SynchronizationTime
                                    ,   ThisCurrentPosition
                                    ,   ThisCurrentVelocity
                                    ,   ThisTargetPosition
                                    ,   ThisTargetVelocity
                                    ,   MaxAcceleration
                                    ,   &(*PolynomialsInternal)
                                    ,   Inverted                );
        goto END_OF_THIS_FUNCTION;

    }
    // ********************************************************************
MDecision_2___007:
    if (Decision_2___007(ThisTargetVelocity))
    {
        goto MDecision_2___008;
    }
    else
    {
        goto MDecision_2___010;
    }
    // ********************************************************************
MDecision_2___008:
    if (Decision_2___008(       ThisCurrentPosition
                            ,   ThisCurrentVelocity
                            ,   ThisTargetPosition
                            ,   ThisTargetVelocity
                            ,   MaxAcceleration
                            ,   CurrentTime
                            ,   SynchronizationTime     ))
    {
        ProfileStep2PosLinHldNegLin(    CurrentTime
                                    ,   SynchronizationTime
                                    ,   ThisCurrentPosition
                                    ,   ThisCurrentVelocity
                                    ,   ThisTargetPosition
                                    ,   ThisTargetVelocity
                                    ,   MaxAcceleration
                                    ,   &(*PolynomialsInternal)
                                    ,   Inverted                );
        goto END_OF_THIS_FUNCTION;
    }
    else
    {
        goto MDecision_2___009;
    }
    // ********************************************************************
MDecision_2___009:
    if (Decision_2___009(       ThisCurrentPosition
                            ,   ThisCurrentVelocity
                            ,   ThisTargetPosition
                            ,   ThisTargetVelocity
                            ,   MaxAcceleration
                            ,   CurrentTime
                            ,   SynchronizationTime     ))
    {
        ProfileStep2NegLinHldNegLin(    CurrentTime
                                    ,   SynchronizationTime
                                    ,   ThisCurrentPosition
                                    ,   ThisCurrentVelocity
                                    ,   ThisTargetPosition
                                    ,   ThisTargetVelocity
                                    ,   MaxAcceleration
                                    ,   &(*PolynomialsInternal)
                                    ,   Inverted                );
        goto END_OF_THIS_FUNCTION;
    }
    else
    {
        goto MDecision_2___006;
    }
    // ********************************************************************
MDecision_2___010:
    if (Decision_2___010(       ThisCurrentPosition
                            ,   ThisCurrentVelocity
                            ,   ThisTargetPosition
                            ,   ThisTargetVelocity
                            ,   MaxAcceleration         ))
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

        goto MDecision_2___011;
    }
    else
    {
        goto MDecision_2___012;
    }
    // ********************************************************************
MDecision_2___011:
    if (Decision_2___011(       ThisCurrentPosition
                            ,   ThisCurrentVelocity
                            ,   ThisTargetPosition
                            ,   ThisTargetVelocity
                            ,   MaxAcceleration
                            ,   CurrentTime
                            ,   SynchronizationTime     ))
    {
        ProfileStep2PosLinHldNegLin(    CurrentTime
                                    ,   SynchronizationTime
                                    ,   ThisCurrentPosition
                                    ,   ThisCurrentVelocity
                                    ,   ThisTargetPosition
                                    ,   ThisTargetVelocity
                                    ,   MaxAcceleration
                                    ,   &(*PolynomialsInternal)
                                    ,   Inverted                );
        goto END_OF_THIS_FUNCTION;
    }
    else
    {
        ProfileStep2PosLinHldPosLin(    CurrentTime
                                    ,   SynchronizationTime
                                    ,   ThisCurrentPosition
                                    ,   ThisCurrentVelocity
                                    ,   ThisTargetPosition
                                    ,   ThisTargetVelocity
                                    ,   MaxAcceleration
                                    ,   &(*PolynomialsInternal)
                                    ,   Inverted                );
        goto END_OF_THIS_FUNCTION;
    }
    // ********************************************************************
MDecision_2___012:
    if (Decision_2___012(       ThisCurrentPosition
                            ,   ThisCurrentVelocity
                            ,   ThisTargetPosition
                            ,   ThisTargetVelocity
                            ,   MaxAcceleration
                            ,   CurrentTime
                            ,   SynchronizationTime     ))
    {
        ProfileStep2PosTrapNegLin(      CurrentTime
                                    ,   SynchronizationTime
                                    ,   ThisCurrentPosition
                                    ,   ThisCurrentVelocity
                                    ,   ThisTargetPosition
                                    ,   ThisTargetVelocity
                                    ,   MaxAcceleration
                                    ,   &(*PolynomialsInternal)
                                    ,   Inverted                );
        goto END_OF_THIS_FUNCTION;
    }
    else
    {
        ProfileStep2NegLinHldNegLinNegLin(      CurrentTime
                                            ,   SynchronizationTime
                                            ,   ThisCurrentPosition
                                            ,   ThisCurrentVelocity
                                            ,   ThisTargetPosition
                                            ,   ThisTargetVelocity
                                            ,   MaxAcceleration
                                            ,   &(*PolynomialsInternal)
                                            ,   Inverted                );
        goto END_OF_THIS_FUNCTION;
    }
    // ********************************************************************
END_OF_THIS_FUNCTION:

    return;
}

