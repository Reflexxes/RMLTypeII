//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLDecisions.cpp
//!
//! \brief
//! Implementation file for decisions of the two decision trees of the
//! Type II On-Line Trajectory Generation algorithm
//!
//! \details
//! \sa TypeIIRMLDecision.h
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



#include <TypeIIRMLDecisions.h>
#include <TypeIIRMLMath.h>


//************************************************************************************
// Decision_1A__001()

bool TypeIIRMLMath::Decision_1A__001(       const double &CurrentVelocity)
{
    return(CurrentVelocity >= 0.0);
}


//************************************************************************************
// Decision_1A__002()

bool TypeIIRMLMath::Decision_1A__002(       const double &CurrentVelocity
                                        ,   const double &MaxVelocity)
{
    return(CurrentVelocity <= MaxVelocity);
}


//************************************************************************************
// Decision_1A__003()

bool TypeIIRMLMath::Decision_1A__003(       const double &CurrentVelocity
                                        ,   const double &TargetVelocity)
{
    return(CurrentVelocity <= TargetVelocity);
}


//************************************************************************************
// Decision_1A__004()

bool TypeIIRMLMath::Decision_1A__004(       const double &CurrentPosition
                                        ,   const double &CurrentVelocity
                                        ,   const double &TargetPosition
                                        ,   const double &TargetVelocity
                                        ,   const double &MaxAcceleration)
{
    return(     (CurrentPosition + 0.5 * (pow2(TargetVelocity)
                - pow2(CurrentVelocity)) / MaxAcceleration)
            <=  TargetPosition);
}


//************************************************************************************
// Decision_1A__005()

bool TypeIIRMLMath::Decision_1A__005(       const double &CurrentPosition
                                        ,   const double &CurrentVelocity
                                        ,   const double &TargetPosition
                                        ,   const double &TargetVelocity
                                        ,   const double &MaxVelocity
                                        ,   const double &MaxAcceleration)
{
    return(     (CurrentPosition + (3.0 * pow2(MaxVelocity)
                - pow2(CurrentVelocity) - CurrentVelocity * (MaxVelocity
                - TargetVelocity) - MaxVelocity * TargetVelocity
                - pow2(TargetVelocity)) / (2.0 * MaxAcceleration))
            <=  TargetPosition);
}


//************************************************************************************
// Decision_1A__006()

bool TypeIIRMLMath::Decision_1A__006(       const double &TargetVelocity)
{
    return(TargetVelocity >= 0.0);
}


//************************************************************************************
// Decision_1A__007()

bool TypeIIRMLMath::Decision_1A__007(       const double &CurrentPosition
                                        ,   const double &CurrentVelocity
                                        ,   const double &TargetPosition
                                        ,   const double &TargetVelocity
                                        ,   const double &MaxAcceleration)
{
    return(     (CurrentPosition + 0.5 * (pow2(CurrentVelocity)
                - pow2(TargetVelocity)) / MaxAcceleration)
            >=  TargetPosition);
}


//************************************************************************************
// Decision_1A__008()

bool TypeIIRMLMath::Decision_1A__008(       const double &CurrentPosition
                                        ,   const double &CurrentVelocity
                                        ,   const double &TargetPosition
                                        ,   const double &TargetVelocity
                                        ,   const double &MaxAcceleration)
{
    return(     (CurrentPosition + (pow2(CurrentVelocity)
                - pow2(TargetVelocity)) / (2.0 * MaxAcceleration))
            >=  TargetPosition);

}


//************************************************************************************
// Decision_1A__009()

bool TypeIIRMLMath::Decision_1A__009(       const double &CurrentPosition
                                        ,   const double &CurrentVelocity
                                        ,   const double &TargetPosition
                                        ,   const double &TargetVelocity
                                        ,   const double &MaxVelocity
                                        ,   const double &MaxAcceleration)
{
    return(     (CurrentPosition + (2.0 * pow2(MaxVelocity)
                - pow2(TargetVelocity) - pow2(CurrentVelocity))
                / (2.0 * MaxAcceleration))
            >=  TargetPosition);
}


//************************************************************************************
// Decision_1B__001()

bool TypeIIRMLMath::Decision_1B__001(       const double &CurrentVelocity)
{
    return(Decision_1A__001(CurrentVelocity));
}


//************************************************************************************
// Decision_1B__002()

bool TypeIIRMLMath::Decision_1B__002(       const double &CurrentVelocity
                                        ,   const double &MaxVelocity)
{
    return(Decision_1A__002(    CurrentVelocity
                            ,   MaxVelocity     ));
}


//************************************************************************************
// Decision_1B__003()

bool TypeIIRMLMath::Decision_1B__003(       const double &TargetVelocity)
{
    return(Decision_1A__006(TargetVelocity));
}


//************************************************************************************
// Decision_1B__004()

bool TypeIIRMLMath::Decision_1B__004(       const double &CurrentVelocity
                                        ,   const double &TargetVelocity)
{
    return(Decision_1A__003(    CurrentVelocity
                            ,   TargetVelocity      ));
}


//************************************************************************************
// Decision_1B__005()

bool TypeIIRMLMath::Decision_1B__005(       const double &CurrentPosition
                                        ,   const double &CurrentVelocity
                                        ,   const double &TargetPosition
                                        ,   const double &TargetVelocity
                                        ,   const double &MaxAcceleration)
{
    return(Decision_1A__004(    CurrentPosition
                            ,   CurrentVelocity
                            ,   TargetPosition
                            ,   TargetVelocity
                            ,   MaxAcceleration     ));
}


//************************************************************************************
// Decision_1B__006()

bool TypeIIRMLMath::Decision_1B__006(       const double &CurrentPosition
                                        ,   const double &CurrentVelocity
                                        ,   const double &TargetPosition
                                        ,   const double &TargetVelocity
                                        ,   const double &MaxAcceleration)
{
    return(     (CurrentPosition + (pow2(CurrentVelocity)
                + pow2(TargetVelocity)) / (2.0 * MaxAcceleration))
            <=  TargetPosition);
}


//************************************************************************************
// Decision_1B__007()

bool TypeIIRMLMath::Decision_1B__007(       const double &CurrentPosition
                                        ,   const double &CurrentVelocity
                                        ,   const double &TargetPosition
                                        ,   const double &TargetVelocity
                                        ,   const double &MaxAcceleration)
{
    return(Decision_1A__007(    CurrentPosition
                            ,   CurrentVelocity
                            ,   TargetPosition
                            ,   TargetVelocity
                            ,   MaxAcceleration     ));
}


//************************************************************************************
// Decision_1C__001()

bool TypeIIRMLMath::Decision_1C__001(       const double &CurrentVelocity)
{
    return(Decision_1A__001(CurrentVelocity));
}


//************************************************************************************
// Decision_1C__002()

bool TypeIIRMLMath::Decision_1C__002(       const double &CurrentVelocity
                                        ,   const double &MaxVelocity)
{
    return(Decision_1A__002(    CurrentVelocity
                            ,   MaxVelocity     ));
}


//************************************************************************************
// Decision_1C__003()

bool TypeIIRMLMath::Decision_1C__003(       const double &CurrentPosition
                                        ,   const double &CurrentVelocity
                                        ,   const double &TargetPosition
                                        ,   const double &TargetVelocity
                                        ,   const double &MaxVelocity
                                        ,   const double &MaxAcceleration)
{
    return(Decision_1A__009(    CurrentPosition
                            ,   CurrentVelocity
                            ,   TargetPosition
                            ,   TargetVelocity
                            ,   MaxVelocity
                            ,   MaxAcceleration     ));
}


//************************************************************************************
// Decision_2___001()

bool TypeIIRMLMath::Decision_2___001(       const double &CurrentVelocity)
{
    return(Decision_1A__001(CurrentVelocity));
}


//************************************************************************************
// Decision_2___002()

bool TypeIIRMLMath::Decision_2___002(       const double &CurrentVelocity
                                        ,   const double &MaxVelocity)
{
    return(Decision_1A__002(    CurrentVelocity
                            ,   MaxVelocity         ));
}


//************************************************************************************
// Decision_2___003()

bool TypeIIRMLMath::Decision_2___003(       const double &CurrentVelocity
                                        ,   const double &TargetVelocity)
{
    return(Decision_1A__002(    CurrentVelocity
                            ,   TargetVelocity      ));
}


//************************************************************************************
// Decision_2___004()

bool TypeIIRMLMath::Decision_2___004(       const double &CurrentPosition
                                        ,   const double &CurrentVelocity
                                        ,   const double &TargetPosition
                                        ,   const double &TargetVelocity
                                        ,   const double &MaxAcceleration
                                        ,   const double &CurrentTime
                                        ,   const double &SynchronizationTime)
{
    return(     (CurrentPosition + (SynchronizationTime - CurrentTime)
                * TargetVelocity - pow2(CurrentVelocity - TargetVelocity)
                / (2.0 * MaxAcceleration))
            <=  TargetPosition);
}


//************************************************************************************
// Decision_2___005()

bool TypeIIRMLMath::Decision_2___005(       const double &CurrentPosition
                                        ,   const double &CurrentVelocity
                                        ,   const double &TargetPosition
                                        ,   const double &TargetVelocity
                                        ,   const double &MaxAcceleration
                                        ,   const double &CurrentTime
                                        ,   const double &SynchronizationTime)
{
    return(     (CurrentPosition + (SynchronizationTime - CurrentTime)
                * CurrentVelocity + pow2(CurrentVelocity - TargetVelocity)
                / (2.0 * MaxAcceleration))
            <=  TargetPosition);
}


//************************************************************************************
// Decision_2___006()

bool TypeIIRMLMath::Decision_2___006(       const double &CurrentTime
                                        ,   const double &SynchronizationTime
                                        ,   const double &CurrentPosition
                                        ,   const double &CurrentVelocity
                                        ,   const double &TargetPosition
                                        ,   const double &TargetVelocity
                                        ,   const double &MaxAcceleration)
{
    return(         ((CurrentPosition + (pow2(CurrentVelocity)
                    + pow2(TargetVelocity)) / (2.0 * MaxAcceleration))
                <=  TargetPosition)
            ||      ((((CurrentVelocity + TargetVelocity) / MaxAcceleration)
                >   (SynchronizationTime - CurrentTime))));
}


//************************************************************************************
// Decision_2___007()

bool TypeIIRMLMath::Decision_2___007(       const double &TargetVelocity)
{
    return(Decision_1A__006(TargetVelocity));
}


//************************************************************************************
// Decision_2___008()

bool TypeIIRMLMath::Decision_2___008(       const double &CurrentPosition
                                        ,   const double &CurrentVelocity
                                        ,   const double &TargetPosition
                                        ,   const double &TargetVelocity
                                        ,   const double &MaxAcceleration
                                        ,   const double &CurrentTime
                                        ,   const double &SynchronizationTime)
{
    return(     (CurrentPosition + (SynchronizationTime - CurrentTime)
                * CurrentVelocity - pow2(CurrentVelocity - TargetVelocity)
                / (2.0 * MaxAcceleration))
            <=  TargetPosition);
}


//************************************************************************************
// Decision_2___009()

bool TypeIIRMLMath::Decision_2___009(       const double &CurrentPosition
                                        ,   const double &CurrentVelocity
                                        ,   const double &TargetPosition
                                        ,   const double &TargetVelocity
                                        ,   const double &MaxAcceleration
                                        ,   const double &CurrentTime
                                        ,   const double &SynchronizationTime)
{
    return(     (CurrentPosition + (SynchronizationTime - CurrentTime)
                * TargetVelocity + pow2(CurrentVelocity - TargetVelocity)
                / (2.0 * MaxAcceleration))
            <=  TargetPosition);
}


//************************************************************************************
// Decision_2___010()

bool TypeIIRMLMath::Decision_2___010(       const double &CurrentPosition
                                        ,   const double &CurrentVelocity
                                        ,   const double &TargetPosition
                                        ,   const double &TargetVelocity
                                        ,   const double &MaxAcceleration)
{
    return(Decision_1A__008(    CurrentPosition
                            ,   CurrentVelocity
                            ,   TargetPosition
                            ,   TargetVelocity
                            ,   MaxAcceleration     ));
}


//************************************************************************************
// Decision_2___011()

bool TypeIIRMLMath::Decision_2___011(       const double &CurrentPosition
                                        ,   const double &CurrentVelocity
                                        ,   const double &TargetPosition
                                        ,   const double &TargetVelocity
                                        ,   const double &MaxAcceleration
                                        ,   const double &CurrentTime
                                        ,   const double &SynchronizationTime)
{
    return(Decision_2___004(    CurrentPosition
                            ,   CurrentVelocity
                            ,   TargetPosition
                            ,   TargetVelocity
                            ,   MaxAcceleration
                            ,   CurrentTime
                            ,   SynchronizationTime     ));
}


//************************************************************************************
// Decision_2___012()

bool TypeIIRMLMath::Decision_2___012(       const double &CurrentPosition
                                        ,   const double &CurrentVelocity
                                        ,   const double &TargetPosition
                                        ,   const double &TargetVelocity
                                        ,   const double &MaxAcceleration
                                        ,   const double &CurrentTime
                                        ,   const double &SynchronizationTime)
{
    return(     (CurrentPosition + (SynchronizationTime - CurrentTime)
                * CurrentVelocity - pow2(CurrentVelocity - TargetVelocity)
                / (2.0 * MaxAcceleration))
            <=  TargetPosition);
}

//************************************************************************************
// Decision_V___001()

bool TypeIIRMLMath::Decision_V___001(       const double &CurrentVelocity
                                        ,   const double &TargetVelocity)
{
    return(Decision_1A__003(    CurrentVelocity
                            ,   TargetVelocity  ));
}
