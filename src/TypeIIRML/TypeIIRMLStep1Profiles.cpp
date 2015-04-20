//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLStep1Profiles.cpp
//!
//! \brief
//! Implementation file for the calculation of all Step 1 motion
//! profiles for the Type II On-Line Trajectory Generation algorithm
//!
//! \details
//! For further information, please refer to the file
//! TypeIIRMLStep1Profiles.h
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



#include <TypeIIRMLStep1Profiles.h>
#include <TypeIIRMLMath.h>
#include <math.h>


//************************************************************************************
// ProfileStep1PosLinHldNegLin()

double TypeIIRMLMath::ProfileStep1PosLinHldNegLin(      const double &CurrentPosition
                                                    ,   const double &CurrentVelocity
                                                    ,   const double &TargetPosition
                                                    ,   const double &TargetVelocity
                                                    ,   const double &MaxVelocity
                                                    ,   const double &MaxAcceleration)
{
    return( (2.0 * MaxAcceleration * (TargetPosition - CurrentPosition)
            +   pow2(CurrentVelocity)
            +   pow2(TargetVelocity)
            +   2.0 * MaxVelocity * (MaxVelocity - CurrentVelocity
            -   TargetVelocity))
            /   (2.0 * MaxAcceleration * MaxVelocity)                   );
}


//************************************************************************************
// ProfileStep1PosLinNegLin()

double TypeIIRMLMath::ProfileStep1PosLinNegLin(     const double &CurrentPosition
                                                ,   const double &CurrentVelocity
                                                ,   const double &TargetPosition
                                                ,   const double &TargetVelocity
                                                ,   const double &MaxAcceleration)
{
    return ((1.4142135623730950488016887242096980785696718753769480731766
            *   RMLSqrt(pow2(MaxAcceleration) * (2.0 * MaxAcceleration
            *   (TargetPosition - CurrentPosition) + pow2(CurrentVelocity)
            +   pow2(TargetVelocity))) - MaxAcceleration
            *   (CurrentVelocity + TargetVelocity))
            /   (pow2(MaxAcceleration))                                     );
}


//************************************************************************************
// ProfileStep1PosTriNegLin()

double TypeIIRMLMath::ProfileStep1PosTriNegLin(     const double &CurrentPosition
                                                ,   const double &CurrentVelocity
                                                ,   const double &TargetPosition
                                                ,   const double &TargetVelocity
                                                ,   const double &MaxAcceleration)
{
    return ((1.4142135623730950488016887242096980785696718753769480731766
            *   RMLSqrt(pow2(MaxAcceleration) * (pow2(CurrentVelocity)
            +   pow2(TargetVelocity) + 2.0 * MaxAcceleration
            *   (TargetPosition - CurrentPosition )))
            -   MaxAcceleration * (CurrentVelocity + TargetVelocity))
            /   (pow2(MaxAcceleration))                                     );
}


//************************************************************************************
// ProfileStep1PosTrapNegLin()

double TypeIIRMLMath::ProfileStep1PosTrapNegLin(        const double &CurrentPosition
                                                    ,   const double &CurrentVelocity
                                                    ,   const double &TargetPosition
                                                    ,   const double &TargetVelocity
                                                    ,   const double &MaxVelocity
                                                    ,   const double &MaxAcceleration)
{
    return ((TargetPosition - CurrentPosition) / MaxVelocity
            +   (0.5 * (pow2(CurrentVelocity) + pow2(TargetVelocity))
            +   MaxVelocity * (MaxVelocity - CurrentVelocity
            -   TargetVelocity)) / (MaxAcceleration * MaxVelocity));
}


//************************************************************************************
// ProfileStep1NegLinPosLin()

double TypeIIRMLMath::ProfileStep1NegLinPosLin(     const double &CurrentPosition
                                                ,   const double &CurrentVelocity
                                                ,   const double &TargetPosition
                                                ,   const double &TargetVelocity
                                                ,   const double &MaxAcceleration)
{
    return ( (CurrentVelocity + TargetVelocity
            -   1.4142135623730950488016887242096980785696718753769480731766
            *   RMLSqrt(2.0 * MaxAcceleration
            *   (CurrentPosition - TargetPosition)
            +   pow2(CurrentVelocity) + pow2(TargetVelocity)))
            /   MaxAcceleration                                                 );
}


//************************************************************************************
// IsSolutionForProfile_PosLinHldNegLin_Possible()

bool TypeIIRMLMath::IsSolutionForProfile_PosLinHldNegLin_Possible(      const double &CurrentPosition
                                                                    ,   const double &CurrentVelocity
                                                                    ,   const double &TargetPosition
                                                                    ,   const double &TargetVelocity
                                                                    ,   const double &MaxVelocity
                                                                    ,   const double &MaxAcceleration)
{
    // Check for t01 and vi
    if (    (CurrentVelocity > MaxVelocity  )
        ||  (CurrentVelocity < 0.0          )   )
    {
        return (false);
    }

    // Check for t12
    if (    (2.0 * MaxAcceleration * (TargetPosition - CurrentPosition)
            + pow2(CurrentVelocity) - 2.0 * pow2(MaxVelocity)
            + pow2(TargetVelocity)) < -RML_VALID_SOLUTION_EPSILON)
    {
        return (false);
    }

    // Check for t23 and vtrgt
    if (    (TargetVelocity > MaxVelocity)
        ||  (TargetVelocity < 0.0)  )
    {
        return (false);
    }

    return (true);
}


//************************************************************************************
// IsSolutionForProfile_PosLinNegLin_Possible()

bool TypeIIRMLMath::IsSolutionForProfile_PosLinNegLin_Possible(     const double &CurrentPosition
                                                                ,   const double &CurrentVelocity
                                                                ,   const double &TargetPosition
                                                                ,   const double &TargetVelocity
                                                                ,   const double &MaxVelocity
                                                                ,   const double &MaxAcceleration)
{
    double      BufferVariable          =   0.0
            ,   SqrtOfBufferVariable    =   0.0
            ,   PeakVelocity            =   0.0;

    // Check for vi
    if (    (CurrentVelocity > MaxVelocity  )
        ||  (CurrentVelocity < 0.0          )   )
    {
        return (false);
    }

    // Check for vtrgt
    if (    (TargetVelocity > MaxVelocity   )
        ||  (TargetVelocity < 0.0           )   )
    {
        return (false);
    }

    BufferVariable  =   pow2(MaxAcceleration) * (2.0
                        *   MaxAcceleration * (TargetPosition
                        -   CurrentPosition) + pow2(CurrentVelocity)
                        +   pow2(TargetVelocity));

    // Check for vpeak
    if (BufferVariable < -RML_VALID_SOLUTION_EPSILON)
    {
        return (false);
    }

    SqrtOfBufferVariable    =   RMLSqrt(BufferVariable);

    PeakVelocity            =   SqrtOfBufferVariable
                                /   (1.4142135623730950488016887242096980785696718753769480731766
                                *   MaxAcceleration);

    if (    (PeakVelocity > MaxVelocity     )
        ||  (PeakVelocity < CurrentVelocity )
        ||  (PeakVelocity < TargetVelocity  )   )
    {
        return (false);
    }

    // Check for t01
    if (    ((1.4142135623730950488016887242096980785696718753769480731766
            *   SqrtOfBufferVariable - 2.0 * MaxAcceleration
            *   CurrentVelocity) / (2.0 * pow2(MaxAcceleration)))
            <   -RML_VALID_SOLUTION_EPSILON)
    {
        return (false);
    }

    // Check for t12
    if (    ((1.4142135623730950488016887242096980785696718753769480731766
            *   SqrtOfBufferVariable - 2.0 * MaxAcceleration
            *   TargetVelocity) / (2.0 * pow2(MaxAcceleration)))
            <   -RML_VALID_SOLUTION_EPSILON)
    {
        return (false);
    }

    return (true);
}


//************************************************************************************
// IsSolutionForProfile_PosTriNegLin_Possible()

bool TypeIIRMLMath::IsSolutionForProfile_PosTriNegLin_Possible(     const double &CurrentPosition
                                                                ,   const double &CurrentVelocity
                                                                ,   const double &TargetPosition
                                                                ,   const double &TargetVelocity
                                                                ,   const double &MaxVelocity
                                                                ,   const double &MaxAcceleration)
{
    double      BufferVariable          =   0.0
            ,   SqrtOfBufferVariable    =   0.0
            ,   PeakVelocity            =   0.0;

    // Check for vi
    if (    (CurrentVelocity > MaxVelocity  )
        ||  (CurrentVelocity < 0.0          )   )
    {
        return (false);
    }

    // Check for t23 and vtrgt
    if (    (TargetVelocity < -MaxVelocity  )
        ||  (TargetVelocity > 0.0           )   )
    {
        return (false);
    }

    BufferVariable  =   pow2(MaxAcceleration) * (2.0
                        *   MaxAcceleration * (TargetPosition
                        -   CurrentPosition) + pow2(CurrentVelocity)
                        +   pow2(TargetVelocity));

    // Check for vpeak
    if (BufferVariable < -RML_VALID_SOLUTION_EPSILON)
    {
        return (false);
    }

    SqrtOfBufferVariable    =   RMLSqrt(BufferVariable);

    PeakVelocity            =   SqrtOfBufferVariable
                                /   (1.4142135623730950488016887242096980785696718753769480731766
                                *   MaxAcceleration);

    if (    (PeakVelocity > MaxVelocity     )
        ||  (PeakVelocity < CurrentVelocity )   )
    {
        return (false);
    }

    // Check for t01
    if (    ((1.4142135623730950488016887242096980785696718753769480731766
            *   SqrtOfBufferVariable - 2.0 * MaxAcceleration
            *   CurrentVelocity) / (2.0 * pow2(MaxAcceleration)))
            <   -RML_VALID_SOLUTION_EPSILON)
    {
        return (false);
    }

    // Check for t12
    if (    ((1.4142135623730950488016887242096980785696718753769480731766
            *   SqrtOfBufferVariable - 2.0 * MaxAcceleration
            *   TargetVelocity) / (2.0 * pow2(MaxAcceleration)))
            <   -RML_VALID_SOLUTION_EPSILON)
    {
        return (false);
    }

    return (true);
}


//************************************************************************************
// IsSolutionForProfile_PosTrapNegLin_Possible()

bool TypeIIRMLMath::IsSolutionForProfile_PosTrapNegLin_Possible(    const double &CurrentPosition
                                                                ,   const double &CurrentVelocity
                                                                ,   const double &TargetPosition
                                                                ,   const double &TargetVelocity
                                                                ,   const double &MaxVelocity
                                                                ,   const double &MaxAcceleration)
{
    // Check for t01 and vi
    if (    (CurrentVelocity > MaxVelocity  )
        ||  (CurrentVelocity < 0.0          )   )
    {
        return (false);
    }

    // Check for t12
    if (    (2.0 * MaxAcceleration * (TargetPosition - CurrentPosition)
            + pow2(CurrentVelocity) - 2.0 * pow2(MaxVelocity)
            + pow2(TargetVelocity)) < -RML_VALID_SOLUTION_EPSILON)
    {
        return (false);
    }

    // Check for t34 and vtrgt
    if (    (TargetVelocity < -MaxVelocity)
        ||  (TargetVelocity > 0.0)  )
    {
        return (false);
    }

    return (true);
}
