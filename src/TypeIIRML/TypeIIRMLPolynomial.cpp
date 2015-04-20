//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLPolynomial.cpp
//!
//! \brief
//! Implementation file for for the class TypeIIRMLMath::TypeIIRMLPolynomial
//!
//! \details
//! Implementation file for the a polynomial class designed
//! for the Type II On-Line Trajectory Generation algorithm.
//! For further information, please refer to the file
//! TypeIIRMLPolynomial.h.
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


#include <TypeIIRMLPolynomial.h>
#include <TypeIIRMLMath.h>


//************************************************************************************
// Constructor()

TypeIIRMLMath::TypeIIRMLPolynomial::TypeIIRMLPolynomial()
{
    a0        =    0.0;
    a1        =    0.0;
    a2        =    0.0;
    DeltaT    =    0.0;
    Degree    =    0;
}


//************************************************************************************
// Destructor()

TypeIIRMLMath::TypeIIRMLPolynomial::~TypeIIRMLPolynomial()
{}


//************************************************************************************
// SetCoefficients()
// f(t) = a_2 * (t - DeltaT)^2 + a_1 * (t - DeltaT) + a_0

void TypeIIRMLMath::TypeIIRMLPolynomial::SetCoefficients(        const double    &Coeff2
                                                            ,    const double    &Coeff1
                                                            ,    const double    &Coeff0
                                                            ,    const double    &Diff)
{
    a0        =    Coeff0;
    a1        =    Coeff1;
    a2        =    Coeff2;
    DeltaT    =    Diff;

    if (a2 != 0.0)
    {
        Degree    =    2;
        return;
    }

    if (a1 != 0.0)
    {
        Degree    =    1;
        return;
    }

    Degree    =    0;
    return;
}



//************************************************************************************
// GetCoefficients()

void TypeIIRMLMath::TypeIIRMLPolynomial::GetCoefficients(    double    *Coeff2
                                                        ,    double    *Coeff1
                                                        ,    double    *Coeff0
                                                        ,    double    *Diff    ) const
{
    *Coeff2     =    this->a2;
    *Coeff1     =    this->a1;
    *Coeff0     =    this->a0;
    *Diff       =    this->DeltaT;

    return;
}



//*******************************************************************************************
// CalculateValue()
// calculates f(t)

double TypeIIRMLMath::TypeIIRMLPolynomial::CalculateValue(const double &t) const
{
    return( ((Degree == 2)?
            (a2 * (t - DeltaT) * (t - DeltaT) + a1 * (t - DeltaT) + a0):
            ((Degree == 1)?
            (a1 * (t - DeltaT) + a0):
            (a0))));
}


//*******************************************************************************************
// CalculateRoots()

void TypeIIRMLMath::TypeIIRMLPolynomial::CalculateRealRoots(        unsigned int    *NumberOfRoots
                                                                ,   double          *Root1
                                                                ,   double          *Root2) const
{
    if (this->Degree == 2)
    {
        double       b0                =    this->a0 / this->a2
                ,    b1                =    this->a1 / this->a2
                ,    SquareRootTerm    =    0.25 * pow2(b1) - b0;

        if (SquareRootTerm < 0.0)
        {
            // only complex roots

            *Root1            =    0.0    ;
            *Root2            =    0.0    ;
            *NumberOfRoots    =    0    ;
        }
        else
        {
            // Polynomial of degree two: x^2 + b1 x + b2 = 0

            SquareRootTerm    =    TypeIIRMLMath::RMLSqrt(SquareRootTerm);

            *Root1            =    - 0.5 * b1 + SquareRootTerm  + this->DeltaT;
            *Root2            =    - 0.5 * b1 - SquareRootTerm  + this->DeltaT;
            *NumberOfRoots    =    2    ;
        }
        return;
    }

    if (this->Degree == 1)
    {
        *Root1            =    - this->a0 / this->a1 + this->DeltaT;
        *Root2            =    0.0    ;
        *NumberOfRoots    =    1    ;
        return;
    }

    if (this->Degree == 0)
    {
        *Root1            =    0.0    ;
        *Root2            =    0.0    ;
        *NumberOfRoots    =    0    ;
    }

    return;
}

