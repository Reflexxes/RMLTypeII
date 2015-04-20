//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLPolynomial.h
//!
//! \brief
//! Header file for the class TypeIIRMLMath::TypeIIRMLPolynomial and the
//! struct TypeIIRMLMath::MotionPolynomials
//!
//! \details
//! Header file for a polynomial class designed for the Type II
//! On-Line Trajectory Generation algorithm. This class is part
//! of the namespace TypeIIRMLMath.
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


#ifndef __TypeIIRMLPolynomial__
#define __TypeIIRMLPolynomial__


#include <TypeIIRMLMath.h>

namespace TypeIIRMLMath
{


//  ---------------------- Doxygen info ----------------------
//! \class TypeIIRMLPolynomial
//!
//! \brief
//! This class realizes polynomials of degree three as required
//! for the Type II On-Line Trajectory Generation algorithm
//!
//! \sa struct MotionPolynomials
//  ----------------------------------------------------------
class TypeIIRMLPolynomial
{
public:


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIIRMLPolynomial(void)
//!
//! \brief
//! Constructor of the class TypeIIRMLPolynomial
//  ----------------------------------------------------------
    TypeIIRMLPolynomial(void);


//  ---------------------- Doxygen info ----------------------
//! \fn ~TypeIIRMLPolynomial(void)
//!
//! \brief
//! Destructor of the class TypeIIRMLPolynomial
//  ----------------------------------------------------------
    ~TypeIIRMLPolynomial(void);


//  ---------------------- Doxygen info ----------------------
//! \fn void SetCoefficients(const double &Coeff2, const double &Coeff1, const double &Coeff0, const double &Diff)
//!
//! \brief
//! Sets the coefficients of the polynomial object
//!
//! \details
//! Sets the coefficients for the polynomial of degree three
//! \f$ f(t) = a_2 \cdot (t - \Delta T)^2 + a_1 \cdot (t - \Delta T) + a_0 \f$.
//!
//! \param Coeff2
//! \f$\ \Longrightarrow \ a_2\f$
//!
//! \param Coeff1
//! \f$\ \Longrightarrow \ a_1\f$
//!
//! \param Coeff0
//! \f$\ \Longrightarrow \ a_0\f$
//!
//! \param Diff
//! \f$\ \Longrightarrow \ \Delta T\f$
//  ----------------------------------------------------------
    void        SetCoefficients(    const double    &Coeff2
                                ,   const double    &Coeff1
                                ,   const double    &Coeff0
                                ,   const double    &Diff);


//  ---------------------- Doxygen info ----------------------
//! \fn void GetCoefficients(double *Coeff2, double *Coeff1, double *Coeff0, double *Diff) const
//!
//! \brief
//! Returns the coefficients of the polynomial object
//!
//! \details
//! Gets the coefficients for the polynomial of degree three
//! \f$ f(t) = a_2 \cdot (t - \Delta T)^2 + a_1 \cdot (t - \Delta T) + a_0 \f$.
//!
//! \param Coeff2
//! \f$\ \Longrightarrow \ a_2\f$
//!
//! \param Coeff1
//! \f$\ \Longrightarrow \ a_1\f$
//!
//! \param Coeff0
//! \f$\ \Longrightarrow \ a_0\f$
//!
//! \param Diff
//! \f$\ \Longrightarrow \ \Delta T\f$
//  ----------------------------------------------------------
    void        GetCoefficients(    double  *Coeff2
                                ,   double  *Coeff1
                                ,   double  *Coeff0
                                ,   double  *Diff   ) const;




//  ---------------------- Doxygen info ----------------------
//! \fn void CalculateRealRoots(unsigned int *NumberOfRoots, double *Root1, double *Root2) const
//!
//! \brief
//! Calculates the real roots of the polynomial specified by the
//! attributes of this object
//!
//! \details
//! Calculates up to three roots of the polynomial of degree two
//! \f$ f(t) = a_2 \cdot (t - \Delta T)^2 + a_1 \cdot (t - \Delta T) + a_0 \f$,
//! that is
//! \f$ f(t) = (t - r_1) \cdot (t - r_2) \f$,
//! where \f$ r_1 \f$ and \f$ r_2 \f$ are the desired roots
//! of the current polynomial function.
//!
//! \param NumberOfRoots
//! A pointer to the value of the number of roots (0...2)
//!
//! \param Root1
//! A pointer to the value of the first root (only valid for polynomials of degree one and higher)
//!
//! \param Root2
//! A pointer to the value of the second root (only valid for polynomials of degree two)
//  ----------------------------------------------------------
    void        CalculateRealRoots  (       unsigned int    *NumberOfRoots
                                        ,   double          *Root1
                                        ,   double          *Root2) const;


//  ---------------------- Doxygen info ----------------------
//! \fn double CalculateValue(const double &t) const
//!
//! \brief
//! Calculates the function value at \f$ t \f$ of the polynomial specified
//! by the attributes of this object
//!
//! \details
//! Calculates the function value of\f$ f(t) \f$ at \f$t\f$\n
//! \f$ f(t) = a_2 \cdot (t - \Delta T)^2 + a_1 \cdot (t - \Delta T) + a_0 \f$.
//!
//! \param t function input value
//!
//! \return The function value at \f$t\f$
//  ----------------------------------------------------------
    double      CalculateValue(const double &t) const;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int Degree
//!
//! \brief
//! Positive integer number that defines the degree of the current polynomial
//  ----------------------------------------------------------
    unsigned int    Degree;


//  ---------------------- Doxygen info ----------------------
//! \var double a2
//!
//! \brief
//! Parameter \f$ a_2 \f$ of the polynomial function
//! \f$ f(t) = a_2 \cdot (t - \Delta T)^2 + a_1 \cdot (t - \Delta T) + a_0 \f$
//  ----------------------------------------------------------
    double          a2;


//  ---------------------- Doxygen info ----------------------
//! \var double a1
//!
//! \brief
//! Parameter \f$ a_1 \f$ of the polynomial function
//! \f$ f(t) = a_2 \cdot (t - \Delta T)^2 + a_1 \cdot (t - \Delta T) + a_0 \f$
//  ----------------------------------------------------------
    double          a1;


//  ---------------------- Doxygen info ----------------------
//! \var double a0
//!
//! \brief
//! Parameter \f$ a_0 \f$ of the polynomial function
//! \f$ f(t) = a_2 \cdot (t - \Delta T)^2 + a_1 \cdot (t - \Delta T) + a_0 \f$
//  ----------------------------------------------------------
    double          a0;


//  ---------------------- Doxygen info ----------------------
//! \var double DeltaT
//!
//! \brief
//! Parameter \f$ \Delta T \f$ of the polynomial function
//! \f$ f(t) = a_2 \cdot (t - \Delta T)^2 + a_1 \cdot (t - \Delta T) + a_0 \f$
//  ----------------------------------------------------------
    double          DeltaT;

};  // class TypeIIRMLPolynomial


//  ---------------------- Doxygen info ----------------------
//! \struct MotionPolynomials
//!
//! \brief
//! Three arrays of TypeIIRMLMath::TypeIIRMLPolynomial
//!
//! \details
//! This data structure contains three arrays of polynomials required for
//! the Type II On-Line Trajectory Generation algorithm. Furthermore, this
//! data structure contains the times until each single two-tuple of
//! polynomials is valid and the number of used polynomial two-tuples
//! that are currently in use. The value of \c MAXIMAL_NO_OF_POLYNOMIALS in
//! the file TypeIIRMLMath.h.
//!
//! \sa TypeIIRMLPolynomial
//  ----------------------------------------------------------
struct MotionPolynomials
{
//  ---------------------- Doxygen info ----------------------
//! \var double PolynomialTimes [MAXIMAL_NO_OF_POLYNOMIALS]
//!
//! \brief
//! An array of ending times in seconds
//!
//! \details
//! An array of ending times in seconds, until which a polynomial is valid
//! (e.g., \c PolynomialTimes[4] determines the ending time of the fourth
//! polynomial).
//  ----------------------------------------------------------
    double                  PolynomialTimes         [MAXIMAL_NO_OF_POLYNOMIALS] ;


//  ---------------------- Doxygen info ----------------------
//! \var TypeIIRMLPolynomial PositionPolynomial [MAXIMAL_NO_OF_POLYNOMIALS]
//!
//! \brief
//! An array of position polynomials
//!
//! \details
//! An array of position polynomials, that is, objects of the
//! class TypeIIRMLPolynomial, that is,
//! \f$\ _{k}^{l}p_{i}(t)\ \forall\ l\ \in\ \{1,\,\dots,\,L\} \f$,
//! where \f$ L \f$ is value of \c MAXIMAL_NO_OF_POLYNOMIALS.
//  ----------------------------------------------------------
    TypeIIRMLPolynomial     PositionPolynomial      [MAXIMAL_NO_OF_POLYNOMIALS] ;


//  ---------------------- Doxygen info ----------------------
//! \var TypeIIRMLPolynomial VelocityPolynomial [MAXIMAL_NO_OF_POLYNOMIALS]
//!
//! \brief
//! An array of velocity polynomials
//!
//! \details
//! An array of velocity polynomials, that is, objects of the
//! class TypeIIRMLPolynomial, that is,
//! \f$\ _{k}^{l}v_{i}(t)\ \forall\ l\ \in\ \{1,\,\dots,\,L\} \f$,
//! where \f$ L \f$ is value of \c MAXIMAL_NO_OF_POLYNOMIALS.
//  ----------------------------------------------------------
    TypeIIRMLPolynomial     VelocityPolynomial      [MAXIMAL_NO_OF_POLYNOMIALS] ;


//  ---------------------- Doxygen info ----------------------
//! \var TypeIIRMLPolynomial AccelerationPolynomial [MAXIMAL_NO_OF_POLYNOMIALS]
//!
//! \brief
//! An array of acceleration polynomials
//!
//! \details
//! An array of acceleration polynomials, that is, objects of the
//! class TypeIIRMLPolynomial, that is,
//! \f$\ _{k}^{l}a_{i}(t)\ \forall\ l\ \in\ \{1,\,\dots,\,L\} \f$,
//! where \f$ L \f$ is value of \c MAXIMAL_NO_OF_POLYNOMIALS.
//  ----------------------------------------------------------
    TypeIIRMLPolynomial     AccelerationPolynomial  [MAXIMAL_NO_OF_POLYNOMIALS] ;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned char ValidPolynomials
//!
//! \brief
//! The number of polynomials in use (0 ... \c MAXIMAL_NO_OF_POLYNOMIALS)
//  ----------------------------------------------------------
    unsigned char           ValidPolynomials                                    ;
};


}   // namespace TypeIIRMLMath


#endif
