//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLQuicksort.h
//!
//! \brief
//! Header file for the Quicksort algorithm
//!
//! \details
//! Header file for the Quicksort algorithm to be used for within in the
//! library of the Type II OTG algorithm. The Quicksort function is part
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

#ifndef __TypeIIRMLQuicksort__
#define __TypeIIRMLQuicksort__



namespace TypeIIRMLMath
{

//  ---------------------- Doxygen info ----------------------
//! \fn void Quicksort(const int &LeftBound, const int &RightBound, double *ArrayOfValues)
//!
//! \brief
//! Standard implementation of the Quicksort algorithm for \c double values
//!
//! \details
//! This function is used to sort of time values that are calculated during
//! Step 1\n
//! \n
//! \f$ _kt_i^{\,min},\,_kt_i^{\,begin},\,_kt_i^{\,end}\ \forall\ k\ \in\ \left\{1,\,\dots,\,K\right\} \f$\n
//! \n
//! where \f$ K \f$ is the number of degrees of freedom.
//!
//! \param LeftBound
//! Index value for the left border
//!
//! \param RightBound
//! Index value for the right border
//!
//! \param ArrayOfValues
//! A pointer to an array of double values to be sorted
//!
//! \sa TypeIIRMLPosition::Step1()
//  ----------------------------------------------------------
void Quicksort(     const int   &LeftBound
                ,   const int   &RightBound
                ,   double      *ArrayOfValues);

}   // namespace TypeIIRMLMath

#endif
