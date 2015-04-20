//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLQuicksort.cpp
//!
//! \brief
//! Implementation file for the Quicksort algorithm
//!
//! \details
//! Implementation file for the Quicksort algorithm to be used within the
//! library of the Type II On-Line Trajectory Generation algorithm.
//! For further information, please refer to the file TypeIIRMLQuicksort.h.
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


#include <TypeIIRMLQuicksort.h>


//************************************************************************************
// Quicksort()


void TypeIIRMLMath::Quicksort(      const int   &LeftBound
                                ,   const int   &RightBound
                                ,   double      *ArrayOfValues)
{
    int             FirstLoopVariable
                ,   SecondLoopVariable;

    double          CurrentValue
                ,   HelpVariable;

    CurrentValue        =   ArrayOfValues[(LeftBound + RightBound) / 2] ;
    FirstLoopVariable   =   LeftBound                                   ;
    SecondLoopVariable  =   RightBound                                  ;

    // try to interexchange elements from the left and from the right
    while ( FirstLoopVariable <= SecondLoopVariable )
    {
        while( ArrayOfValues[FirstLoopVariable] < CurrentValue )
        {
            FirstLoopVariable = FirstLoopVariable + 1;
        }

        while ( ArrayOfValues[SecondLoopVariable] > CurrentValue )
        {
            SecondLoopVariable = SecondLoopVariable - 1;
        }

        if ( FirstLoopVariable <= SecondLoopVariable )
        {
            // Interexchange ArrayOfValues[i] and ArrayOfValues[j]
            HelpVariable                        =   ArrayOfValues[FirstLoopVariable]    ;
            ArrayOfValues[FirstLoopVariable]    =   ArrayOfValues[SecondLoopVariable]   ;
            ArrayOfValues[SecondLoopVariable]   =   HelpVariable                        ;
            FirstLoopVariable                   =   FirstLoopVariable + 1               ;
            SecondLoopVariable                  =   SecondLoopVariable - 1              ;
        }
    }

    // Recursive call for both subsections
    if ( LeftBound < SecondLoopVariable )
    {
        Quicksort(      LeftBound
                    ,   SecondLoopVariable
                    ,   ArrayOfValues       );
    }

    if ( FirstLoopVariable < RightBound )
    {
        Quicksort(      FirstLoopVariable
                    ,   RightBound
                    ,   ArrayOfValues       );
    }
}

