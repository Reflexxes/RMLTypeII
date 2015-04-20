//  ---------------------- Doxygen info ----------------------
//! \file Doxygen_TypeII_Code_01_RMLPositionSampleApplication.h
//!
//! \brief
//! Documentation file for Doxygen (sample application 1, position-based)
//! \n
//!
//! \date April 2015
//! 
//! \version 1.2.7
//!
//!\author Torsten Kroeger, <info@reflexxes.com> \n
//!
//!
//! \copyright Copyright (C) 2015 Google, Inc.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------



// -----------------------------------------------------------------
/*!
\page page_Code_01_RMLPositionSampleApplication Example 1 &mdash; Introduction to the Position-based algorithm

Below, you can find the source code of a very first sample application that makes
use of the position-based Reflexxes algorithm. The source file can 
be found in the folder\n
\n
<tt>/src/RMLPositionSampleApplications/01_RMLPositionSampleApplication.cpp</tt>\n
\n
\ref anc_PositionExample1 "The resulting trajectory of this sample program is displayed below the source code."

\n
\n

\include 01_RMLPositionSampleApplication.cpp

\n\n
\anchor anc_PositionExample1

\image html PositionExample1.png "Resulting trajectory of example 1 for position-based trajectory generation (01_RMLPositionSampleApplication.cpp)."

\n

\note
<b>The variables</b>\n\n
<ul>
    <li>RMLPositionInputParameters::CurrentAccelerationVector
      (containing the the current acceleration vector \f$ \vec{A}_i\f$
      and\n\n</li>
    <li>RMLPositionInputParameters::MaxJerkVector
      (containing the maximum jerk vector \f$ \vec{J}_i^{\,max} \f$
      \n\n</li>
</ul>
<b>are only used by the Type IV Reflexxes Motion Library.</b>

\n\n

\sa \ref page_OutputValues
\sa \ref page_Code_02_RMLPositionSampleApplication
\sa \ref page_Code_04_RMLVelocitySampleApplication


*/

