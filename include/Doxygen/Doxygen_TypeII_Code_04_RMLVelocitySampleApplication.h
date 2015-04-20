//  ---------------------- Doxygen info ----------------------
//! \file Doxygen_TypeII_Code_04_RMLVelocitySampleApplication.h
//!
//! \brief
//! Documentation file for Doxygen (sample application 4, velocity-based)
//!
//! \date April 2015
//! 
//! \version 1.2.7
//!
//! \author Torsten Kroeger, <info@reflexxes.com> \n
//! 
//!
//! \copyright Copyright (C) 2015 Google, Inc.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------



// -----------------------------------------------------------------
/*!
\page page_Code_04_RMLVelocitySampleApplication Example 4 &mdash; Introduction to the Velocity-based algorithm

Below, you can find the source code of a very first sample application that makes
use of the velocity-based Reflexxes algorithm. The source file can 
be found in the folder\n
\n
<tt>/src/RMLVelocitySampleApplications/04_RMLVelocitySampleApplication.cpp</tt>\n
\n
\ref anc_VelocityExample4 "The resulting trajectory of this sample program is displayed below the source code."
\n
\n

\include 04_RMLVelocitySampleApplication.cpp

\n\n
\anchor anc_VelocityExample4

\image html VelocityExample4.png "Resulting trajectory of example 4 for velocity-based trajectory generation (04_RMLVelocitySampleApplication.cpp)."

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
\sa \ref page_Code_05_RMLVelocitySampleApplication
\sa \ref page_Code_01_RMLPositionSampleApplication


*/

