//  ---------------------- Doxygen info ----------------------
//! \file Doxygen_TypeII_Code_02_RMLPositionSampleApplication.h
//!
//! \brief
//! Documentation file for Doxygen (sample application 2, position-based)
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
\page page_Code_02_RMLPositionSampleApplication Example 2 &mdash; Making Use of Output Parameters of the Position-based algorithm

Below, you can find the source code of a sample application that makes
use of the position-based Reflexxes algorithm and shows, how to
interpret the output values of the algorithm. The source file can 
be found in the folder\n
\n
<tt>/src/RMLPositionSampleApplications/02_RMLPositionSampleApplication.cpp</tt>\n
\n
\ref anc_PositionExample2 "The resulting trajectory of this sample program is displayed below the source code."
\n
\n

\include 02_RMLPositionSampleApplication.cpp

\n\n
\anchor anc_PositionExample2

\image html PositionExample2.png "Resulting trajectory of example 2 for position-based trajectory generation (02_RMLPositionSampleApplication.cpp)."

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

\sa \ref page_Result_02_RMLPositionSampleApplication
\sa \ref page_OutputValues
\sa \ref page_Code_05_RMLVelocitySampleApplication
\sa \ref page_Code_03_RMLPositionSampleApplication

*/

