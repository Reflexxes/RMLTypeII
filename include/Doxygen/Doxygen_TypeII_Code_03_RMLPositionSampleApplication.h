//  ---------------------- Doxygen info ----------------------
//! \file Doxygen_TypeII_Code_03_RMLPositionSampleApplication.h
//!
//! \brief
//! Documentation file for Doxygen (sample application 3, position-based)
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
\page page_Code_03_RMLPositionSampleApplication Example 3 &mdash; Different Synchronization Behaviors of the Position-based algorithm

Below, you can find the source code of a sample application that makes
use of the position-based Reflexxes algorithm and that shows, how
to set-up time- and phase-synchronized motion trajectories. The source file
can be found in the folder\n
\n
<tt>/src/RMLPositionSampleApplications/03_RMLPositionSampleApplication.cpp</tt>\n
\n
\ref anc_PositionExample3 "The resulting trajectory of this sample program is displayed below the source code."
\n
\n

\include 03_RMLPositionSampleApplication.cpp

\n\n
\anchor anc_PositionExample3

\image html PositionExample3.png "Resulting trajectory of example 3 for position-based trajectory generation (03_RMLPositionSampleApplication.cpp)."

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

\sa \ref page_SynchronizationBehavior
\sa \ref page_Code_01_RMLPositionSampleApplication 
\sa \ref page_Code_02_RMLPositionSampleApplication 

*/
