//  ---------------------- Doxygen info ----------------------
//! \file Doxygen_TypeII_Code_06_RMLVelocitySampleApplication.h
//!
//! \brief
//! Documentation file for Doxygen (sample application 6, velocity-based)
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
\page page_Code_06_RMLVelocitySampleApplication Example 6 &mdash; Different Synchronization Behaviors of the Velocity-based algorithm

Below, you can find the source code of a further sample application that makes
use of the velocity-based Reflexxes algorithm, and that generates
a time-synchronized motion trajectory. The source file can 
be found in the folder\n
\n
<tt>/src/RMLVelocitySampleApplications/06_RMLVelocitySampleApplication.cpp</tt>\n
\n
\ref anc_VelocityExample6 "The resulting trajectory of this sample program is displayed below the source code."
\n
\n

\include 06_RMLVelocitySampleApplication.cpp

\n\n
\anchor anc_VelocityExample6

\image html VelocityExample6.png "Resulting trajectory of example 6 for velocity-based trajectory generation (06_RMLVelocitySampleApplication.cpp)."

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
\sa \ref page_Code_04_RMLVelocitySampleApplication



*/

