//  ---------------------- Doxygen info ----------------------
//! \file Doxygen_TypeII_ImportantClasses.h
//!
//! \brief
//! Documentation file for Doxygen (important API classes)
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
\page page_ImportantClasses References to All Classes of the Reflexxes API

The API of any Reflexxes Motion Library was designed to be very compact
and easy-to-use. The only relevant class for user applications is\n\n

 - ReflexxesAPI\n\n

with its two methods ReflexxesAPI::RMLPosition() to execute the
position-based On-line Trajectory Generation algorithm and
ReflexxesAPI::RMLVelocity() for the velocity-based algorithm. These two
methods make use of the classes\n\n

<ul>
<li>RMLInputParameters\n\n
    <ul>
        <li>RMLPositionInputParameters</li>
        <li>RMLVelocityInputParameters\n\n</li>
    </ul></li>
<li>RMLOutputParameters\n\n
    <ul>
        <li>RMLPositionOutputParameters</li>
        <li>RMLVelocityOutputParameters\n\n</li>
    </ul></li>
<li>RMLFlags\n\n
    <ul>
        <li>RMLPositionFlags</li>
        <li>RMLVelocityFlags\n\n</li>
    </ul></li>
</ul>
   
which furthermore use the simple vector class\n\n

 - RMLVector.\n\n

\if PUBLIC
All classes mentioned on this website constitute the entire API of any 
<em>Reflexxes Motion Library</em>.
\endif

*/
