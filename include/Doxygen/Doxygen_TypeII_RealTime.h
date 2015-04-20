//  ---------------------<li>Doxygen info ----------------------
//! \file Doxygen_TypeII_RealTime.h
//!
//! \brief
//! Documentation file for Doxygen (description of real-time behavior)
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
\page page_RealTimeBehavior Real-Time Behavior

\section sec_WorstCaseExecutionTime Worst-Case Execution Time

The On-Line Trajectory Generation algorithms (OTG) of the Reflexxes Motion
Libraries are <em>real-time capable</em>, that is, <b>a worst-case execution
time will not be exceeded</b> in any case. To achieve this, the two basic
methods of the Reflexxes API (cf. \ref page_ImportantClasses)\n\n

<ul>
<li>ReflexxesAPI::RMLPosition() and\n\n</li>
<li>ReflexxesAPI::RMLVelocity()\n\n</li>
</ul>

execute a limited number of operations. In all cases,
the worst-case execution time scales linearly with the number of selected
degrees of freedom, such that a complexity of \f$ O(n) \f$ results, where
\f$ n \f$ is the number of degrees of freedom.

\n

\section sec_NonRealTimeCapableParts Non-Real-Time Capable Parts

\warning
Please also note, that all \b constructors of all Reflexxes classes <b>are not
real-time capable</b> as heap memory has to be allocated. As a consequence,
all Reflexxes objects have to be created during a start-up phase of the
control system and deleted during a shutdown phase.\n
\n
List of all constructors:
<ul>
<li>ReflexxesAPI::ReflexxesAPI()</li>
<li>RMLPositionInputParameters::RMLPositionInputParameters()</li>
<li>RMLVelocityInputParameters::RMLVelocityInputParameters()</li>
<li>RMLPositionOutputParameters::RMLPositionOutputParameters()</li>
<li>RMLVelocityOutputParameters::RMLVelocityOutputParameters()</li>
<li>RMLPositionFlags::RMLPositionFlags()</li>
<li>RMLVelocityFlags::RMLVelocityFlags()</li>
<li>RMLVector::RMLVector()</li>
</ul>

*/

