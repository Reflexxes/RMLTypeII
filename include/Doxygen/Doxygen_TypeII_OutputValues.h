//  ---------------------- Doxygen info ----------------------
//! \file Doxygen_TypeII_OutputValues.h
//!
//! \brief
//! Documentation file for Doxygen (detailed description of output values)
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
\page page_OutputValues Description of Output Values

After calling the position-based On-Line Trajectory Generation (OTG)
algorithm with ReflexxesAPI::RMLPosition(), the output values 
can be read from the class RMLPositionOutputParameters. The output values
of the velocity-based On-Line Trajectory Generation algorithm, which is
called with ReflexxesAPI::RMLVelocity() can be read from the class
RMLVelocityOutputParameters. For a detailed description
of these classes, please refer to the class documentation, and for a
description of the input values of the Reflexxes Motion Library,
please refer to the \ref page_InputValues.\n
\n
This page contains two main sections with one example each:\n\n

<ol>
<li>\ref sec_OutputValuesPosition (\ref sec_ExampleOutputValuesPosition)\n\n</li>
<li>\ref sec_OutputValuesVelocity (\ref sec_ExampleOutputValuesVelocity)\n\n</li>
</ol>

<hr>
\section sec_OutputValuesPosition Output Values of the Position-based On-Line Trajectory Generation Algorithm

\n\n

\image html RMLBasicPositionTypeIIColor.png "Input and output values of the \em Reflexxes \em Type \em II \em Motion \em Library in a generic manner."

The <em>position-based Type II On-Line Trajectory Generation
algorithm</em> called at a time instant \f$ T_i \f$ computes the following
output values (cf. RMLPositionOutputParameters):\n\n

<ul>
  <li>The state of motion \f$ {\bf M}_{i+1} \f$ at time instant \f$ T_{i+1} \f$
    consisting of\n\n
    <ul>
    <li>the new position/pose vector \f$ \vec{P}_{i+1} \ \Longrightarrow \  \f$
      RMLPositionOutputParameters::NewPositionVector,\n\n</li>
    <li>the new velocity vector \f$ \vec{V}_{i+1} \ \Longrightarrow \  \f$
      RMLPositionOutputParameters::NewVelocityVector, and\n\n</li>
    <li>the new acceleration vector \f$ \vec{A}_{i+1} \ \Longrightarrow \  \f$
      RMLPositionOutputParameters::NewAccelerationVector.\n\n</li>
    </ul></li>
 <li>The value of the synchronization time \f$ t_i^{\,sync} \f$ in seconds
   (RMLPositionOutputParameters::SynchronizationTime),
   that is, the time which is required until all selected degrees of freedom
   reach their desired target state of motion \f$ {\bf M}_i^{\,trgt} \f$.\n\n
 </li>
 <li>A boolean flag RMLPositionOutputParameters::ANewCalculationWasPerformed,
    which indicates, what an entirely new computation was performed for
    the call of ReflexxesAPI::RMLPosition() (cf. \ref page_RealTimeBehavior).\n\n</li>
 <li>A boolean flag RMLPositionOutputParameters::TrajectoryIsPhaseSynchronized,
    which indicates, whether the current trajectory is phase-synchronized
    (cf. \ref page_SynchronizationBehavior).\n\n</li> 
 <li>All data about the positional extremum values that will occur if
   the motion trajectory is executed. This includes two vectors, 
   which contain the positional extremum values:\n\n</li>
   <ul>
    <li>the minimum positions for each degree of freedom
      \f$ \overrightarrow{\mbox{min}}\Big(\vec{p}_i\left(t\right)\Big) \ \Longrightarrow \ \f$ 
      RMLPositionOutputParameters::MinPosExtremaPositionVectorOnly and\n\n</li>
    <li>the minimum positions for each degree of freedom
      \f$ \overrightarrow{\mbox{max}}\Big(\vec{p}_i\left(t\right)\Big) \ \Longrightarrow \ \f$ 
      RMLPositionOutputParameters::MaxPosExtremaPositionVectorOnly.\n\n</li>      
   </ul>
   Furthermore, the times in seconds, at which the positional extremum
   values are reached, are stored in\n\n
   <ul>
    <li>RMLPositionOutputParameters::MinExtremaTimesVector and\n\n</li>
    <li>RMLPositionOutputParameters::MaxExtremaTimesVector.\n\n</li>      
   </ul>
   As a further addition, it is also possible to read the states of
   motion of \em all degrees of freedom that are reached if one degree
   of freedom is in its positional minimum or maximum:\n\n
   <ul>
    <li>RMLPositionOutputParameters::MinPosExtremaPositionVectorArray,\n\n</li>
    <li>RMLPositionOutputParameters::MinPosExtremaVelocityVectorArray,\n\n</li>
    <li>RMLPositionOutputParameters::MinPosExtremaAccelerationVectorArray,\n\n</li>
    <li>RMLPositionOutputParameters::MaxPosExtremaPositionVectorArray,\n\n</li>
    <li>RMLPositionOutputParameters::MaxPosExtremaVelocityVectorArray, and\n\n</li>
    <li>RMLPositionOutputParameters::MaxPosExtremaAccelerationVectorArray.\n\n</li>
   </ul>
 </li>
</ul>

\sa RMLPositionOutputParameters
\sa \ref page_InputValues

\n
\n

<hr>

\section sec_ExampleOutputValuesPosition Example

The <b>position-based</b> On-Line Trajectory Generation algorithm of the Type II
Reflexxes Motion Library is called by the method 
ReflexxesAPI::RMLPosition().\n\n

In this example (cf. \ref page_Code_01_RMLPositionSampleApplication), we
assume to control a simple Cartesian robot with three translational degrees
of freedom, and we apply the following input values at time instant
\f$ T_i \f$.\n\n

\image html PositionBasedInputValues.png " "

\image html PositionOutput.png "Sample trajectory generated by the \em position-based Type II On-Line Trajectory Generation algorithm of the Reflexxes Motion Libraries."
  
\n
Further examples can be found on pages

  - \ref page_SynchronizationBehavior,
  - \ref page_PSIfPossible, and
  - \ref page_FinalStateReachedBehavior.

\n

\note
<b>The input variables</b>\n\n
<ul>
    <li>RMLPositionInputParameters::CurrentAccelerationVector
    (containing the the current acceleration vector \f$ \vec{A}_i\f$
      and\n\n</li>
    <li>RMLPositionInputParameters::MaxJerkVector
    (containing the maximum jerk vector \f$ \vec{J}_i^{\,max} \f$
      \n\n</li>
</ul>
<b>are only used by the Type IV Reflexxes Motion Library.</b>
\n

\section sec_ExampleSourceCodeOutputValuesPosition Sample Source Code

For a program to set-up the corresponding input parameters please refer
to the \ref page_Code_01_RMLPositionSampleApplication, to the
\ref page_Code_02_RMLPositionSampleApplication, and to the
\ref page_Result_02_RMLPositionSampleApplication.


\n
\n

<hr>

\section sec_OutputValuesVelocity Output Values of the Velocity-based On-Line Trajectory Generation Algorithm

\n\n

\image html RMLBasicVelocityTypeIIColor.png "Input and output values of the velocity-based \em Type \em II On-Line Trajectory Generation algorithm of the \em Reflexxes \em Motion \em Libraries."


The <em>velocity-based Type II On-Line Trajectory Generation
algorithm</em> is executed by a call of ReflexxesAPI::RMLVelocity(), and
its output values RMLVelocityOutputParameters are the very same as for the
position-based algorithm (see above), but the values for the target
position \f$ _kP_i^{\,trgt} \f$ and the maximum velocity vector 
\f$ \vec{V}_i^{\,max} \f$ are not considered, and the flag
RMLPositionFlags::KeepCurrentVelocityInCaseOfFallbackStrategy is not
available.\n

\sa RMLVelocityOutputParameters
\sa \ref page_InputValues

\n
   
\note
The velocity-based algorithm is also used in the second layer of the
safety mechanism to guarantee valid output value for any given set of
input values (cf. \ref page_ErrorHandling).


\n
\n

<hr>

\section sec_ExampleOutputValuesVelocity Example

For this second example (cf. \ref page_Code_04_RMLVelocitySampleApplication),
we again assume a very simply Cartesian robot with three translational
degrees of freedom.

\image html VelocityBasedInputValues.png " "

\image html VelocityOutput.png "Sample trajectory generated by the \em velocity-based Type II On-Line Trajectory Generation algorithm of the Reflexxes Motion Libraries."


\n

\section sec_ExampleSourceCodeOutputValuesVelocity Sample Source Code

For a program to set-up the corresponding input parameters please refer
to the \ref page_Code_04_RMLVelocitySampleApplication, to the
\ref page_Code_05_RMLVelocitySampleApplication, and to the
\ref page_Result_05_RMLVelocitySampleApplication.

*/
