//  ---------------------- Doxygen info ----------------------
//! \file Doxygen_TypeII_ReflexxesMotionLibraries.h
//!
//! \brief
//! Documentation file for Doxygen (Reflexxes Motion Libraries)
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
\page page_ReflexxesMotionLibraries The Reflexxes Motion Libraries &mdash; An Overview

\section sec_IntroductionOverview Introduction

The <em>Reflexxes Motion Libraries</em> are designed to achieve new
opportunities in sensor-based motion control opening the door to new
applications, safe human-robot interaction, and advanced robot motion
capabilities. The three key features of these libraries are:

- <b>Jerk-limited robot motions can be calculated from arbitrary initial states of motion
  (i.e., during any motion).</b>
- <b>New motions are calculated within one low-level control cycle
  (typically within one millisecond or less).</b>
- <b>The interface is very simple and clear, such that it can easily be
  integrated in existing systems.</b>

Based on the Reflexxes framework, the following new motion control features
can be realized with the <em>Reflexxes Motion Libraries</em>:

- <b>Instantaneous changes of coordinate frames</b>
- <b>Instantaneous changes of control state spaces</b>
- <b>Deterministic, instantaneous reactions to sensor signals</b>
- <b>Safe and stable reactions to sensor failures</b>
- <b>Simple and robust visual servo control</b>
- <b>Stable switched-system control</b>

All these features let robots and mechanical systems \b instantaneously
react to \b unforeseen events and enable new opportunities for
sensor-based robot motion control &mdash; and and thus helping to realize
new and advanced robot and motion control applications.\n\n\n
 

\section sec_Overview Overview

This page gives an overview about\n

- the <b>Reflexxes \ref sec_TypeIIBrief "Type II" Motion Library</b> and\n\n
- the <b>Reflexxes \ref sec_TypeIVBrief "Type IV" Motion Library</b>.\n

Each Reflexxes Motion Library contains a set of <b>On-Line Trajectory
Generation (OTG)</b> algorithms that are designed to control robots and
mechanical systems in order to take advantage of the features above.
A description is also provided in:\n

<b>T. Kroeger.</b>\n
<a href="papers/ReflexxesICRA2011.pdf" target="_blank" title="Click to view the pdf file."><b>Opening the Door to New Sensor-based Robot Applications &mdash; The Reflexxes Motion Libraries</b></a>.\n
<b>In Proc. of the IEEE International Conference on Robotics and Automation, Shanghai, China, May 2011.</b>\n\n

The \b difference between <b>Type II</b> and <b>Type IV</b> is the usage
of <b>jerk-limitation</b> and the consideration of <b>initial 
acceleration values</b> in the Type IV library.
\n




<hr>
\section sec_TypeIIBrief Type II Reflexxes Motion Library


 <ul>
  <li>The Type II Reflexxes Motion Library is released as open source
      software under the
      <a href="http://www.gnu.org/licenses/lgpl.html" target="_blank" title="http://www.gnu.org/licenses/lgpl.html"><b>GNU Lesser General Public License</b></a>.
      </li>
  <li>Includes Type I</li>
  <li>Position-based Type II On-Line Trajectory Generation (OTG) algorithm
   <ul>
    <li>Acceleration limitation</li>
    <li>Time- and phase-synchronization</li>
    <li>Time-optimal trajectories</li>
    <li>The desired target state of motion at the end of each motion
        trajectory only consists of a desired position/pose/vector
        and velocity vector.</li>
    <li>Input and output values:
        \image html RMLBasicPositionTypeIIColor.png "Input and output values of the \em position-based \em Type \em II \em On-Line \em Trajectory \em Generation algorithm."</li>
   </ul></li>
   <li>Velocity-based Type II On-Line Trajectory Generation (OTG) algorithm
      <ul>
      <li>Acceleration limitation</li>
      <li>Time-optimal trajectories</li>
      <li>Input and output values:
        \image html RMLBasicVelocityTypeIIColor.png "Input and output values of the \em velocity-based \em Type \em II \em On-Line \em Trajectory \em Generation algorithm."</li>
    </ul></li>
  </ul>        

<hr>
\section sec_TypeIVBrief Reflexxes Type IV Motion Library


 <ul>
<li>Freely available for research and education: 
  <a href="http://www.reflexxes.com/products/academic-version" target="_blank" title="http://www.reflexxes.com/products/academic-version"><b>http://www.reflexxes.com/products/academic-version</b></a>.</li>
  <li>Includes Type I, \ref sec_TypeIIBrief "Type II", and Type III</li>
  <li>Position-based On-Line Trajectory Generation (OTG) algorithm
   <ul>   
    <li>Jerk limitation</li>
    <li>Time- and phase-synchronization</li>
    <li>Time-optimal trajectories</li>
    <li>The desired target state of motion at the end of each motion
        trajectory consists of a desired position/pose/vector and a desired
        velocity vector.</li>
    <li>Input and output values:
        \image html RMLBasicPositionTypeIVColor.png "Input and output values of the \em position-based \em Type \em IV \em On-Line \em Trajectory \em Generation algorithm."</li>
  </ul></li>
  <li>Velocity-based On-Line Trajectory Generation (OTG) algorithm</li>
  <ul>       
     <li>Jerk limitation</li>
     <li>Time-optimal trajectories</li>
     <li>Input and output values:
        \image html RMLBasicVelocityTypeIVColor.png "Input and output values of the \em velocity-based \em Type \em IV \em On-Line \em Trajectory \em Generation algorithm."</li>
  </ul></li>
 </ul>        
 
\n
 
<hr>
\n

Further types are being developed and will be available in the future. 
An overview about all types of On-Line Trajectory Generation algorithms can
be found in\n
\n
<b>T. Kroeger.\n
On-Line Trajectory Generation in Robotic Systems.\n
Springer Tracts in Advanced Robotics, Vol. 58, Springer, January 2010.\n
<a href="http://www.springer.com/978-3-642-05174-6" target="_blank" title="You may order your personal copy at www.springer.com.">http://www.springer.com/978-3-642-05174-6</a></b>.\n\n


This \ref index "on-line documentation" entirely describes the
implementation of the <em>Type II Reflexxes Motion Library</em>.


*/

