//  ---------------------- Doxygen info ----------------------
//! \file Doxygen_TypeII_BriefOverview.h
//!
//! \brief
//! Documentation file for Doxygen (a brief overview about the algorithmic
//! concept)
//! \n
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
\page page_TypeIIAndIVOverview The Position-based Type II On-Line Trajectory Generation Algorithm


This page contains four sections\n\n

<ul>
<li>\ref sec_BriefIntroduction\n\n</li>
<li>\ref sec_Step1\n\n</li>
<li>\ref sec_Step2\n\n</li>
<li>\ref sec_Step3\n\n</li>
</ul>

<hr>

\section sec_BriefIntroduction A Brief Introduction

\image html RMLBasicPositionTypeIIColor.png "Input and output values of the \em Reflexxes \em Type \em II \em Motion \em Library in a generic manner."

The interface (ReflexxesAPI) of all \em Reflexxes \em Motion \em Libraries
is very simple and can easily be integrated into existing systems. Based on
the \em current \em state \em of \em motion and the \em kinematic \em motion
\em constraints, a \em new \em state \em of \em motion is calculated with
lies exactly on the time-optimal trajectory to reach the desired \em target
\em state \em of \em motion. All input values can change arbitrarily based
on sensor signals and even discontinuously, and a steady jerk-limited motion
trajectory is always \em guaranteed at the output
The above figure shows the input and output values of the RML
algorithm in a generic manner. It is the task
of the algorithm to time-optimally transfer an arbitrary current state of
motion\n
\n
\f[ {\bf M}_{i}\,=\,\left(\vec{P}_{i},\, \vec{V}_{i},\, \vec{A}_{i} \right) \f]
\n
into the desired target state of motion\n
\n
\f[ {\bf M}_{i}^{\,trgt}\,=\,\left(\vec{P}_{i}^{\,trgt},\, \vec{V}_{i}^{\,trgt},\, \vec{0} \right) \f]
\n
under consideration of the kinematic motion constraints\n
\n
\f[ {\bf B}_{i}\,=\,\left( \vec{V}_{i}^{\,max},\, \vec{A}_{i}^{\,max},\, \vec{J}_{i}^{\,max} \right) \f]\n
\n
The algorithm works memoryless and calculates only the
next state of motion \f$ {\bf M}_{i+1} \f$, which is used as input value
for lower-level motion controllers. The resulting trajectories are
time-optimal and synchronized, such that all selected DOFs coinstantaneously
reach their target state of motion. The selection vector \f$ \vec{S}_i \f$
contains boolean values to mask single DOFs, for which no output values are
calculated. All types and variant of the algorithm consist of three steps,
which are introduced in the following.\n


\n
\n

For a better understanding of the basic algorithm for Type II On-Line 
Trajectory generation (OTG), this section presents a brief overview. For a
detailed description, please refer to\n
\n
<b>T. Kroeger.</b>\n
<b>On-Line Trajectory Generation in Robotic Systems.</b>\n
<b>Springer Tracts in Advanced Robotics, Vol. 58, Springer, January 2010.</b>\n
<b><a href="http://www.springer.com/978-3-642-05174-6" target="_blank" title="You may order your personal copy at www.springer.com.">http://www.springer.com/978-3-642-05174-6</a></b>\n\n

\n
\n

<hr>

\section sec_Step1 Step 1: Calculate the Synchronization Time

Although only one single scalar value is calculate in this step, it the
the most complex one. First, the minimum execution times
\f$ _kt_{i}^{\,min} \f$ are calculated for each selected DOF
\f$ k \ \in\ \{1,\dots,K\}\f$. The value of the minimum synchronization
time \f$ t_i^{\,sync} \f$ must be equal or greater than the minimum value
of all minimum execution times. To transfer the motion state of one DOF to
another, a finite set of motion profiles \f$ {\cal P}_{Step1} \f$ is
considered, and a decision tree selects a motion profile
\f$ _k\Psi_i^{\,Step1}\ \in\ {\cal P}_{Step1} \f$ for each selected DOF
\f$ k \ \in\ \{1,\dots,K\} \f$. To calculate \f$ _kt_{i}^{\,min} \f$, a
system of nonlinear equation is set-up, and the solution contains the
desired value.\n
\n
Depending on the type of the algorithm (I-IX), it may happen that up
to \f$ Z\,=\,3 \f$ time intervals are existent, within which the target
state of motion cannot be reached. For the Type II algorithm only 
\f$ Z\,=\,1 \f$ inoperative interval may be existent.
The decision trees 1B and 1C are used to
calculate all limits of these intervals \f$ _k{\cal Z}_{i} \f$, whose
elements are denoted by\n
\n
\f[ _k^z\zeta_{i}\,=\,\left[^z_kt_{i}^{\,begin},\,^z_kt_{i}^{\,end}\right],\ \mbox{with}\ z\ \in\ \left\{1,\,\dots,\,Z\right\} \f]\n
\n
Finally, the minimum time not being within any inoperative time interval\n
\n
\f[ _k^z\zeta_{i}\ \forall\ (z,\,k)\ \in \ \left\{1,\,\dots,\,Z\right\}\times\left\{1,\,\dots,\,K\right\} \f] \n
\n
is selected as the value for the synchronization time \f$ t_i^{\,sync} \f$.

\n
\n

<hr>

\section sec_Step2 Step 2: Synchronization of All Selected DOFs

All selected DOFs that did not determine \f$ t_i^{\,sync} \f$ have to be
synchronized to this time value. In principal, an infinite number of
solutions can be found to parameterize a trajectory that transfers such
a DOF from \f$ _k\vec{M}_i \f$ to \f$ _k\vec{M}_i^{\,trgt} \f$ in
\f$ t_i^{\,sync} \f$. In order to achieve a deterministic framework, an
optimization criterion must be used in order to compute a \em time- or a
\em phase-synchronized motion trajectory. Therefore, another decision tree
is used, which selects a motion profile \f$ _k\Psi_i^{\,Step2} \f$ for each
DOF \f$ k \f$ from a different set that is denoted by
\f$ {\cal P}_{Step2} \f$. Based on this profile, a nonlinear system of
equations can be solved, and the solution contains all required parameters
for the synchronized motion trajectory.

\n
\n

<hr>

\section sec_Step3 Step 3: Calculate Output Values

This last step is trivial: the resulting trajectory parameters of Step 2
are used to calculate a new state of motion \f$ {\bf M}_{i+1} \f$, which is
used as set-point for lower-level controllers at \f$ T_{i+1}\f$.\n


\image html GenericStructogram.png "Nassi-Shneiderman structogram of the basic OTG algorithm."

The figure above summarizes the generic version of the OTG algorithm for 
time-synchronization, while the figure below summarizes the version, in 
which phase-synchronization is applied.\n


\image html GenericStructogramPS.png "Generic Nassi-Shneiderman structogram for the OTG algorithm for phase-synchronized trajectories."

The class ReflexxesAPI constitutes the actual user interface and provides 
the complete set of functionalities while hiding all parts of the library
that are not required by the user. Users, who like to take a deeper look
to the implementation of the algorithm, may learn about the class 
TypeIIRMLPosition and the namespace TypeIIRMLMath, both of which contain
all mathematical details.


*/