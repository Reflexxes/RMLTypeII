//  ---------------------- Doxygen info ----------------------
//! \file Doxygen_TypeII_PhaseSynchronizationIfPossible.h
//!
//! \brief
//! Documentation file for Doxygen (description of the flag
//! <c>RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE</c>)
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

\page page_PSIfPossible About the Flag RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE

Time-synchronized trajectories can be generated from any arbitrary state
of motion, but to generate phase-synchronized trajectories, certain
conditions have to be fulfilled. The flag
<c>RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE</c>
allows user to \b always generate phase-synchronized trajectories if
possible. In order to demonstrate the behavior difference between the flags

-  <c>RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE</c> and
-  <c>RMLFlags::ONLY_TIME_SYNCHRONIZATION</c>

the following diagram shows the path and trajectories of two different
motions. The left one was generated using the flag
<c>RMLFlags::ONLY_TIME_SYNCHRONIZATION</c>, and the right one using the flag 
<c>RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE</c>. In both cases the
desired target state of motion
\f${\bf M}_0^{\,trgt}\,=\,\left(\vec{P}_{0}^{trgt},\,\vec{V}_{0}^{trgt},\,\vec{0}\right)\f$
is reached after 3074 milliseconds for the first time. Because the 
trajectory started from an arbitrary initial state of motion 
\f${\bf M}_0 \f$, only time- but no phase-synchronization is possible.
Also in both cases, the flag 
<c>RMLPositionFlags::RECOMPUTE_TRAJECTORY</c>
is used, such that after reaching \f${\bf M}_0^{\,trgt} \f$ at
\f$t\,=\,3075\,ms\f$, \f${\bf M}_{3075}^{\,trgt}\,=\,{\bf M}_0^{\,trgt}\f$
is the new desired target state of motion. In this case, however, a
phase-synchronized trajectory can be generated, because all input vectors
are collinear now. This can be seen in the right diagram. The left diagram
shows the corresponding time-synchronized trajectory.

In the two top diagrams, one can clearly recognize that the second part
of the phase-synchronized trajectory is a straight-line motion
instead of an eight-shaped path in the case of time-synchronization.

\n\n

\image html BehaviorAfterFinalStateIsReachedValues.png " "

\n

\image html PhaseSynchronizationIfPossible.png " "

\n\n

The position diagram (\f$ \vec{P}_i(t)\ =\ \left(\,\!_1p_i(t),\,_2p_i(t)\right) \f$)
also contains the positional extreme values depicted in darker colors (cf. 
RMLFlags::EnableTheCalculationOfTheExtremumMotionStates).\n

This example was generated using the same example as on page
\ref page_FinalStateReachedBehavior. Further information about the
different synchronization behaviors can be found on page
\ref page_SynchronizationBehavior.

\n\n

\note
<b>The examples above have been generated with the Type IV Reflexxes Motion
Library, which takes into account the values of</b>\n\n
<ul>
    <li>RMLPositionInputParameters::CurrentAccelerationVector
    (containing the the current acceleration vector \f$ \vec{A}_i\f$
      and</li>
    <li>RMLPositionInputParameters::MaxJerkVector
    (containing the maximum jerk vector \f$ \vec{J}_i^{\,max} \f$.
      \n\n</li>
</ul>

\n

*/

