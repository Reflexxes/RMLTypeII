//  ---------------------- Doxygen info ----------------------
//! \file Doxygen_TypeII_FinalStateReachedBehavior.h
//!
//! \brief
//! Documentation file for Doxygen (description of the behavior after the
//! final state of motion is reached)
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


\page page_FinalStateReachedBehavior Behavior after the Final State of Motion is Reached

In order to determine the behavior of the trajectory generator after the
final state of motion is reached, the flag
<c>RMLPositionFlags::BehaviorAfterFinalStateOfMotionIsReached</c> can be
used. This flag can contain two different values:

-#  <c>RMLPositionFlags::KEEP_TARGET_VELOCITY</c> (default)
-#  <c>RMLPositionFlags::RECOMPUTE_TRAJECTORY</c>.

The following example has been generated with the Type IV Reflexxes Motion
Library and shows the difference between both behaviors.

\n\n

\image html BehaviorAfterFinalStateIsReachedValues.png " "

\n

\image html BehaviorAfterFinalStateIsReached.png " "

\n\n

The left diagram shows the behavior if
<c>RMLPositionFlags::BehaviorAfterFinalStateOfMotionIsReached</c> is set to
<c>RMLPositionFlags::RECOMPUTE_TRAJECTORY</c>, and the right diagram 
shows the case of <c>RMLPositionFlags::RECOMPUTE_TRAJECTORY</c>. In the
first case, the desired target velocity is kept until the input values 
change, and the method ReflexxesAPI::RMLPosition() keeps returning
<c>ReflexxesAPI::RML_FINAL_STATE_REACHED</c>. In the second case,
ReflexxesAPI::RMLPosition() will return 
<c>ReflexxesAPI::RML_FINAL_STATE_REACHED</c> only once (in this example
in the control cycle at \f$ t\ =\ 3075\,ms\f$), and in the subsequent
control cycle, a new trajectory to reach the same state of motion again
is computed, which will then be reached in the control cycle at
\f$ t\ =\ 6190\,ms\f$ (and again at \f$ t\ =\ 9305\,ms, 12420\,ms, 15535\,ms\f$,
etc.). This behavior repeats until the input values are changed.\n

The position diagram (\f$ \vec{P}_i(t)\ =\ \left(\,\!_1p_i(t),\,_2p_i(t)\right) \f$)
also contains the positional extreme values depicted in darker colors (cf. 
RMLFlags::EnableTheCalculationOfTheExtremumMotionStates).\n


In this example, a time-synchronized trajectory was used. At page
\ref page_PSIfPossible, the same example with a phase-synchronized motion
is shown. \n\n



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

