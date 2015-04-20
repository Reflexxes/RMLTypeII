//  ---------------------- Doxygen info ----------------------
//! \file Doxygen_TypeII_ErrorHandling.h
//!
//! \brief
//! Documentation file for Doxygen (description of error-handling
//! procedures and the Reflexxes Safety Concept)
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
\page page_ErrorHandling Safety: Three Layers of Error Handling 

If one the methods\n\n

 - ReflexxesAPI::RMLPosition() or\n\n
 - ReflexxesAPI::RMLVelocity()\n\n

is called with valid input values, it is <b>guaranteed</b> that the output values of the
Reflexxes algorithms lead to a <b>continuous acceleration-limited motion trajectory
in any case</b>. This fundamental feature was implemented using a
three-layered approach of error-handling, which is described in the
following two sections.\n\n

 -# \ref sec_ErrorHandlingPositionBased\n\n
 -# \ref sec_ErrorHandlingVelocityBased\n\n
 
 \n

<hr>


\section sec_ErrorHandlingPositionBased Error-Handling for the Position-based On-Line Trajectory Generation Algorithm

The position-based Reflexxes Type II On-Line Trajectory Generation (OTG)
algorithm is the most complex algorithm of the Reflexxes Motion Libraries.
Robustness and numerical stability are key features to assure deterministic
and safe behaviors of robots and mechanical systems. After more than eight
years of research and development, the released algorithm runs robust and 
is numerically stable.

In order to guarantee continuous acceleration-limited
motion trajectories, three layers of error-handling have been implemented
in order to obtain a deterministic and safe behavior of all Reflexxes 
Motion Libraries for any set given input values.

\n

\image html ThreeSafetyLayers.png "Three layers of safety and error-handling for the position-based On-Line Trajectory Generation algorithm."

\n

\subsection ssec_Layer3Position Layer 3

This layer contains the position-based Reflexxes Type II On-Line Trajectory
Generation algorithm. This algorithm is called through the API method
ReflexxesAPI::RMLPosition(), which calls the internal method
TypeIIRMLPosition::GetNextStateOfMotion() that contains the actual
position-based trajectory generation algorithm. If the algorithm is fed with
valid input values, that is, the method
RMLPositionInputParameters::CheckForValidity() returns \c false, we switch
from <em>Layer 3</em> to <em>Layer 2</em> (even after after more than
\f$ 10^{11} \f$ cylces, this has not happened yet).

The implementation of <em>Layer 2</em> can be found in the method
TypeIIRMLPosition::FallBackStrategy().

\subsection ssec_Layer2Position Layer 2

In the second layer, the velocity-based algorithm of the Reflexxes Type II
Motion Library is executed in order to safely
transfer the system to a predefined velocity
(cf. ReflexxesAPI::RMLVelocity()). The default value for this alternative
desired velocity vector \f$ \vec{V}_{i}^{\,\underline{trgt}} \f$ is zero,
that is, the robot or mechanical system would simply slow down to
zero-velocity by using the currently given maximum acceleration vector
\f$ \vec{A}_i^{\,max} \f$ (cf.
RMLPositionInputParameters::MaxAccelerationVector) until
\f$ \vec{V}_{i}^{\,\underline{trgt}}\ =\ \vec{0}\f$ is reached. In order to
decouple all selected degrees of freedom and to reach this sate safely,
a non-synchronized motion will be computed, and the flag
RMLFlags::NO_SYNCHRONIZATION will be used for the velocity-based 
algorithm.

If it is not desired to decrease the velocity to zero but to continue
moving the system with a certain application- or task-dependent velocity,
an alternative desired target velocity vector
\f$ \vec{V}_{i}^{\,\underline{trgt}} \f$ can be defined. The attribute
RMLPositionInputParameters::AlternativeTargetVelocityVector can be used to
specify this alternative velocity vector unequal to zero,
\f$ \vec{V}_{i}^{\,\underline{trgt}}\ \neq\ \vec{0}\f$. Furthermore,
the input flag RMLPositionFlags::KeepCurrentVelocityInCaseOfFallbackStrategy
can be used to set the alternative desired target velocity vector to the
current velocity vector, that is,
\f$ \vec{V}_{i}^{\,\underline{trgt}}\ =\ \vec{V}_i\f$.

The mathematical equations of the velocity-based On-Line Trajectory
Generation only contain closed-form solutions; numerical instabilities
cannot occur if the \ref sec_NumericalStability are met.
In case, the user specifies elements of the maximum
acceleration vector \f$ \vec{A}_i^{\,max} \f$ that are equal or less than
zero (cf. RMLPositionInputParameters::MaxAccelerationVector), the algorithm
will even switch from <em>Layer 2</em> to <em>Layer 1</em> in order to
continue the current motion. This layer is implemented in the method
TypeIIRMLVelocity::FallBackStrategy().


\subsection ssec_Layer1Position Layer 1
Although even the probability of switching from <em>Layer 3</em> to 
<em>Layer 2</em> can only happen in case of incorrect input
values, a third layer is used to handle this case. In this case, the
motion of the system is continued with zero-acceleration. That means the
output values RMLPositionOutputParameters determined by the following
equations:

\f[
\begin{array}{rrcl}
\forall\ k\ \in\ \left\{1,\,\dots,\,K\right\}:\ \ \ \ 
&_{k}P_{i+1}&\ =\ &_{k}P_{i}\ +\ _{k}V_{i}\,T^{\,cycle}\\[3ex]
&_{k}V_{i+1}&\ =\ &_{k}V_{i}\\[3ex]
\end{array}
\f]
Where the variables and symbols represent the following data:\n\n

 - \f$ K \f$ is the number of degrees of freedom of the system\n\n
 - \f$ T^{\,cycle} \f$ is the cycle time given in seconds\n\n
 - \f$ _{k}P_{i} \f$ is the position of the degree of freedom \f$ k \f$ at
   time instant \f$ T_i \f$\n\n
 - \f$ _{k}V_{i} \f$ is the velocity of the degree of freedom \f$ k \f$ at
   time instant \f$ T_i \f$\n\n
 - \f$ _{k}P_{i+1} \f$ is the position of the degree of freedom \f$ k \f$ at
   time instant \f$ T_{i+1} \f$\n\n
 - \f$ _{k}V_{i+1} \f$ is the velocity of the degree of freedom \f$ k \f$ at
   time instant \f$ T_{i+1} \f$\n\n
 - The time difference from one control cycle at \f$ T_{i} \f$ to the next
   control cycle at \f$ T_{i+1} \f$ is \f$ T^{\,cycle} \f$, that is
   \f$ T_{i+1}\ -\ T_{i}\ =\ T^{\,cycle} \f$.\n\n

Details about the implementation of these equations can be found in the
method TypeIIRMLVelocity::FallBackStrategy().

\n

\subsection ssec_RemarkPhaseSynchronizationPosition Remark on Phase-Synchronized Trajectories

If a phase-synchronized motion trajectory is required (i.e., the input flag
RMLPositionFlags::ONLY_PHASE_SYNCHRONIZATION is set), and if phase-synchronisation
is not possible because the input vectors are not collinear, we also
switch from <em>Layer 3</em> to <em>Layer 2</em>, because in this case,
priority is given to the fact that phase-synchronization is required
(otherwise, the flag RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE
may be used). If we switch to the second layer in this case, the algorithm
will check, whether a phase-synchronous solution is existent if the
velocity-based On-Line Trajectory Generation is executed. If so, the
velocity-based algorithm will compute a phase-synchronized motion.
Otherwise, we will switch from <em>Layer 2</em> to <em>Layer 1</em>. If the
current state of motion is homothetic, the trajectory will be continued
homothetically (phase-synchronized) by applying zero-acceleration
in the current control cycle.


\n

<hr>

\section sec_ErrorHandlingVelocityBased Error-Handling for the Velocity-based On-Line Trajectory Generation Algorithm

The velocity-based Reflexxes Type II On-Line Trajectory Generation (OTG)
algorithm is of much simpler nature than the position-based algorithm and
only a two-layered method is used for error-handling. This safety mechanism
is directly derived from <em>Layer 2</em> and <em>Layer 1</em> of the
position-based On-Line Trajectory Generation algorithm.


\n

\image html ThreeSafetyLayersVelocity.png "Two layers of safety and error-handling for the velocity-based On-Line Trajectory Generation algorithm."

\n

The algorithm is called through the API method ReflexxesAPI::RMLVelocity(),
which executes the internal method
TypeIIRMLVelocity::GetNextStateOfMotion(). If the input values for this
algorithm are valid, that is, the result of
RMLVelocityInputParameters::CheckForValidity() is \c true, the velocity-based
algorithm will always find the correct solution. If
RMLVelocityInputParameters::CheckForValidity() returns \c false, because
elements of the maximum acceleration vector \f$ \vec{A}_i^{\,max} \f$
that or equal or less than zero (cf.
RMLVelocityInputParameters::MaxAccelerationVector and
RMLVelocityInputParameters::MaxJerkVector), the algorithm will switch
from <em>Layer 2</em> to <em>Layer 1</em> in order to continue the
current motion. This layer is implemented in the method
TypeIIRMLVelocity::FallBackStrategy(). Please refer to Sec.
\ref ssec_Layer1Position of the position-based algorithm for a detailed
description.

\n

\subsection ssec_RemarkPhaseSynchronizationVelocity Remark on Phase-Synchronized Trajectories
If a phase-synchronized motion trajectory is required (i.e., the input flag
RMLVelocityFlags::ONLY_PHASE_SYNCHRONIZATION is set), and if phase-synchronisation
is not possible because the input vectors are not collinear, we also
apply the switching mechanism described above (i.e., the new state of
motion is computed by applying a zero-acceleration). If the current state of motion
is homothetic, the trajectory will be continued homothetically
(phase-synchronized).

\n\n

*/

