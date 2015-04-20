//  ---------------------- Doxygen info ----------------------
//! \file Doxygen_TypeII_InputValues.h
//!
//! \brief
//! Documentation file for Doxygen (detailed description of input values)
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
\page page_InputValues Description of Input Values


The input values of the position-based On-Line Trajectory Generation (OTG)
algorithm are set-up through the classes RMLPositionInputParameters
and RMLPositionFlags, and the input values of the velocity-based
On-Line Trajectory Generation algorithm are set-up through the classes
RMLVelocityInputParameters and RMLVelocityFlags. For a detailed description
of these classes, please refer to the class documentation, and for a
description of the output values, please refer to the \ref page_OutputValues.\n
\n
This page contains three sections:\n\n

 -# \ref sec_InputValuesPosition\n\n
 -# \ref sec_InputValuesVelocity\n\n
 -# \ref sec_NumericalStability\n\n
 
<hr>
\section sec_InputValuesPosition Input Values for the Position-based On-Line Trajectory Generation Algorithm

\n\n

\image html RMLBasicPositionTypeIIColor.png "Input and output values of the position-based \em Type \em II On-Line Trajectory Generation algorithm of the \em Reflexxes \em Motion \em Libraries."

The <em>position-based Type II On-Line Trajectory Generation
algorithm</em> is executed by a call of ReflexxesAPI::RMLPosition(). The
input values for the algorithm at a time instant \f$ T_i \f$ are 
contained in RMLPositionInputParameters:\n\n

<ul>
    <li>the current state of motion \f$ {\bf M}_i \f$ consisting of\n\n
        <ul>
            <li>the current position/pose vector \f$ \vec{P}_i \ \Longrightarrow \  \f$
                RMLPositionInputParameters::CurrentPositionVector and\n\n</li>
            <li>the current velocity vector \f$ \vec{V}_i \ \Longrightarrow \  \f$
            RMLPositionInputParameters::CurrentVelocityVector,\n\n</li>
        </ul></li>
    <li>the kinematic motion constraints \f$ {\bf B}_i \ \Longrightarrow \  \f$ consisting of\n\n
        <ul>
            <li>the current maximum velocity vector \f$ \vec{V}_i^{\,max} \ \Longrightarrow \  \f$
                RMLPositionInputParameters::MaxVelocityVector and\n\n</li>
            <li>the current maximum acceleration vector \f$ \vec{A}_i^{\,max} \ \Longrightarrow \  \f$
                RMLPositionInputParameters::MaxAccelerationVector,\n\n</li>
        </ul>
    </li>
    <li>the target state of motion \f$ {\bf M}_i^{\,trgt} \ \Longrightarrow \  \f$ consisting of\n\n
        <ul>
            <li>the target position/pose vector \f$ \vec{P}_i^{\,trgt} \ \Longrightarrow \  \f$
                RMLPositionInputParameters::TargetPositionVector and\n\n</li>
            <li>the target velocity vector \f$ \vec{V}_i^{\,trgt} \ \Longrightarrow \  \f$
                RMLPositionInputParameters::TargetVelocityVector,\n\n</li>
        </ul>
    </li>
    <li>the boolean selection vector \f$ \vec{S}_i \ \Longrightarrow \  \f$
        RMLPositionInputParameters::SelectionVector, and\n\n</li>
    <li>an \b optional value for the minimum synchronization time 
        \f$ \vec{S}_i \ \Longrightarrow \  \f$
       RMLPositionInputParameters::MinimumSynchronizationTime.</li> 
</ul>

Each vector contains the values for all degrees of freedom
\f$ 1,\,\dots,\,K \f$, for instance,

\f[ \vec{P}_i\ =\ \left(\,\!_1P_i,\,\dots,\,_kP_i,\,\dots,\,_KP_i\right)^T \f]

such that the current position value of the degree of freedom with the 
index \f$ k \f$ is represented by \f$ _kP_i \f$.\n\n

The selection vector \f$ \vec{S}_i \f$ can be used to mask degrees of 
freedom individually. If the value of \f$ _kS_i \f$ is \c false,
the degree of freedom with the index \f$ k \f$ will not be considered by
the algorithm.\n\n

Besides the input values RMLPositionInputParameters, the class
RMLPositionFlags can be used to determine and obtain a certain behavior.
The following flags may be used:\n\n

 - RMLPositionFlags::SynchronizationBehavior,\n\n
 - RMLPositionFlags::EnableTheCalculationOfTheExtremumMotionStates, and\n\n
 - RMLPositionFlags::KeepCurrentVelocityInCaseOfFallbackStrategy.\n\n

A first and simple example that shows, how the position-based input values 
are commonly set-up, please refer to the
\ref page_Code_01_RMLPositionSampleApplication. A description of the output
values of this example can be found at the page \ref page_OutputValues.
Please also refer to the Section \ref sec_NumericalStability, which
describes the input domains of the algorithm.

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

\n

\sa RMLPositionInputParameters
\sa RMLPositionFlags
\sa \ref page_OutputValues

\n

<hr>

\section sec_InputValuesVelocity Input Values for the Velocity-based On-Line Trajectory Generation Algorithm

\n\n

\image html RMLBasicVelocityTypeIIColor.png "Input and output values of the velocity-based \em Type \em II On-Line Trajectory Generation algorithm of the \em Reflexxes \em Motion \em Libraries."

The <em>velocity-based Type II On-Line Trajectory Generation
algorithm</em> is executed by a call of ReflexxesAPI::RMLVelocity(), and
its input values are the very same as for the position-based algorithm 
(see above), but the values for the target position
\f$ _kP_i^{\,trgt} \f$ and the maximum velocity vector 
\f$ \vec{V}_i^{\,max} \f$ are not considered, and the flag
RMLPositionFlags::KeepCurrentVelocityInCaseOfFallbackStrategy is not
available.\n
\n
The input values for the velocity-based algorithm at a time
instant \f$ T_i \f$ are contained in RMLVelocityInputParameters:\n\n

<ul>
    <li>the current state of motion \f$ {\bf M}_i \f$ consisting of\n\n
        <ul>
            <li>the current position/pose vector \f$ \vec{P}_i \ \Longrightarrow \  \f$
                RMLVelocityInputParameters::CurrentPositionVector and\n\n</li>
            <li>the current velocity vector \f$ \vec{V}_i \ \Longrightarrow \  \f$
                RMLVelocityInputParameters::CurrentVelocityVector,\n\n</li>
        </ul></li>
    <li>the kinematic motion constraints \f$ {\bf B}_i \ \Longrightarrow \  \f$ consisting of\n\n
        <ul>
            <li>the current maximum acceleration vector \f$ \vec{A}_i^{\,max} \ \Longrightarrow \  \f$
                RMLVelocityInputParameters::MaxAccelerationVector,\n\n</li>
        </ul></li>
    <li>the target state of motion \f$ {\bf M}_i^{\,trgt} \ \Longrightarrow \  \f$ consisting of\n\n
        <ul>
            <li>the target velocity vector \f$ \vec{V}_i^{\,trgt} \ \Longrightarrow \  \f$
                RMLPositionInputParameters::TargetVelocityVector,\n\n</li>
        </ul></li>
    <li>the boolean selection vector \f$ \vec{S}_i \ \Longrightarrow \  \f$
        RMLVelocityInputParameters::SelectionVector, and\n\n
        an \b optional value for the minimum synchronization time 
        \f$ \vec{S}_i \ \Longrightarrow \  \f$
        RMLVelocityInputParameters::MinimumSynchronizationTime.\n\n</li>
</ul>


Besides the input values RMLVelocityInputParameters, the class
RMLVelocityFlags can be used to determine and obtain a certain behavior.
The following flags may be used:\n\n

 - RMLVelocityFlags::SynchronizationBehavior and\n\n
 - RMLVelocityFlags::EnableTheCalculationOfTheExtremumMotionStates.\n\n

A first and simple example that shows, how the velocity-based input values 
are commonly set-up, please refer to the
\ref page_Code_04_RMLVelocitySampleApplication. A description of the output
values of this example can be found at the page \ref page_OutputValues.
Please also refer to the Section \ref sec_NumericalStability, which
describes the input domains of the algorithm.

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

\n
   
\note
The velocity-based algorithm is also used in the second layer of the
safety mechanism to guarantee valid output value for any given set of
input values (cf. \ref page_ErrorHandling).

\n

\sa RMLVelocityInputParameters
\sa RMLVelocityFlags
\sa \ref page_OutputValues

\n

<hr>

\section sec_NumericalStability Input Requirements for Numerical Stability

In order to assure feasibility and numerical stability, <b>two basic
conditions</b> have to be fulfilled for each selected degree of freedom
\f$ k \f$ in order to obtain correct output values:\n\n

<ol>
    <li>The maximum velocity \f$ _kV_i^{\,max} \f$ and the maximum acceleration
        \f$ _kA_i^{\,max} \f$ must be greater than zero:\n\n
        <ul>
            <li>\f$ _kV_i^{\,max} \ > \ 0 \f$\n\n</li>
            <li>\f$ _kA_i^{\,max} \ > \ 0 \f$\n\n\n</li>
        </ul>
    </li>
    <li>For the <b>position-based</b> algorithm, the elements of the maximum velocity
        vector \f$ \vec{V}_i^{\,max} \f$ has to be greater or equal to the 
        elements of the target velocity \f$ \vec{V}_i^{\,trgt} \f$.
        \f[ _kV_i^{\,trgt}\ \le \ _kV_i^{\,max}\ \forall\ \left\{1,\,\dots,\,K\right\} \f]        
        \n\n</li>
    <li>All input values have to be
        within a certain range of magnitude. For the <b>position-based</b>
        algorithm this is range is specified by
        \f[ \frac{\displaystyle \mbox{max}\left(
        \,\!_kA_i^{\,max},
        \, \,\!_kV_i^{\,max},
        \, \left|\,\!_kP_i\right|,
        \, \left|\,\!_kP_i^{\,trgt}\right|,
        \, \left|\,\!_kV_i\right|
        \right)}{\displaystyle 
        \mbox{min}\left(\,\!_kA_i^{\,max},
        \, \,\!_kV_i^{\,max} \right)}
        \ \le\ 10^8 \f]
        and for the <b>velocity-based</b> algorithm
        \f[ \frac{\displaystyle \mbox{max}\left(
        \,\!_kA_i^{\,max},
        \, \left|\,\!_kP_i\right|,
        \, \left|\,\!_kV_i^{\,trgt}\right|,
        \, \left|\,\!_kV_i\right| \right)}{\displaystyle 
        \mbox{min}\left(\,\!_kA_i^{\,max} \right)}
        \ \le\ 10^{10} \f]
    </li>
</ol>

\note
<ol>
    <li>Although this condition looks like a limitation, in real-world 
        systems it does not lead to a disadvantage, because the range
        of <b>eight orders of magnitude</b> should be very sufficient for
        any application. As the velocity-based algorithm is much simpler
        from a numerical point of view, <b>ten orders of magnitude</b>
        are allowed here.\n\n
    </li>
    <li>If this condition is \em not fulfilled, the Type II Reflexxes Motion 
        Library will try to provide correct output values, and in many cases
        this will be possible, but no guarantee can be given. Valid output
        values that lead to steady, continuous and jerk-limited motion
        trajectories are guaranteed in any case (cf.
        \ref page_ErrorHandling).\n\n
    </li>
</ol>
    
\n\n<em>Example for a set of \b valid input values for one degree of
freedom \f$ k \f$ at time instant \f$ T_i \f$ used for the
position-based algorithm:</em>\n\n

\f[ \begin{array}{rcl}
_kV_i^{\,max}&=&10000000\\[2ex]
_kA_i^{\,max}&=&500\\[2ex]
_kP_i&=&25\\[2ex]
_kV_i&=&-35\\[2ex]
_kP_i^{\,trgt}&=&5\\[2ex]
_kV_i^{\,trgt}&=&5000\\[2ex]
\end{array} \f]

Reason: \f$ \frac{\displaystyle _kV_i^{\,max}}{\displaystyle _kP_i^{\,trgt}} \ =\ \frac{\displaystyle 10^7}{\displaystyle 5}\ =\ 2\cdot10^6\ \le 10^8 \f$.

\n\n<em>Example for a set of \b invalid input values for one degree of
freedom \f$ k \f$ at time instant \f$ T_i \f$ used for the
position-based algorithm:</em>\n\n

\f[ \begin{array}{rcl}
_kV_i^{\,max}&=&10000000\\[2ex]
_kA_i^{\,max}&=&0.001\\[2ex]
_kP_i&=&25\\[2ex]
_kV_i&=&-35\\[2ex]
_kP_i^{\,trgt}&=&5\\[2ex]
_kV_i^{\,trgt}&=&5000\\[2ex]
\end{array} \f]

Reason: \f$ \frac{\displaystyle _kV_i^{\,max}}{\displaystyle _kA_i^{\,max}} \ =\ \frac{\displaystyle 10^7}{\displaystyle 10^{-3}}\ =\ 10^{10}\ \not\leq 10^8 \f$.\n\n


The check, whether the input values are valid, can be done with the methods
RMLPositionInputParameters::CheckForValidity() and
RMLVelocityInputParameters::CheckForValidity().



*/

