//  ---------------------- Doxygen info ----------------------
//! \file Doxygen_TypeII_GettingStarted.h
//!
//! \brief
//! Documentation file for Doxygen (getting started)
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
//    For a convenient reading of this file's source code,
//    please use a tab width of four characters.
//  ----------------------------------------------------------



// -----------------------------------------------------------------
/*!
\page page_GettingStarted Getting Started &mdash; Simple Examples

\n

\section sec_DownloadReflexxesMotionLibrary Download the Type II Reflexxes Motion Library

You can freely \ref page_DownloadInstructions "download" the
\ref page_ReflexxesMotionLibraries "Type II Reflexxes Motion Library"
featuring its <b>entire set of functionalities without limitations</b>.\n\n

The provided files contain six simple <b>ready-to-compile sample
applications</b> that were written in order to demonstrate the clear and
simple API of ReflexxesAPI (two methods only).
If you have not downloaded the Reflexxes Motion Libraries yet, please refer
to the \ref page_DownloadInstructions "download instructions".

To get started without a any big effort, a set of basic sample applications
has been prepared, all of which are fully documented. The provided source
code of these samples is meant to be very simple and self-explaining.

<ul>
    <li>\ref page_Code_01_RMLPositionSampleApplication\n
        A first application using the position-based Online Trajectory
        Generation (OTG) algorithm:
        <ul>
            <li>Setting-up input values RMLPositionInputParameters and
            <li>calling the algorithm ReflexxesAPI::RMLPosition().\n\n
        </ul>
    </li>
    <li>\ref page_Code_02_RMLPositionSampleApplication\n
        The same as <tt>01_RMLPositionSampleApplication.cpp</tt> but in addition,
        the entire set of output values of the algorithm are used and described:
        <ul>
            <li>making use of all output values RMLPositionOutputParameters.\n\n</li>
        </ul>
    </li>
    <li>\ref page_Code_03_RMLPositionSampleApplication\n
        This sample application shows the difference between time- and
        phase-synchronized motion trajectories and how they can be
        specified (cf. \ref page_SynchronizationBehavior):
        <ul>
            <li>making use of the flag RMLFlags::SynchronizationBehavior.\n\n</li>
        </ul>
    </li>
    <li>\ref page_Code_04_RMLVelocitySampleApplication\n
        Similar to <tt>01_RMLPositionSampleApplication.cpp</tt>; a first
        application using the velocity-based Online Trajectory Generation
        algorithm:
        <ul>
            <li>Setting-up input values RMLVelocityInputParameters and</li>
            <li>calling the algorithm ReflexxesAPI::RMLVelocity().\n\n</li>
        </ul>
    </li>
    <li>\ref page_Code_05_RMLVelocitySampleApplication\n
        Similar to <tt>02_RMLPositionSampleApplication.cpp</tt>; the same as 
        <tt>04_RMLVelocitySampleApplication.cpp</tt> but in addition, the entire
        set of output values of the algorithm are used and described:
        <ul>
            <li>making use of all output values RMLVelocityOutputParameters.\n\n</li>
        </ul>
    </li>
    <li>\ref page_Code_06_RMLVelocitySampleApplication\n
        Similar to <tt>04_RMLVelocitySampleApplication.cpp</tt>, but here,
        a time-synchronized trajectory is generated:
        <ul>
            <li>making use of the flag RMLFlags::SynchronizationBehavior.\n\n</li>
        </ul>
    </li>
    <li>\ref page_Code_07_RMLPositionSampleApplication\n
        The same as \ref page_Code_02_RMLPositionSampleApplication "Example 2" but in addition,
        <ul>
            <li>the optional input value
            RMLInputParameters::MinimumSynchronizationTime is used and
            described.\n\n</li>
        </ul>
    </li>
    <li>\ref page_Code_08_RMLVelocitySampleApplication\n
        The same as \ref page_Code_05_RMLVelocitySampleApplication "Example 5" but in addition,
        <ul>
            <li>the optional input value
            RMLInputParameters::MinimumSynchronizationTime is used and
            described.\n\n</li>
        </ul>
    </li>
</ul>

All resulting trajectories are shown together with the source code files.
Within a source code file, you may click on the links of the used
data structures to learn more about the Reflexxes API.


*/

