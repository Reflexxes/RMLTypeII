//  ---------------------<li>Doxygen info ----------------------
//! \file Doxygen_TypeII_DownloadInstructions.h
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
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------



// -----------------------------------------------------------------
/*!
\if PUBLIC

\page page_DownloadInstructions Download and Build Instructions for the Type II Reflexxes Motion Library

The Reflexxes Motion Libraries are OS independent and can, for instance, be used
on the following operating systems (32-bit and 64-bit)

<ul>
<li><b>Linux</b>,</li>
<li><b>Windows</b>,</li>
<li><b>MacOS X</b>,</li>
<li><b>QNX Neutrino</b>, and</li>
<li><b>VxWorks</b>.</li>
</ul>

Ready-to-use sample make files are provided for <em>Linux</em> and <em>Mac
OS X</em>, and a <em>Microsoft Windows Visual Studio 2008 Express</em> 
solution file are provided to get started immediately.
Links for further build instructions:\n\n

<ul>
<li>\ref sec_BuildingTheLibraryForLinux\n\n</li>
<li>\ref sec_BuildingTheLibraryForMacOSX\n\n</li>
<li>\ref sec_BuildingTheLibraryForMSWindows\n\n</li>
<li>\ref sec_ContentsOfTheDownloadedFile\n\n</li>
</ul>

The library is released as <b>open source software</b> under the 
<a href="http://www.gnu.org/licenses/lgpl.html" target="_blank" ><b>GNU Lesser General Public License</b></a>.
For purposes beyond this license, please
<a href="http://www.reflexxes.com/company/contact" title="http://www.reflexxes.com/company/contact" target="_blank" ><b>contact us</b></a>.

\n\n\n

<hr>

\section sec_BuildingTheLibraryForLinux Building the Library for Linux

Change to the directory <tt>ReflexxesTypeII/Linux</tt> and enter
\code
make clean32 all32
\endcode
for 32-bit systems or
\code
make clean64 all64
\endcode
for 64-bit systems, respectively to check whether all files
compile correctly on your system. If so, six simple 
\ref page_GettingStarted "ready-to-compile sample applications"
are provided to learn about the \ref page_ImportantClasses "Reflexxes API"
and to use it for your own applications.\n\n

To get an overview about the contents of the downloaded file, please refer
to \ref sec_ContentsOfTheDownloadedFile "the section at the bottom of this page".

\n\n\n

<hr>

\section sec_BuildingTheLibraryForMacOSX Building the Library for Mac OS X


Change to the directory <tt>ReflexxesTypeII/MacOS</tt> and enter
\code
make clean32 all32
\endcode
for 32-bit systems or
\code
make clean64 all64
\endcode
for 64-bit systems, respectively to check whether all files
compile correctly on your system. If so, six simple 
\ref page_GettingStarted "ready-to-compile sample applications"
are provided to learn about the \ref page_ImportantClasses "Reflexxes API"
and to use it for your own applications.\n\n

To get an overview about the contents of the downloaded file, please refer
to \ref sec_ContentsOfTheDownloadedFile "the section at the bottom of this page".

\n\n\n

<hr>

\section sec_BuildingTheLibraryForMSWindows Building the Library for Microsoft Windows

Open the Visual Studio solution file
\code
ReflexxesTypeII\Windows\ReflexxesTypeII_ExampleProject.sln
\endcode
with your
<a href="http://www.microsoft.com/visualstudio" target="_blank" title="" ><b>Microsoft Visual Studio</b></a>
development environment. You may rebuild the entire solution to check
whether all files compile correctly on your system. If so, six simple 
\ref page_GettingStarted "ready-to-compile sample applications"
are provided to learn about the \ref page_ImportantClasses "Reflexxes API"
and to use it for your own applications.\n\n

To get an overview about the contents of the downloaded file, please refer
to \ref sec_ContentsOfTheDownloadedFile "the section at the bottom of this page".

\n\n\n


<hr>

\section sec_ContentsOfTheDownloadedFile Contents of the Downloaded File

The downloaded archive contains the following subfolders:

<ul>
<li><b>include</b>: header files</li>
<li><b>src</b>: C++ source files
	<ul>
		<li><b>TypeIIRML</b>: \ref page_SourceCode  "Source code of the Type II Reflexxes Motion Library"</li>
		<li><b>RMLPositionSampleApplications</b>: \ref page_GettingStarted "Examples"</li>
		<li><b>RMLVelocitySampleApplications</b>: \ref page_GettingStarted "Examples"</li>
	</ul></li>
<li><b>Linux</b>: example makefiles for Linux</li>
<li><b>MacOS</b>: example makefiles for Mac OS X</li>
<li><b>Windows</b>: example project files for Microsoft Windows (Visual Studio 2008 Express)</li>
</ul>

Select the operating system of your choice. The software is designed to be
easily integrated in your own individual build system. The provided
makefiles and project files for Linux, Max OS X, and Windows systems allow
an overview of the build process.

In case of questions or problems during the build process, please
<a href="http://www.reflexxes.com/company/contact" title="http://www.reflexxes.com/company/contact" target="_blank" ><b>contact us</b></a>.

\sa \ref page_SourceCode
\sa \ref page_GettingStarted


\n

\endif

*/

