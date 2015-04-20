//  ---------------------- Doxygen info ----------------------
//! \file Doxygen_TypeII_VersionHistory.h
//!
//! \brief
//! Documentation file for Doxygen (version history)
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
\page page_VersionHistory Version History of the Type II Reflexxes Motion Library


\section sec_Version12 Version 1.2 (February 2012)

- Version 1.2 is the first public release of the Type II Reflexxes Motion Library

\section sec_Version122 Version 1.2.2 (March 2012)

- API GetReferenceOfXXX() methods removed.
- RMLOutputParameters::Echo() is now protected.
- Minor fixes and improvements.

\section sec_Version123 version 1.2.3 (October 2013)

 - Minor fixes and improvements.

\section sec_Version124 version 1.2.4 (June 2013)

- The \b optional input parameter
RMLInputParameters::MinimumSynchronizationTime was added, which
allows specifying a desired execution of the trajectory.
- Two examples
<ul>
<li>\ref page_Code_07_RMLPositionSampleApplication</li>
<li>\ref page_Code_08_RMLVelocitySampleApplication</li>
</ul>
were added.
- \b Bug fixed: In rare cases, when the maximum number of polynomial 
  was used, an memory fault could have happen in the method
  TypeIIRMLPosition::Step3(). This has been corrected.

\section sec_Version125 version 1.2.5 (February 2014)  

- \b Bug fixed: In rare cases, the maximum velocity and the maximum
  acceleration were swapped (cf. file TypeIIRMLStep1.cpp).

\section sec_Version126 version 1.2.6 (March 2014)  

- \b Bug fixed: If inoperative time intervals are existent, an incorrect
  trajectory is computed, because Decision 6 in Step 2 was not
  implemented correctly (cf. TypeIIRMLMath::Decision_2___006()).


\section sec_Version127 version 1.2.7 (April 2015)  

 - The source code has moved to <a href="http://www.github.com" target="_blank" >GitHub</a>.
 - \b Bug fixed: Under very unlikely circumstances, a division by zero 
   could happen in some of the function in the file TypeIIRMLStep2Profiles.cpp.
   This has been fixed.
 - Minor changes and improvements
 - Doxygen documentation files openly available

*/

