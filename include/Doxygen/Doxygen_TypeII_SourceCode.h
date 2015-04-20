//  ---------------------- Doxygen info ----------------------
//! \file Doxygen_TypeII_SourceCode.h
//!
//! \brief
//! Documentation file for Doxygen (brief overview about the source code)
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
\page page_SourceCode Overview About the Source Code

The source code of the <em>Type II Reflexxes Motion Library</em> is structured in a
simple and clear three-layered way:\n\n

<ol>
    <li>\ref sec_InterfaceLayer\n\n</li>
    <li>\ref sec_AlgorithmLayer\n\n</li>
    <li>\ref sec_MathLayer\n\n</li>
</ol>
 
\image html SourceCodeLayers.png "Illustration of the three source code layers of the \em Reflexxes \em Motion \em Libraries."

<hr>
 
\section sec_InterfaceLayer The Interface Layer

The <em>interface layer</em> is based on the class ReflexxesAPI and constitutes the user API of the Reflexxes
Motion Libraries. It wraps around the algorithmic classes
TypeIIRMLPosition and TypeIIRMLPosition (cf. \ref sec_AlgorithmLayer) and
hides all functionalities that are not required by the user application in
order to obtain a clean and minimal interface for all applications. 
For a description of input and output values, please refer to the
\ref page_InputValues and the \ref page_OutputValues.\n

\n

<b>Class List</b>

<ul>
    <li>ReflexxesAPI</li>
    <li>RMLPositionInputParameters</li>
    <li>RMLPositionOutputParameters</li>
    <li>RMLPositionFlags</li>
    <li>RMLVelocityInputParameters</li>
    <li>RMLVelocityOutputParameters</li>
    <li>RMLVelocityFlags</li>
    <li>RMLVector</li>
</ul>
\n

<b>File List</b>

<ul>
    <li>ReflexxesAPI.h</li>
    <li>RMLInputParameters.h</li>
    <li>RMLOutputParameters.h</li>
    <li>RMLFlags.h</li>
    <li>RMLPositionInputParameters.h</li>
    <li>RMLPositionOutputParameters.h</li>
    <li>RMLPositionFlags.h</li>
    <li>RMLVelocityInputParameters.h</li>
    <li>RMLVelocityOutputParameters.h</li>
    <li>RMLVelocityFlags.h</li>
    <li>RMLVector.h</li>
    <li>ReflexxesAPI.cpp</li> 
</ul>

\n

<hr>

\section sec_AlgorithmLayer The Algorithm Layer

The <em>algorithm layer</em> contains the actual Type II On-Line Trajectory
Generation (OTG) algorithm. The position-based algorithm is realized by the
class TypeIIRMLPosition, and the velocity-based algorithm is realized by 
the class TypeIIRMLVelocity. Both classes make use of the namespace 
TypeIIRMLMath, which provides all mathematical foundation for the 
algorithms (cf. \ref sec_MathLayer). The methods to call the actual 
algorithms are TypeIIRMLPosition::GetNextStateOfMotion() and
TypeIIRMLPosition::GetNextStateOfMotion(), both of which are used by the
API class ReflexxesAPI (cf. \ref sec_InterfaceLayer).

\n

<b>Class List</b>

<ul>
    <li>TypeIIRMLPosition</li>
    <li>TypeIIRMLVelocity</li>
</ul>

\n

<b>File List for the Position-based Algorithm</b>

<ul>
    <li>TypeIIRMLPosition.h</li>
    <li>TypeIIRMLPosition.cpp</li>
    <li>TypeIIRMLCalculatePositionalExtrems.cpp</li>
    <li>TypeIIRMLFallBackStrategy.cpp</li>
    <li>TypeIIRMLIsPhaseSynchronizationPossible.cpp</li>
    <li>TypeIIRMLSetupModifiedSelectionVector.cpp</li>
    <li>TypeIIRMLStep1.cpp</li>
    <li>TypeIIRMLStep2PhaseSynchronization.cpp</li>
    <li>TypeIIRMLStep2.cpp</li>
    <li>TypeIIRMLStep3.cpp</li>
</ul>
 

\n

<b>File List for the Velocity-based Algorithm</b>

<ul>
    <li>TypeIIRMLVelocity.h</li>
    <li>TypeIIRMLVelocity.cpp</li>
    <li>TypeIIRMLVelocityCalculatePositionalExtrems.cpp</li>
    <li>TypeIIRMLVelocityFallBackStrategy.cpp</li>
    <li>TypeIIRMLVelocityIsPhaseSynchronizationPossible.cpp</li>
    <li>TypeIIRMLVelocityMethods.cpp</li>
    <li>TypeIIRMLVelocitySetupPhaseSyncSelectionVector.cpp</li>
</ul>

<hr>

\section sec_MathLayer The Math Layer

The <em>math layer</em> is the most fundamental layer. It provides
a collection of mathematical functions that required by the classes
TypeIIRMLPosition and TypeIIRMLVelocity. All these functions are contained
in the namespace TypeIIRMLMath.

\n

<b>Class List</b>

<ul>
    <li>TypeIIRMLMath::MotionPolynomials</li>
    <li>TypeIIRMLMath::TypeIIRMLPolynomial</li>
</ul>

\n

<b>File List</b>

<ul>
    <li>TypeIIRMLDecisionTree1A.h</li>
    <li>TypeIIRMLDecisionTree1B.h</li>
    <li>TypeIIRMLDecisionTree1C.h</li>
    <li>TypeIIRMLDecisionTree2.h</li>
    <li>TypeIIRMLMath.h</li>
    <li>TypeIIRMLPolynomial.h</li>
    <li>TypeIIRMLQuicksort.h</li>
    <li>TypeIIRMLDecisions.h</li>
    <li>TypeIIRMLStep1IntermediateProfiles.h</li>
    <li>TypeIIRMLStep1Profiles.h</li>
    <li>TypeIIRMLStep2IntermediateProfiles.h</li>
    <li>TypeIIRMLStep2Profiles.h</li>
    <li>TypeIIRMLStep2WithoutSynchronization.h</li>
    <li>TypeIIRMLDecisionTree1A.cpp</li>
    <li>TypeIIRMLDecisionTree1B.cpp</li>
    <li>TypeIIRMLDecisionTree1C.cpp</li>
    <li>TypeIIRMLDecisionTree2.cpp</li>
    <li>TypeIIRMLPolynomial.cpp</li>
    <li>TypeIIRMLQuicksort.cpp</li>
    <li>TypeIIRMLDecisions.cpp</li>
    <li>TypeIIRMLStep1IntermediateProfiles.cpp</li>
    <li>TypeIIRMLStep1Profiles.cpp</li>
    <li>TypeIIRMLStep2IntermediateProfiles.cpp</li>
    <li>TypeIIRMLStep2Profiles.cpp</li>
    <li>TypeIIRMLStep2WithoutSynchronization.cpp</li>
</ul>

*/


