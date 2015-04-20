//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLPosition.h
//!
//! \brief
//! Header file for the class TypeIIRMLPosition, which constitutes the
//! actual interface of the Type II Reflexxes Motion Library
//!
//! \details
//! The class TypeIIRMLPosition contains the actual Reflexxes Type II
//! On-Line Trajectory Generation algorithm. The class
//! provides its interface by the the class ReflexxesAPI.
//!
//! \date April 2015
//!
//! \version 1.2.7
//!
//! \author Torsten Kroeger, <info@reflexxes.com> \n
//!
//! \copyright Copyright (C) 2015 Google, Inc.
//! \n
//! \n
//! <b>GNU Lesser General Public License</b>
//! \n
//! \n
//! This file is part of the Type II Reflexxes Motion Library.
//! \n\n
//! The Type II Reflexxes Motion Library is free software: you can redistribute
//! it and/or modify it under the terms of the GNU Lesser General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Type II Reflexxes Motion Library is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU Lesser General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU Lesser General Public License
//! along with the Type II Reflexxes Motion Library. If not, see
//! <http://www.gnu.org/licenses/>.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#ifndef __TypeIIRMLPosition__
#define __TypeIIRMLPosition__


#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <RMLPositionFlags.h>
#include <RMLVelocityFlags.h>
#include <RMLVector.h>
#include <TypeIIRMLPolynomial.h>
#include <TypeIIRMLStep1Profiles.h>
#include <TypeIIRMLVelocity.h>


using namespace TypeIIRMLMath;


//  ---------------------- Doxygen info ----------------------
//! \class TypeIIRMLPosition
//!
//! \brief
//! <b>This class constitutes the low-level user interface of the Reflexxes
//! Type II Motion Library, which contains the Type II On-Line Trajectory
//! Generation algorithm</b>
//!
//! \details
//! This class is the low-level application interface of the Type II
//! On-Line Trajectory Generation algorithm. The wrapper class ReflexxesAPI,
//! simplifies this interface for the user, such that all relevant
//! functionalities can be accessed, but all parts that are not needed by
//! the user are hidden.\n
//! \n
//! The mathematical futtocks of the algorithm are described in\n
//! \n
//! <b>T. Kroeger.</b>\n
//! <b>On-Line Trajectory Generation in Robotic Systems.</b>\n
//! <b>Springer Tracts in Advanced Robotics, Vol. 58, Springer, January 2010.</b>\n
//! <b><a href="http://www.springer.com/978-3-642-05174-6" target="_blank" title="You may order your personal copy at www.springer.com.">http://www.springer.com/978-3-642-05174-6</a></b>\n
//! \n
//! The algorithm makes use of the namespace TypeIIRMLMath, which is a
//! collection of mathematical functions required for the algorithm.
//! Besides others, this namespace contains the functions for each single
//! decision of all decision trees as well as all functions to solve the
//! systems of equations that occur in the leaves of
//! of the decision trees.\n
//!
//! <ul>
//!  <li>The decision trees itself are part of this class.</li>
//!  <li>Furthermore, all \em management functionalities (e.g., the usage of
//!    time- \em or phase-synchronization criteria,
//!    cf. \ref page_SynchronizationBehavior),</li>
//!  <li>the three-layered error handling mechanism to ensure valid output
//!    values in \em any case,</li>
//!  <li>verification and scalings of input and output values,</li>
//!  <li>the implementation of
//! <ul>
//!     <li><b>Step 1</b>, that is, calculating \f$ t_i^{\,sync} \f$ by using the
//!       the decision trees 1A, 1B, and 1C\n</li>
//!     <li><b>Step 2</b>, that is, calculating all parameters
//!       \f$ {\cal M}_{i}(t) \f$ of the synchronized motion trajectory\n</li>
//!     <li><b>Step 3</b>, that is, calculating the output values of the algorithm\n</li>
//! </ul>
//!    are part of this class, and</li>
//!  <li> the calculation of extremum states of motion is done by this class.</li>
//! </ul>
//!
//! Besides the constructor, the class only has one method that is relevant
//! for the user: TypeIIRMLPosition::GetNextStateOfMotion(). The input and
//! and output values of this function are described in the classes
//! RMLFlags, RMLPositionInputParameters, and RMLPositionOutputParameters,
//! all of which are also described in the context of the class
//! ReflexxesAPI. Information about all these parameters can be found
//! on the pages
//!
//!  - \ref page_TypeIIAndIVOverview,
//!  - \ref page_InputValues, and
//!  - \ref page_OutputValues.\n
//!
//! The class TypeIIRMLVelocity is a byproduct only. It can be used in the
//! very same as this class, but target position vectors
//! \f$ \vec{P}_i^{\,trgt} \f$ cannot be specified. While the class
//! TypeIIRMLPosition can be used for the instantaneous computation of
//! motions towards a desired target pose or position, the class
//! TypeIIRMLVelocity basically has two functionalities:
//!
//! -# TypeIIRMLVelocity is the main component for the \em second safety
//!    layer (cf. \ref page_ErrorHandling).
//! -# TypeIIRMLVelocity can be used to generate a trajectory starting
//!    from an arbitrary initial state of motion to achieve a certain
//!    target velocity \f$ \vec{V}_i^{\,trgt} \f$.
//!
//! \sa TypeIIRMLVelocity
//! \sa RMLPositionInputParameters
//! \sa RMLPositionOutputParameters
//! \sa ReflexxesAPI
//! \sa TypeIIRMLMath
//  ----------------------------------------------------------
class TypeIIRMLPosition
{
public:


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIIRMLPosition(const unsigned int &DegreesOfFreedom, const double &CycleTimeInSeconds)
//!
//! \brief
//! Constructor of the class TypeIIRMLPosition
//!
//! \details
//! The two tasks of the constructor are
//!
//!  -# Initializing all class attributes
//!  -# Allocating and initializing memory for all pointer attributes
//!
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//!
//! \param DegreesOfFreedom
//! Specifies the number of degrees of freedom
//!
//! \param CycleTimeInSeconds
//! Specifies the cycle time in seconds
//!
//! \sa TypeIIRMLPosition::~TypeIIRMLPosition()
//  ----------------------------------------------------------
    TypeIIRMLPosition(      const unsigned int  &DegreesOfFreedom
                        ,   const double        &CycleTimeInSeconds);


//  ---------------------- Doxygen info ----------------------
//! \fn ~TypeIIRMLPosition(void)
//!
//! \brief
//! Destructor of the class TypeIIRMLPosition
//!
//! \details
//! \em All the heap memory that was allocated by the constructor is freed
//! again.
//!
//! \sa TypeIIRMLPosition::TypeIIRMLPosition()
//  ----------------------------------------------------------
    ~TypeIIRMLPosition(void);


//  ---------------------- Doxygen info ----------------------
//! \fn int GetNextStateOfMotion(const RMLPositionInputParameters &InputValues, RMLPositionOutputParameters *OutputValues, const RMLPositionFlags &Flags)
//!
//! \brief
//! <b>The main method of the class TypeIIRMLPosition. It executes
//! the position-based Type II On-Line Trajectory Generation algorithm</b>
//!
//! \details
//! Given a set of \c InputValues consisting of
//!
//!  - a current state of motion \f$ {\bf M}_i \f$ at intstant \f$ T_i \f$,
//!  - a target state of motion \f$ {\bf M}_i^{\,trgt} \f$ at intstant
//!     \f$ T_i \f$ (with zero acceleration),
//!  - kinematic motion constraints \f$ {\bf B}_i \f$ at intstant
//!    \f$ T_i \f$, and
//!  - a selection vector \f$ \vec{S}_i \f$ at intstant \f$ T_i \f$
//!
//! and a set of boolean \c Flags to control the behavior of the algorithm,
//! this method executes the position-based Type II On-Line Trajectory
//! Generation algorithm and provides a set of \c OutputValues, which
//! contain
//!
//!  - the desired state of motion \f$ {\bf M}_{i+1} \f$ at intstant
//!    \f$ T_{i+1} \f$ and
//!  - (optionally) further complementary values of the current trajectory.
//!
//! For a detailed description, please refer to TypeIIRMLPosition and
//! to the start page \ref index.
//!
//! \param InputValues
//! Input values of the position-based Type II On-Line Trajectory
//! Generation algorithm. For detailed information, please refer to the
//! class RMLPositionInputParameters and to the page \ref page_InputValues.
//!
//! \param OutputValues
//! Output values of the position-based Type II On-Line Trajectory
//! Generation algorithm. For detailed information, please refer to the
//! class RMLPositionOutputParameters and to the page
//! \ref page_OutputValues.
//!
//! \param Flags
//! A set of boolean values to configure the behavior of the algorithm
//! (e.g., specify whether a time- or a phase-synchronized trajectory is
//! desired, specify, whether the complementary output values are supposed
//! to be computed). For a detailed description of this data structure and
//! its usage, please refer to RMLPositionFlags.
//!
//! \return
//! An element of ReflexxesAPI::RMLResultValue:\n\n
//!  - ReflexxesAPI::RML_WORKING: \copydoc ReflexxesAPI::RML_WORKING\n\n
//!  - ReflexxesAPI::RML_FINAL_STATE_REACHED: \copydoc ReflexxesAPI::RML_FINAL_STATE_REACHED\n\n
//!  - ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES: \copydoc ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES\n\n
//!  - ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS: \copydoc ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS\n\n
//!  - ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION: \copydoc ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION\n\n
//!  - ReflexxesAPI::RML_ERROR_NULL_POINTER: \copydoc ReflexxesAPI::RML_ERROR_NULL_POINTER\n\n
//!  - ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG: \copydoc ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG\n\n
//!
//! \note
//! A complete description of the On-Line Trajectory Generation framework
//! may be also found at\n
//! \n
//! <b>T. Kroeger.</b>\n
//! <b>On-Line Trajectory Generation in Robotic Systems.</b>\n
//! <b>Springer Tracts in Advanced Robotics, Vol. 58, Springer, January 2010.</b>\n
//! <b><a href="http://www.springer.com/978-3-642-05174-6" target="_blank" title="You may order your personal copy at www.springer.com.">http://www.springer.com/978-3-642-05174-6</a></b>\n
//! \n
//!
//! \sa TypeIIRMLPosition
//! \sa RMLPositionInputParameters
//! \sa RMLPositionOutputParameters
//! \sa RMLPositionFlags
//! \sa ReflexxesAPI
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa TypeIIRMLPosition::GetNextStateOfMotionAtTime()
//! \sa TypeIIRMLMath
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    int GetNextStateOfMotion(       const RMLPositionInputParameters    &InputValues
                                ,   RMLPositionOutputParameters         *OutputValues
                                ,   const RMLPositionFlags              &Flags);


//  ---------------------- Doxygen info ----------------------
//! \fn int GetNextStateOfMotionAtTime(const double &TimeValueInSeconds, RMLPositionOutputParameters *OutputValues) const
//!
//! \brief
//! Once the method of TypeIIRMLPosition::GetNextStateOfMotion() was
//! \em successfully called to compute a trajectory, this method can be
//! used to compute a state of motion on this trajectory at a given time
//! instant.
//!
//! \details
//! After the method GetNextStateOfMotion() was called and no error value
//! was returned (i.e., ReflexxesAPI::RML_WORKING or
//! ReflexxesAPI::RML_FINAL_STATE_REACHED was returned), a trajectory was
//! successfully generated. In order to compute a state of motion of this
//! trajectory at a given time instant, this method can be used.
//! No new calculations are started by calling this method; only the
//! existing result of the method GetNextStateOfMotion() is used.
//! \c TimeValueInSeconds specifies the time of the desired state of
//! motion, which is copied to OutputValues (cf.
//! RMLPositionOutputParameters).\n
//! \n
//! If the method TypeIIRMLPosition::GetNextStateOfMotion() returned an
//! error, the same error will be returned by this method. The value of
//! \c TimeValueInSeconds has to be positive and below the values of
//! RML_MAX_EXECUTION_TIME (\f$ 10^{10} \f$ seconds).\n
//! \n
//! For further information, please refer to the documentation of
//! TypeIIRMLPosition::GetNextStateOfMotion().
//!
//! \param TimeValueInSeconds
//! Time value in seconds, at which the desired state of motion is
//! calculated.
//!
//! \param OutputValues
//! Output values of the position-based Type II On-Line Trajectory
//! Generation algorithm. For detailed information, please refer to the
//! class RMLPositionOutputParameters and to the page
//! \ref page_OutputValues.
//!
//! \return
//! An element of ReflexxesAPI::RMLResultValue:\n\n
//!  - ReflexxesAPI::RML_WORKING: \copydoc ReflexxesAPI::RML_WORKING\n\n
//!  - ReflexxesAPI::RML_FINAL_STATE_REACHED: \copydoc ReflexxesAPI::RML_FINAL_STATE_REACHED\n\n
//!  - ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES: \copydoc ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES\n\n
//!  - ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS: \copydoc ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS\n\n
//!  - ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION: \copydoc ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION\n\n
//!  - ReflexxesAPI::RML_ERROR_NULL_POINTER: \copydoc ReflexxesAPI::RML_ERROR_NULL_POINTER\n\n
//!  - ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG: \copydoc ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG\n\n
//!  - ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE: \copydoc ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE\n\n
//!
//! \sa TypeIIRMLPosition
//! \sa RMLPositionOutputParameters
//! \sa RMLPositionFlags
//! \sa ReflexxesAPI
//! \sa TypeIIRMLPosition::GetNextStateOfMotion()
//! \sa TypeIIRMLVelocity::GetNextStateOfMotionAtTime()
//  ----------------------------------------------------------
    int GetNextStateOfMotionAtTime(     const double                        &TimeValueInSeconds
                                    ,   RMLPositionOutputParameters         *OutputValues       ) const;

protected:

//  ---------------------- Doxygen info ----------------------
//! \enum FunctionResults
//!
//! \brief
//! For class-internal use only: return values of boolean methods
//  ----------------------------------------------------------
    enum FunctionResults
    {
        //! \brief The method was executed without any error
        FUNC_SUCCESS        =   false,
        //! \brief The method was executed, and an error occurred
        FUNC_ERROR_OCCURRED =   true
    };


//  ---------------------- Doxygen info ----------------------
//! \enum DominatValueForPhaseSync
//!
//! \brief
//! Set of input vector identifiers that can determine the normalized
//! vector for phase-synchronization.
//!
//! \details
//! In order to assure numerical stability, the input vector with the
//! greatest magnitude is used to compute the normalized reference
//! direction vector for phase-synchronized trajectories. The attribute
//! TypeIIRMLPosition::PhaseSynchronizationMagnitude is set-up by the
//! method TypeIIRMLPosition::IsPhaseSynchronizationPossible(),
//! which checks, whether all conditions for the generation of a
//! phase-synchronized trajectory are fulfilled.
//!
//! \sa TypeIIRMLPosition::PhaseSynchronizationMagnitude
//! \sa TypeIIRMLPosition::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    enum DominatValueForPhaseSync
    {
        //! \brief No value has been assigned yet.
        UNDEFINED           =   0,
        //! \brief The position difference vector
        //! \f$ \left( \vec{P}_i^{\,trgt}\ -\ \vec{P}_i\right) \f$
        //! was to the dominant value to determine the reference vector
        //! \f$ \vec{\varrho}_i \f$ for a phase-synchronized trajectory.
        POSITION            =   1,
        //! \brief The current acceleration vector \f$ \vec{A}_i \f$
        //! was to the dominant value to determine the reference vector
        //! \f$ \vec{\varrho}_i \f$ for a phase-synchronized trajectory.
        CURRENT_VELOCITY    =   3,
        //! \brief The target velocity vector \f$ \vec{V}_i^{\,trgt} \f$
        //! was to the dominant value to determine the reference vector
        //! \f$ \vec{\varrho}_i \f$ for a phase-synchronized trajectory.
        TARGET_VELOCITY     =   4
    };


//  ---------------------- Doxygen info ----------------------
//! \fn void CompareInitialAndTargetStateofMotion(void)
//!
//! \brief
//! If the initial state of motion \em exactly equals the target state of
//! motion, an adaptation is performed
//!
//! \details
//! If the initial state of motion \em exactly equals the target state
//! of motion, we add a negligible error to the input state of motion in
//! order to let the decision trees run deterministically. Otherwise, these
//! values would be a singularity for the decision trees as
//!
//! - either <em>no trajectory</em> is required (i.e., an execution time
//!   of zero seconds), or
//! - the current motion is continued until the same state of motion is
//!   reached again (time-optimally).
//!
//! As this case can only occur, if the input values change, \em and if
//! the output values of the last cycle are not directly fed back to the
//! input parameters of this cycle, we need to calculate the trajectory to
//! reach the desired state of motion. Otherwise, no trajectory would be
//! required at all and it would not make sense to the On-Line
//! Trajectory Generation algorithm.
//!
//! \sa TypeIIRMLPosition::GetNextStateOfMotion()
//  ----------------------------------------------------------
    void CompareInitialAndTargetStateofMotion(void);


//  ---------------------- Doxygen info ----------------------
//! \fn void Step1(void)
//!
//! \brief
//! Step 1 of the On-Line Trajectory Generation algorithm: Calculate
//! the synchronization time \f$ t_i^{\,sync} \f$
//!
//! \details
//! The only result of this method is a value for the synchronization time
//! \f$ t_i^{\,sync} \f$ (cf. TypeIIRMLPosition::SynchronizationTime) and
//! the information, whether the motion is phase-synchronized or
//! time-synchronized (cf.
//! TypeIIRMLPosition::CurrentTrajectoryIsPhaseSynchronized).\n
//! \n
//! To achieve this, a set of other functionalities is used:
//! <ul>
//!   <li>TypeIIRMLMath::TypeIIRMLDecisionTree1A() to calculate
//!       \f$ \vec{t}_i^{\,min} \f$ (cf.
//!       TypeIIRMLPosition::MinimumExecutionTimes)</li>
//!   <li>TypeIIRMLPosition::IsPhaseSynchronizationPossible() to check, whether
//!       phase-synchronization is possible</li>
//!   <li><ul>
//!         <li>TypeIIRMLMath::TypeIIRMLDecisionTree1B(), and</li>
//!         <li>TypeIIRMLMath::TypeIIRMLDecisionTree1C()\n</li>
//!       </ul>
//!       to calculate all inoperative time intervals
//!       \f$ _k{\cal Z}_i\ \forall\ k\ \in\ \left\{1,\,\dots,\,K\right\} \f$
//!   <li>TypeIIRMLMath::Quicksort() to sort all calculated times and to
//!       finally determine the synchronization time \f$ t_i^{\,sync} \f$.
//! </ul>
//! \n
//! A brief overview about the interrelations among the different steps
//! and decision trees can be found in section \ref page_TypeIIAndIVOverview.
//!
//! \sa TypeIIRMLPosition::Step2()
//! \sa TypeIIRMLPosition::Step3()
//! \sa TypeIIRMLPosition::SynchronizationTime
//! \sa TypeIIRMLPosition::CurrentTrajectoryIsPhaseSynchronized
//! \sa \ref page_TypeIIAndIVOverview
//  ----------------------------------------------------------
    void Step1(void);


//  ---------------------- Doxygen info ----------------------
//! \fn void Step2(void)
//!
//! \brief
//! Step 2 of the On-Line Trajectory Generation algorithm: time
//! synchronization of all selected degrees of freedom
//!
//! \details
//! After the synchronization time \f$ t_i^{\,sync} \f$ (cf.
//! TypeIIRMLPosition::SynchronizationTime) was calculated by the method
//! TypeIIRMLPosition::Step1(), this method computes all trajectory
//! parameters \f$ {\cal M}_{i}(t) \f$ of the entire
//! trajectory, which finally is represented by
//! TypeIIRMLPosition::Polynomials.
//! Depending on whether the motion is phase-synchronized, we
//! have to distinguish between two cases
//! (cf. TypeIIRMLPosition::CurrentTrajectoryIsPhaseSynchronized):
//!
//!  - <b>Time-synchronization.</b> In this case and depending on how many
//!    threads are available (cf. TypeIIRMLPosition::NumberOfOwnThreads),
//!    parts of this method may be executed in a concurrent way
//!    (cf. TypeIIRMLPosition::ThreadControlInstance). The method
//!    TypeIIRMLMath::TypeIIRMLDecisionTree2() contains the actual
//!    Step 2 decision tree, which selects a Step velocity profile,
//!    solves the corresponding system of nonlinear equations, and uses
//!    the solution to set-up all trajectory parameters.
//!  - <b>Phase-synchronization.</b> The method
//!    TypeIIRMLPosition::Step2PhaseSynchronization() will be applied.
//!    This method always runs single-threaded, because it only computes
//!    the trajectory for one single degree of freedom (cf.
//!    TypeIIRMLPosition::GreatestDOFForPhaseSynchronization), and all
//!    other selected degrees of freedom are linearly dependent on this
//!    reference one.\n
//! \n
//! A brief overview about the interrelations among the different steps
//! and decision trees can be found in section \ref page_TypeIIAndIVOverview.
//!
//! \sa TypeIIRMLPosition::Step1()
//! \sa TypeIIRMLPosition::Step3()
//! \sa TypeIIRMLPosition::SynchronizationTime
//! \sa TypeIIRMLPosition::CurrentTrajectoryIsPhaseSynchronized
//! \sa \ref page_TypeIIAndIVOverview
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    void Step2(void);


//  ---------------------- Doxygen info ----------------------
//! \fn int Step3(const double &TimeValueInSeconds) const
//!
//! \brief
//! Step 3 of the On-Line Trajectory Generation algorithm: calculate output
//! values
//!
//! \details
//! After all parameters \f$ {\cal M}_{i}(t) \f$
//! (cf. TypeIIRMLPosition::Polynomials) of the synchronized trajectory
//! have been calculated in the second step by TypeIIRMLPosition::Step2(),
//! this method computes the actual output values,
//! that is, the desired state of motion \f$ {\bf M}_{i+1} \f$ at intstant
//! \f$ T_{i+1} \f$ (cf. TypeIIRMLPosition::OutputParameters) and the
//! return value TypeIIRMLPosition::ReturnValue, which is an element of
//! the enumeration ReflexxesAPI::RMLResultValue.\n
//! \n
//! A brief overview about the interrelations among the different steps
//! and decision trees can be found in section \ref page_TypeIIAndIVOverview.
//!
//! \param TimeValueInSeconds
//! Time value in seconds, at which the next state of motion is calculated.
//!
//! \param OP
//! Pointer to an object of the class RMLPositionOutputParameters. All
//! output values will be written into this data structure.
//!
//! \return
//! The return value for the method TypeIIRMLPosition::GetNextStateOfMotion()
//!
//! \sa TypeIIRMLPosition::Step1()
//! \sa TypeIIRMLPosition::Step2()
//! \sa TypeIIRMLPosition::OutputParameters
//! \sa TypeIIRMLPosition::ReturnValue
//! \sa RMLPositionFlags
//! \sa \ref page_TypeIIAndIVOverview
//  ----------------------------------------------------------
    int Step3(      const double                    &TimeValueInSeconds
                ,   RMLPositionOutputParameters     *OP                 ) const;


//  ---------------------- Doxygen info ----------------------
//! \fn void FallBackStrategy( const RMLPositionInputParameters &InputValues, RMLPositionOutputParameters *OutputValues, const RMLPositionFlags &InputsFlags)
//!
//! \brief
//! In case of an error, this method triggers the second layer of the
//! safety concept
//!
//! \details
//! If no trajectory can be calculated by the position-based On-Line
//! Trajectory Generation algorithm
//! (TypeIIRMLPosition::GetNextStateOfMotion()), the velocity-based
//! algorithm (TypeIIRMLVelocity::GetNextStateOfMotion()) is called in the
//! second safety layer. Before this call can be made, this method casts the
//! TypeIIRMLPosition::CurrentInputParameters object and the
//! RMLPositionFlags input flag object used in
//! TypeIIRMLPosition::GetNextStateOfMotion() to an
//! RMLVelocityInputParameters object and an RMLVelocityFlags object.
//! During this casting, the desired target velocity vector
//! \f$ \vec{V}_{i}^{\,trgt} \f$ for the velocity-based On-Line
//! Trajectory Generation algorithm is either set
//!
//!  - to RMLPositionInputParameters::AlternativeTargetVelocityVector of
//!    the TypeIIRMLPosition::CurrentInputParameters object if the flag
//!    RMLPositionFlags::KeepCurrentVelocityInCaseOfFallbackStrategy
//!    is set to \c false or
//!  - to the current velocity vector \f$ \vec{V}_{i} \f$ if
//!    the flag RMLPositionFlags::KeepCurrentVelocityInCaseOfFallbackStrategy
//!    is set to \c true.
//!
//! Subsequently to the casting procedure,
//! TypeIIRMLVelocity::GetNextStateOfMotion() is called and generates
//! valid and feasible output values, which are represented in a
//! RMLVelocityOutputParameters that finally is casted to a
//! RMLPositionOutputParameters object, namely \c OutputValues.
//!
//! A detailed description of the three-layered safety mechanism of the
//! Reflexxes Motion Libraries can be found at \ref page_ErrorHandling.
//!
//! \param InputValues
//! The current input values of the position-based On-Line Trajectory
//! Generation algorithm. These values are casted to an
//! RMLVelocityInputParameters as described above
//!
//! \param OutputValues
//! Pointer to an RMLVelocityOutputParameters object. The method writes
//! the resulting output values of the velocity-based On-Line Trajectory
//! Generation algorithm into this object.
//!
//! \param InputsFlags
//! The current input flags of the position-based On-Line Trajectory
//! Generation algorithm.
//!
//! \note
//! By default, the flag
//! RMLPositionFlags::KeepCurrentVelocityInCaseOfFallbackStrategy is set to
//! \c false, and the alternative target velocity vector
//! \f$ \vec{V}_{i}^{\,\underline{trgt}} \f$ is set to zero. Depending
//! on the requirements of the application, one may choose between
//! the two additional options that are described above by setting up
//! the value of
//! RMLPositionFlags::KeepCurrentVelocityInCaseOfFallbackStrategy and
//! RMLPositionInputParameters::AlternativeTargetVelocityVector of
//! the TypeIIRMLPosition::CurrentInputParameters object
//! correspondingly.
//!
//! \sa TypeIIRMLVelocity
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa TypeIIRMLPosition::OutputParameters
//! \sa TypeIIRMLPosition::ReturnValue
//! \sa \ref page_ErrorHandling
//  ----------------------------------------------------------
    void FallBackStrategy(      const RMLPositionInputParameters    &InputValues
                            ,   RMLPositionOutputParameters         *OutputValues
                            ,   const RMLPositionFlags              &InputsFlags);


//  ---------------------- Doxygen info ----------------------
//! \fn bool IsWithinAnInoperativeTimeInterval(const double &SynchronizationTimeCandidate, const RMLDoubleVector &MaximalExecutionTime, const RMLDoubleVector &AlternativeExecutionTime) const
//!
//! \brief
//! Checks, whether the value \c SynchronizationTimeCandidate lies
//! within an inoperative timer interval
//!
//! \details
//! After all minimum execution times \f$ \vec{t}_i^{\,min} \f$,
//! all inoperative time interval beginnings \f$ \vec{t}_i^{\,begin} \f$,
//! and inoperative time interval endings \f$ \vec{t}_i^{\,end} \f$ are
//! calculated be the decision trees 1A, 1B, and 1C, it has to be
//! checked for each possible candidate for \f$ t_i^{\,sync} \f$,
//! whether it is within an inoperative time interval
//! \f$ _k\zeta_{i}\ \forall\ k\ \in \ \left\{1,\,\dots,\,K\right\} \f$
//! where all inoperative intervals are described by
//! \f$ _k\zeta_{i}\,=\,\left[_kt_{i}^{\,begin},\,_kt_{i}^{\,end}\right] \f$.
//!
//! \param SynchronizationTimeCandidate
//! Possible candidate for \f$ t_i^{\,sync} \f$ that will be checked by
//! this method. The value is given in seconds.
//!
//! \param MaximalExecutionTime
//! Beginning of an inoperative time interval \f$ _kt_{i}^{\,begin} \f$ in
//! seconds
//!
//! \param AlternativeExecutionTime
//! Ending of an inoperative time interval \f$ _kt_{i}^{\,end} \f$ in
//! seconds
//!
//! \return
//!  - \c true if \c SynchronizationTimeCandidate lies within the
//!    inoperative time interval
//!  - \c false otherwise
//!
//! \sa TypeIIRMLPosition::Step1()
//  ----------------------------------------------------------
    bool IsWithinAnInoperativeTimeInterval(     const double            &SynchronizationTimeCandidate
                                            ,   const RMLDoubleVector   &MaximalExecutionTime
                                            ,   const RMLDoubleVector   &AlternativeExecutionTime) const;


//  ---------------------- Doxygen info ----------------------
//! \fn bool IsPhaseSynchronizationPossible(RMLDoubleVector *ReferenceVector)
//!
//! \brief
//! Checks, whether the motion trajectory can be phase-synchronized
//!
//! \details
//! After all minimum execution times \f$ \vec{t}_i^{\,min} \f$
//! have been calculated in Step 1A, it can be checked whether the
//! trajectory can be phase-synchronized. Therefore, this method checks
//! whether the input vectors
//!
//!  - current position difference vector
//!    \f$ \left(\vec{P}_{i}^{\,trgt}\,-\,\vec{P}_{i}\right) \f$,
//!  - current velocity vector \f$ \vec{V}_{i} \f$, and
//!  - target velocity vector \f$ \vec{V}_{i}^{\,trgt} \f$,
//!
//! are collinear to each other. If this is the case,
//!
//!  - \c true will be returned,
//!  - the reference vector \c ReferenceVector (\f$ \vec{\varrho}_i \f$)
//!    will be calculated, and
//!  - the value TypeIIRMLPosition::PhaseSynchronizationMagnitude
//!    will be set by this method.
//!
//! If this is not the case,
//!
//!  - \c false will be returned,
//!  - the content of \c ReferenceVector remains unchanged
//!  - the value of TypeIIRMLPosition::PhaseSynchronizationMagnitude
//!    remains unchanged.
//!
//! For all these computations, the attributes
//!
//!  - TypeIIRMLPosition::PhaseSynchronizationReferenceVector
//!  - TypeIIRMLPosition::PhaseSynchronizationCurrentPositionVector
//!  - TypeIIRMLPosition::PhaseSynchronizationTargetPositionVector
//!  - TypeIIRMLPosition::PhaseSynchronizationPositionDifferenceVector
//!  - TypeIIRMLPosition::PhaseSynchronizationCurrentVelocityVector
//!  - TypeIIRMLPosition::PhaseSynchronizationTargetVelocityVector
//!  - TypeIIRMLPosition::PhaseSynchronizationMaxVelocityVector
//!  - TypeIIRMLPosition::PhaseSynchronizationMaxAccelerationVector
//!  - TypeIIRMLPosition::PhaseSynchronizationTimeVector
//!  - TypeIIRMLPosition::PhaseSynchronizationCheckVector
//!
//! are used. Further information about time- and phase-synchronization
//! can be found in the section on \ref page_SynchronizationBehavior.
//!
//! \param ReferenceVector
//! Pointer to an \c RMLDoubleVector object.
//! If all mentioned conditions are fulfilled, such that the motion
//! trajectory can be phase-synchronized, the reference vector
//! \f$ \vec{\varrho}_i \f$ will be written to this RMLDoubleVector object.
//! If the motion cannot be phase-synchronized, the RMLDoubleVector object
//! will not be changed.
//!
//! \return
//!  - \c true if phase-synchronization is possible
//!  - \c false otherwise
//!
//! \sa TypeIIRMLPosition::Step1()
//! \sa TypeIIRMLVelocity::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    bool IsPhaseSynchronizationPossible(RMLDoubleVector *ReferenceVector);


//  ---------------------- Doxygen info ----------------------
//! \fn void Step2PhaseSynchronization(void)
//!
//! \brief
//! Executes Step 2 for phase-synchronized motion trajectories
//!
//! \details
//! This function executes the actual Step 2 for all selected degrees of
//! freedom. For the degree of freedom with the index \f$ \kappa \f$
//! (TypeIIRMLPosition::GreatestDOFForPhaseSynchronization), all trajectory
//! parameters \f$ _{\kappa}{\cal M}_{i}(t) \f$ are calculated. The
//! trajectory parameters for all other degrees of freedom
//! \f$ \left\{1,\,\dots,\,K\right\}\backslash\left\{\kappa\right\} \f$
//! are calculated using the reference vector \f$ \vec{\varrho}_i \f$
//! TypeIIRMLPosition::PhaseSynchronizationReferenceVector.\n
//! \n
//! In order to compensate numerical inaccuracies, the resulting
//! polynomials for the degrees of freedom
//! \f$ \left\{1,\,\dots,\,K\right\}\backslash\left\{\kappa\right\} \f$
//! are adapted. Therefore, a first-order polynomial is added to the
//! polynomials represented by TypeIIRMLPosition::Polynomials.
//!
//! \sa TypeIIRMLPosition::Step2()
//! \sa TypeIIRMLPosition::Step1()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    void Step2PhaseSynchronization(void);


//  ---------------------- Doxygen info ----------------------
//! \fn void CalculatePositionalExtrems(const double &TimeValueInSeconds, RMLPositionOutputParameters *OP) const
//!
//! \brief
//! Set all positional extremum parameters of the output values of the
//! algorithm (TypeIIRMLPosition::OutputParameters)
//!
//! \details
//! After all trajectory parameters \f$ {\cal M}_{i}(t) \f$ have been
//! computed in Step 2, they are stored in the attribute
//! TypeIIRMLPosition::Polynomials. Using this attribute, this method
//! computes all positional extremum values and corresponding states of
//! motion and writes the results to TypeIIRMLPosition::OutputParameters.
//! In particular, the following values are calculated:
//!
//!  - RMLPositionOutputParameters::MaxPosExtremaPositionVectorOnly
//!  - RMLPositionOutputParameters::MinPosExtremaPositionVectorOnly
//!  - RMLPositionOutputParameters::MaxExtremaTimesVector
//!  - RMLPositionOutputParameters::MaxPosExtremaPositionVectorArray
//!  - RMLPositionOutputParameters::MaxPosExtremaVelocityVectorArray
//!  - RMLPositionOutputParameters::MinExtremaTimesVector
//!  - RMLPositionOutputParameters::MinPosExtremaPositionVectorArray
//!  - RMLPositionOutputParameters::MinPosExtremaVelocityVectorArray
//!
//! All these values may be used by the user to perform further
//! calculations based on the currently calculated motion trajectory
//! (e.g., a check for workspace boundaries).
//!
//! \param TimeValueInSeconds
//! Time value in seconds, at which the next state of motion is calculated.
//! The positional extremes are calculated with respect to this value.
//!
//! \param OP
//! Pointer to an object of the class RMLPositionOutputParameters. The
//! positional extreme values will be calculated for these data.
//!
//! \note
//! The calculation of these values can be disabled by setting the flag
//! RMLPositionFlags::EnableTheCalculationOfTheExtremumMotionStates to
//! \c false when the method TypeIIRMLPosition::GetNextStateOfMotion() is
//! called.
//!
//! \sa \ref page_OutputValues
//! \sa RMLPositionOutputParameters
//! \sa RMLPositionFlags::EnableTheCalculationOfTheExtremumMotionStates
//! \sa TypeIIRMLPosition::GetNextStateOfMotion()
//! \sa TypeIIRMLPosition::SetPositionalExtremsToZero()
//! \sa TypeIIRMLVelocity::CalculatePositionalExtrems
//  ----------------------------------------------------------
    void CalculatePositionalExtrems(    const double                &TimeValueInSeconds
                                    ,   RMLPositionOutputParameters *OP                 ) const;


//  ---------------------- Doxygen info ----------------------
//! \fn void SetPositionalExtremsToZero(RMLPositionOutputParameters *OP) const
//!
//! \brief
//! Set all positional extremum parameters of the output values of the
//! algorithm (TypeIIRMLPosition::OutputParameters) to zero
//!
//! \details
//! If the input flag
//! RMLPositionFlags::EnableTheCalculationOfTheExtremumMotionStates is set
//! to \c false, this method is used to set all output values that are
//! related to the calculation of the positional extremum values to zero
//! in order to obtain defined output values:
//!
//!  - RMLPositionOutputParameters::MaxPosExtremaPositionVectorOnly
//!  - RMLPositionOutputParameters::MinPosExtremaPositionVectorOnly
//!  - RMLPositionOutputParameters::MaxExtremaTimesVector
//!  - RMLPositionOutputParameters::MaxPosExtremaPositionVectorArray
//!  - RMLPositionOutputParameters::MaxPosExtremaVelocityVectorArray
//!  - RMLPositionOutputParameters::MinExtremaTimesVector
//!  - RMLPositionOutputParameters::MinPosExtremaPositionVectorArray
//!  - RMLPositionOutputParameters::MinPosExtremaVelocityVectorArray
//!
//! If the input flag
//! RMLPositionFlags::EnableTheCalculationOfTheExtremumMotionStates
//! is set to \c true, the method
//! TypeIIRMLPosition::CalculatePositionalExtrems() is used to compute
//! this part of the output values.
//!
//! \param OP
//! Pointer to an object of the class RMLPositionOutputParameters. The
//! values of this data structure will be set to zero.
//!
//! \sa \ref page_OutputValues
//! \sa RMLPositionOutputParameters
//! \sa RMLPositionFlags::EnableTheCalculationOfTheExtremumMotionStates
//! \sa TypeIIRMLPosition::GetNextStateOfMotion()
//! \sa TypeIIRMLPosition::CalculatePositionalExtrems()
//  ----------------------------------------------------------
    void SetPositionalExtremsToZero(RMLPositionOutputParameters *OP) const;


//  ---------------------- Doxygen info ----------------------
//! \fn void SetupModifiedSelectionVector(void)
//!
//! \brief
//! Modify the current selection vector and exclude unnecessary
//! degrees of freedom
//!
//! \details
//! This method modifies the selection vector
//! RMLPositionInputParameters::SelectionVector of
//! TypeIIRMLPosition::CurrentInputParameters to
//! TypeIIRMLPosition::ModifiedSelectionVector.
//! Degrees of freedom that are already in their target state of motion,
//! and whose target state of motion \f$ {\bf M}_i^{\,trgt} \f$
//! consists of a velocity value of zero are removed from the selection
//! vector \f$ \vec{S}_i \f$, that is, the corresponding
//! elements are set to \c false. Although a correct solution
//! would be calculated for such cases, it is important to exclude them
//! in order give remaining degrees of freedom the chance to become
//! <em>phase-synchronized</em>. During the procedure of
//! phase-synchronization might, numerical problems may occur if degrees of
//! freedom are involved, that already reached their final and desired
//! target state of motion.
//!
//! \note
//! The method is called within the method TypeIIRMLPosition::Step1()
//! right after Step 1A was executed. All parts of the
//! algorithm before this point use the original selection vector of
//! TypeIIRMLPosition::CurrentInputParameters \f$ \vec{S}_i \f$, and all
//! following parts make use of TypeIIRMLPosition::ModifiedSelectionVector.
//!
//! \sa TypeIIRMLPosition::ModifiedSelectionVector
//! \sa RMLPositionInputParameters::SelectionVector
//! \sa TypeIIRMLPosition::GetNextStateOfMotion()
//! \sa TypeIIRMLPosition::Step1()
//! \sa TypeIIRMLVelocity::SetupPhaseSyncSelectionVector()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    void SetupModifiedSelectionVector(void);


//  ---------------------- Doxygen info ----------------------
//! \fn unsigned int GetNumberOfSelectedDOFs(const RMLBoolVector &BoolVector) const
//!
//! \brief
//! Returns the number of elements in \c BoolVector that are \c true
//!
//! \param BoolVector
//! An RMLBoolVector object (cf. RMLVector)
//!
//! \return
//! The number of elements in \c BoolVector that are \c true
//!
//! \sa TypeIIRMLPosition::ModifiedSelectionVector
//! \sa RMLPositionInputParameters::SelectionVector
//  ----------------------------------------------------------
    unsigned int GetNumberOfSelectedDOFs(const RMLBoolVector &BoolVector) const;


//  ---------------------- Doxygen info ----------------------
//! \var bool CurrentTrajectoryIsPhaseSynchronized
//!
//! \brief
//! Indicates, whether the current trajectory is phase-synchronized
//!
//! \sa TypeIIRMLPosition::IsPhaseSynchronizationPossible()
//! \sa RMLPositionOutputParameters::TrajectoryIsPhaseSynchronized
//! \sa TypeIIRMLVelocity::CurrentTrajectoryIsPhaseSynchronized
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    bool                        CurrentTrajectoryIsPhaseSynchronized;


//  ---------------------- Doxygen info ----------------------
//! \var bool CurrentTrajectoryIsNotSynchronized
//!
//! \brief
//! Indicates that no synchronization is required for the current set
//! of input values, that is, the input flag RMLFlags::NO_SYNCHRONIZATION
//! is set.
//!
//! \sa RMLFlags::NO_SYNCHRONIZATION
//  ----------------------------------------------------------
    bool                        CurrentTrajectoryIsNotSynchronized;


//  ---------------------- Doxygen info ----------------------
//! \var bool CalculatePositionalExtremsFlag
//!
//! \brief
//! Indicates, whether the positional extremes are to be calculated.
//!
//! \sa CalculatePositionalExtrems()
//  ----------------------------------------------------------
    bool                        CalculatePositionalExtremsFlag;


//  ---------------------- Doxygen info ----------------------
//! \var int ReturnValue
//!
//! \brief
//! Contains the return value of the method
//! TypeIIRMLPosition::GetNextStateOfMotion()
//!
//! \sa TypeIIRMLPosition::GetNextStateOfMotion()
//  ----------------------------------------------------------
    int                         ReturnValue;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int NumberOfDOFs
//!
//! \brief
//! The number of degrees of freedom \f$ K \f$
//!
//! \sa TypeIIRMLPosition::TypeIIRMLPosition()
//  ----------------------------------------------------------
    unsigned int                NumberOfDOFs;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int GreatestDOFForPhaseSynchronization
//!
//! \brief
//! Contains the index of the degree of freedom that was used to compute
//! the reference vector for phase-synchronization, \f$ \vec{\varrho}_i \f$
//!
//! \sa TypeIIRMLPosition::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    unsigned int                GreatestDOFForPhaseSynchronization;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int MotionProfileForPhaseSynchronization
//!
//! \brief
//! Contains the ID of the profile that is used for phase-synchronization
//!
//! \sa TypeIIRMLMath::Step1_Profile
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    unsigned int                MotionProfileForPhaseSynchronization;


//  ---------------------- Doxygen info ----------------------
//! \var double CycleTime
//!
//! \brief
//! Contains the cycle time in seconds
//!
//! \sa TypeIIRMLPosition::TypeIIRMLPosition()
//  ----------------------------------------------------------
    double                      CycleTime;


//  ---------------------- Doxygen info ----------------------
//! \var double SynchronizationTime
//!
//! \brief
//! If the trajectory is time- or phase-synchronized, this
//! attribute will contain the synchronization time \f$ t_i^{\,sync} \f$.
//! Otherwise,is used for the execution time of the degree of freedom that
//! requires the greatest time.
//!
//! \sa TypeIIRMLPosition::Step1()
//  ----------------------------------------------------------
    double                      SynchronizationTime;


//  ---------------------- Doxygen info ----------------------
//! \var double InternalClockInSeconds
//!
//! \brief
//! In order to prevent from recalculating the trajectory within every
//! control cycle and to safe CPU time, this time value in seconds
//! represents the elapsed time since the last calculation
//!
//! \sa TypeIIRMLPosition::GetNextStateOfMotion()
//  ----------------------------------------------------------
    double                      InternalClockInSeconds;


//  ---------------------- Doxygen info ----------------------
//! \var RMLPositionFlags OldFlags
//!
//! \brief
//! In order to check, whether a new calculation has to be started, the
//! input values have to be compared to the input and output values
//! of the previous cycle. This variable is used to store the flags of
//! last cycle
//!
//! \sa RMLOutputParameters::ANewCalculationWasPerformed
//! \sa TypeIIRMLPosition::GetNextStateOfMotion()
//! \sa TypeIIRMLVelocity::OldFlags
//! \sa OldInputParameters
//! \sa \ref page_InputValues
//  ----------------------------------------------------------
    RMLPositionFlags            OldFlags;


//  ---------------------- Doxygen info ----------------------
//! \var RMLBoolVector *ModifiedSelectionVector
//!
//! \brief
//! Boolean vector, which contains the modified selection vector that is
//! based on the original selection vector \f$ \vec{S}_i \f$
//!
//! \sa TypeIIRMLPosition::SetupModifiedSelectionVector()
//! \sa RMLPositionInputParameters::SelectionVector
//! \sa TypeIIRMLPosition::GetNextStateOfMotion()
//! \sa TypeIIRMLPosition::Step1()
//  ----------------------------------------------------------
    RMLBoolVector               *ModifiedSelectionVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLVector<Step1_Profile> *UsedStep1AProfiles
//!
//! \brief
//! Vector that contains the profiles that are used by Step 1A
//! to calculate \f$ \vec{t}_i^{\,min} \f$
//!
//! \sa TypeIIRMLPosition::Step1()
//! \sa TypeIIRMLMath::TypeIIRMLDecisionTree1A()
//  ----------------------------------------------------------
    RMLVector<Step1_Profile>    *UsedStep1AProfiles;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *StoredTargetPosition
//!
//! \brief
//! Stores the original target position vector \f$ \vec{P}_i^{\,trgt} \f$
//!
//! \details
//! In order to prevent from numerical inaccuracies, the algorithm
//! internally transforms the current position vector \f$ \vec{P}_i \f$
//! and the target position vector \f$ \vec{P}_i^{\,trgt} \f$
//! to a difference vector. This vector of double values
//! stores the original target position vector for the inverse
//! transformation before the output values are returned to the
//! user application.
//!
//! \sa TypeIIRMLPosition::GetNextStateOfMotion()
//  ----------------------------------------------------------
    RMLDoubleVector             *StoredTargetPosition;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MinimumExecutionTimes
//!
//! \brief
//! Vector that contains the minimum execution times in seconds,
//! \f$ \vec{t}_i^{\,min} \f$
//!
//! \sa TypeIIRMLPosition::Step1()
//  ----------------------------------------------------------
    RMLDoubleVector             *MinimumExecutionTimes;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *BeginningsOfInoperativeTimeIntervals
//!
//! \brief
//! Vector that contains the beginnings of inoperative time intervals in
//! seconds, \f$ \vec{t}_i^{\,begin} \f$
//!
//! \sa TypeIIRMLPosition::Step1()
//! \sa TypeIIRMLPosition::EndingsOfInoperativeTimeIntervals
//  ----------------------------------------------------------
    RMLDoubleVector             *BeginningsOfInoperativeTimeIntervals;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *EndingsOfInoperativeTimeIntervals
//!
//! \brief
//! Vector that contains the endings of inoperative time intervals in
//! seconds, \f$ \vec{t}_i^{\,end} \f$
//!
//! \sa TypeIIRMLPosition::Step1()
//! \sa TypeIIRMLPosition::BeginningsOfInoperativeTimeIntervals
//  ----------------------------------------------------------
    RMLDoubleVector             *EndingsOfInoperativeTimeIntervals;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *PhaseSynchronizationReferenceVector
//!
//! \brief
//! Reference vector for phase-synchronized trajectories,
//! \f$ \vec{\varrho}_i \f$ with \f$ _{\kappa}\varrho_i\,=\,1 \f$
//!
//! \sa TypeIIRMLPosition::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    RMLDoubleVector             *PhaseSynchronizationReferenceVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *PhaseSynchronizationCurrentPositionVector
//!
//! \brief
//! Current position vector \f$ \vec{P}_i \f$ used for the calculation
//! of phase-synchronized motion trajectories
//!
//! \sa TypeIIRMLPosition::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    RMLDoubleVector             *PhaseSynchronizationCurrentPositionVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *PhaseSynchronizationTargetPositionVector
//!
//! \brief
//! Target position vector \f$ \vec{P}_i^{\,trgt} \f$ used for the
//! calculation of phase-synchronized motion trajectories
//!
//! \sa TypeIIRMLPosition::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    RMLDoubleVector             *PhaseSynchronizationTargetPositionVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *PhaseSynchronizationPositionDifferenceVector
//!
//! \brief
//! Position difference vector
//! \f$ \left( \vec{P}_i^{\,trgt}\,-\,\vec{P}_i \right) \f$ used for the
//! calculation of phase-synchronized motion trajectories
//!
//! \sa TypeIIRMLPosition::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    RMLDoubleVector             *PhaseSynchronizationPositionDifferenceVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *PhaseSynchronizationCurrentVelocityVector
//!
//! \brief
//! Current velocity vector \f$ \vec{V}_i \f$ used for the
//! calculation of phase-synchronized motion trajectories
//!
//! \sa TypeIIRMLPosition::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    RMLDoubleVector             *PhaseSynchronizationCurrentVelocityVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *PhaseSynchronizationTargetVelocityVector
//!
//! \brief
//! Target velocity vector \f$ \vec{V}_i^{\,trgt} \f$ used for the
//! calculation of phase-synchronized motion trajectories
//!
//! \sa TypeIIRMLPosition::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    RMLDoubleVector             *PhaseSynchronizationTargetVelocityVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *PhaseSynchronizationMaxVelocityVector
//!
//! \brief
//! Contains the adapted maximum velocity vector
//! \f$ \left.\vec{V}_i^{\,max}\right.' \f$
//! for phase-synchronized trajectories
//!
//! \sa TypeIIRMLPosition::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    RMLDoubleVector             *PhaseSynchronizationMaxVelocityVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *PhaseSynchronizationMaxAccelerationVector
//!
//! \brief
//! Contains the adapted maximum acceleration vector
//! \f$ \left.\vec{A}_i^{\,max}\right.' \f$
//! for phase-synchronized trajectories
//!
//! \sa TypeIIRMLPosition::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    RMLDoubleVector             *PhaseSynchronizationMaxAccelerationVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *PhaseSynchronizationTimeVector
//!
//! \brief
//! A vector of execution time values in seconds for all selected degrees
//! of freedom used for phase-synchronized motion trajectories
//!
//! \sa TypeIIRMLPosition::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    RMLDoubleVector             *PhaseSynchronizationTimeVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *PhaseSynchronizationCheckVector
//!
//! \brief
//! Candidate for a reference vector \f$ \vec{\varrho}_i \f$ used for
//! phase-synchronized motion trajectories
//!
//! \sa TypeIIRMLPosition::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    RMLDoubleVector             *PhaseSynchronizationCheckVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *ArrayOfSortedTimes
//!
//! \brief
//! An array of possible synchronization times in seconds. It contains all
//! values of \f$ \vec{t}_i^{\,min} \f$, \f$ \vec{t}_i^{\,begin} \f$, and
//! \f$ \vec{t}_i^{\,end} \f$. The array contains \f$ 3\,\cdot\,K \f$ elements
//!
//! \sa TypeIIRMLMath::Quicksort()
//! \sa TypeIIRMLPosition::Step1()
//  ----------------------------------------------------------
    RMLDoubleVector             *ArrayOfSortedTimes;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *ZeroVector
//!
//! \brief
//! A vector with \f$ K \f$ \c double elements, all of which are zero
//  ----------------------------------------------------------
    RMLDoubleVector             *ZeroVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLPositionInputParameters *OldInputParameters
//!
//! \brief
//! Pointer to an RMLPositionInputParameters object.
//! In order to check, whether a new calculation has to be started, the
//! input values have to be compared to the input and output values
//! of the previous cycle. This variable is used to store the old input
//! values
//!
//! \sa RMLOutputParameters::ANewCalculationWasPerformed
//! \sa TypeIIRMLPosition::GetNextStateOfMotion()
//! \sa TypeIIRMLVelocity::OldInputParameters
//! \sa OldFlags
//! \sa \ref page_InputValues
//  ----------------------------------------------------------
    RMLPositionInputParameters  *OldInputParameters;


//  ---------------------- Doxygen info ----------------------
//! \var RMLPositionInputParameters *CurrentInputParameters
//!
//! \brief
//! Pointer to an RMLPositionInputParameters object. This object contains
//! a complete set of input values \f$ {\bf W}_i \f$
//!
//! \sa TypeIIRMLPosition::GetNextStateOfMotion()
//! \sa \ref page_InputValues
//  ----------------------------------------------------------
    RMLPositionInputParameters  *CurrentInputParameters;


//  ---------------------- Doxygen info ----------------------
//! \var RMLPositionOutputParameters *OutputParameters
//!
//! \brief
//! Pointer to an RMLPositionOutputParameters object. This object contains
//! the output parameters of the method
//! TypeIIRMLPosition::GetNextStateOfMotion(). Besides the new desired
//! state of motion \f$ {\bf M}_{i+1} \f$, further complementary values
//! for positional extremes are provided.
//!
//! \sa RMLPositionOutputParameters
//! \sa TypeIIRMLPosition::GetNextStateOfMotion()
//! \sa \ref page_OutputValues
//  ----------------------------------------------------------
    RMLPositionOutputParameters *OutputParameters;


//  ---------------------- Doxygen info ----------------------
//! \var TypeIIRMLVelocity *RMLVelocityObject
//!
//! \brief
//! Pointer to an TypeIIRMLVelocity object. The velocity-based
//! On-Line Trajectory Generation algorithm is used in the second
//! layer of the safety concept
//!
//! \sa \ref page_ErrorHandling
//! \sa TypeIIRMLPosition::FallBackStrategy()
//! \sa TypeIIRMLVelocity
//! \sa TypeIIRMLPosition::VelocityInputParameters
//! \sa TypeIIRMLPosition::VelocityOutputParameters
//! \sa TypeIIRMLPosition::VelocityFlags
//  ----------------------------------------------------------
    TypeIIRMLVelocity           *RMLVelocityObject;


//  ---------------------- Doxygen info ----------------------
//! \var RMLVelocityInputParameters *VelocityInputParameters
//!
//! \brief
//! Pointer to an RMLVelocityInputParameters object. It is used
//! for the input parameters of the velocity-based On-Line
//! Trajectory Generation algorithm called with
//! TypeIIRMLVelocity::GetNextStateOfMotion() in the
//! second safety layer
//!
//! \sa \ref page_ErrorHandling
//! \sa TypeIIRMLPosition::FallBackStrategy()
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa TypeIIRMLVelocity
//! \sa TypeIIRMLPosition::VelocityOutputParameters
//! \sa TypeIIRMLPosition::VelocityFlags
//  ----------------------------------------------------------
    RMLVelocityInputParameters  *VelocityInputParameters;


//  ---------------------- Doxygen info ----------------------
//! \var RMLVelocityOutputParameters *VelocityOutputParameters
//!
//! \brief
//! Pointer to an RMLVelocityOutputParameters object. It is used
//! for the output parameters of the velocity-based On-Line
//! Trajectory Generation algorithm called with
//! TypeIIRMLVelocity::GetNextStateOfMotion() in the
//! second safety layer
//!
//! \sa \ref page_ErrorHandling
//! \sa TypeIIRMLPosition::FallBackStrategy()
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa TypeIIRMLVelocity
//! \sa TypeIIRMLPosition::VelocityInputParameters
//! \sa TypeIIRMLPosition::VelocityFlags
//  ----------------------------------------------------------
    RMLVelocityOutputParameters *VelocityOutputParameters;


//  ---------------------- Doxygen info ----------------------
//! \var RMLVelocityFlags VelocityFlags
//!
//! \brief
//! Pointer to an RMLVelocityFlags object. It is used
//! for the velocity-based On-Line
//! Trajectory Generation algorithm called with
//! TypeIIRMLVelocity::GetNextStateOfMotion() in the
//! second safety layer
//!
//! \sa \ref page_ErrorHandling
//! \sa TypeIIRMLPosition::FallBackStrategy()
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa TypeIIRMLVelocity
//! \sa TypeIIRMLPosition::VelocityOutputParameters
//! \sa TypeIIRMLPosition::VelocityInputParameters
//  ----------------------------------------------------------
    RMLVelocityFlags            VelocityFlags;


//  ---------------------- Doxygen info ----------------------
//! \var MotionPolynomials *Polynomials
//!
//! \brief
//! Pointer to an MotionPolynomials object, which contains the actual
//! trajectory \f$ {\cal M}_i \f$. It is a two-dimensional array of
//! polynomial functions.
//!
//! \sa TypeIIRMLMath::TypeIIRMLPolynomial
//! \sa TypeIIRMLMath::MotionPolynomials
//! \sa TypeIIRMLPosition::Step2()
//! \sa TypeIIRMLPosition::Step3()
//  ----------------------------------------------------------
    MotionPolynomials           *Polynomials;


//  ---------------------- Doxygen info ----------------------
//! \var DominatValueForPhaseSync PhaseSynchronizationMagnitude
//!
//! \brief
//! Value indicating, which of input vectors was used to compute
//! the reference vector for phase-synchronization
//! TypeIIRMLPosition::PhaseSynchronizationReferenceVector
//! (\f$ \vec{\varrho}_i \f$)
//!
//! \sa DominatValueForPhaseSync
//! \sa TypeIIRMLPosition::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    DominatValueForPhaseSync    PhaseSynchronizationMagnitude;


};  // class TypeIIRMLPosition


#endif


