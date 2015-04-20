//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLVelocity.h
//!
//! \brief
//! Header file for the class TypeIIRMLVelocity
//!
//! \details
//! The class TypeIIRMLVelocity contains a derivative of the
//! velocity-based On-Line Trajectory Generation algorithm
//! of the Reflexxes Motion Libraries. The class
//! provides its interface by the the class ReflexxesAPI.
//!
//! \sa TypeIIRMLMath
//! \sa ReflexxesAPI
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


#ifndef __TypeIIRMLVelocity__
#define __TypeIIRMLVelocity__


#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLVector.h>
#include <TypeIIRMLPolynomial.h>
#include <RMLVelocityFlags.h>



using namespace TypeIIRMLMath;


//  ---------------------- Doxygen info ----------------------
//! \class TypeIIRMLVelocity
//!
//! \brief
//! <b>This class constitutes the user interface of \em velocity-based the
//! Type II On-Line Trajectory Generation algorithm</b>
//!
//! \details
//! The class has to purposes:
//!
//!  -# As an independent class, it provides the possibility to
//!     on-line generate jerk-limited motion trajectories, which
//!     guide all selected degrees of freedom to desired target velocity
//!     vectors \f$ \vec{V}_i^{\,trgt} \f$ (indepentendly of any
//!     position \f$ \vec{P}_i^{\,trgt} \f$. This can be useful, for
//!     example to approach for an expected contact with a certain
//!     (limited) velocity, such that the force controller can be
//!     turned on in the moment of contact.
//!  -# This algorithm constitutes the second layer of the safety concept
//!     of the Reflexxes Motion Libraries. If the position-based
//!     algorithm (cf. TypeIIRMLPosition) is not capable to provide the
//!     correct solution (e.g., due to invalid input values), this
//!     algorithm will guide all selected degrees of freedom to a
//!     certain velocity vector (commonly the zero vector, but another
//!     target velocity vector can be specified as well, cf.
//!     RMLPositionFlags::KeepCurrentVelocityInCaseOfFallbackStrategy).
//!
//! The algorithm is capable to generate phase-synchronized and
//! non-synchronized motions. The only interface for the user
//! application is the method
//!
//! \sa TypeIIRMLPosition
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa RMLVelocityInputParameters
//! \sa RMLVelocityOutputParameters
//! \sa ReflexxesAPI
//! \sa \ref page_ErrorHandling
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
class TypeIIRMLVelocity
{
public:


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIIRMLVelocity(const unsigned int &DegreesOfFreedom, const double &CycleTimeInSeconds)
//!
//! \brief
//! Constructor of the class TypeIIRMLVelocity
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
//! \sa TypeIIRMLVelocity::TypeIIRMLVelocity()
//! \sa TypeIIRMLPosition::TypeIIRMLPosition()
//  ----------------------------------------------------------
    TypeIIRMLVelocity(      const unsigned int  &DegreesOfFreedom
                        ,   const double        &CycleTimeInSeconds);


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIIRMLVelocity(const TypeIIRMLVelocity &TypeIIRMLObject)
//!
//! \brief
//! Copy constructor of class TypeIIRMLVelocity
//!
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//!
//! \param TypeIIRMLObject
//! Object to be copied
//!
//! \sa TypeIIRMLVelocity(const unsigned int &DegreesOfFreedom, const double &CycleTimeInSeconds)
//  ----------------------------------------------------------
    TypeIIRMLVelocity(const TypeIIRMLVelocity &TypeIIRMLObject);


//  ---------------------- Doxygen info ----------------------
//! \fn ~TypeIIRMLVelocity(void)
//!
//! \brief
//! Destructor of the class TypeIIRMLVelocity
//  ----------------------------------------------------------
    ~TypeIIRMLVelocity(void);


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIIRMLVelocity &operator = (const TypeIIRMLVelocity &TypeIIRMLObject)
//!
//! \brief
//! Copy operator
//!
//! \param TypeIIRMLObject
//! TypeIIRMLVelocity object to be copied
//  ----------------------------------------------------------
    TypeIIRMLVelocity &operator = (const TypeIIRMLVelocity &TypeIIRMLObject);


//  ---------------------- Doxygen info ----------------------
//! \fn int GetNextStateOfMotion(const RMLVelocityInputParameters &InputValues, RMLVelocityOutputParameters *OutputValues, const RMLVelocityFlags &Flags)
//!
//! \brief
//! <b>The main method of the class TypeIIRMLVelocity. It executes
//! the velocity-based Type II On-Line Trajectory Generation algorithm</b>
//!
//! \details
//! Given a set of \c InputValues consisting of
//!
//!  - a current state of motion \f$ {\bf M}_i \f$ at intstant \f$ T_i \f$,
//!  - a target velocity vector \f$ \vec{V}_i^{\,trgt} \f$ at intstant
//!     \f$ T_i \f$ (with zero acceleration),
//!  - kinematic motion constraints \f$ {\bf B}_i \f$ at intstant
//!    \f$ T_i \f$, and
//!  - a selection vector \f$ \vec{S}_i \f$ at intstant \f$ T_i \f$
//!
//! and a set of boolean \c Flags to control the behavior of the algorithm,
//! this method executes the velocity-based Type II On-Line Trajectory
//! Generation algorithm and provides a set of \c OutputValues, which
//! contain
//!
//!  - the desired state of motion \f$ {\bf M}_{i+1} \f$ at intstant
//!    \f$ T_{i+1} \f$ and
//!  - (optionally) further complementary values of the current trajectory.
//!
//! For a detailed description, please refer to TypeIIRMLVelocity and
//! to the start page \ref index.
//!
//! \param InputValues
//! Input values of the velocity-based Type II On-Line Trajectory
//! Generation algorithm. For detailed information, please refer to the
//! class RMLVelocityInputParameters and to the page \ref page_InputValues.
//!
//! \param OutputValues
//! Output values of the velocity-based Type II On-Line Trajectory
//! Generation algorithm. For detailed information, please refer to the
//! class RMLVelocityOutputParameters and to the page
//! \ref page_OutputValues.
//!
//! \param Flags
//! A set of boolean values to configure the behavior of the algorithm
//! (e.g., specify whether a time- or a phase-synchronized trajectory is
//! desired, specify, whether the complementary output values are supposed
//! to be computed). For a detailed description of this data structure and
//! its usage, please refer to RMLVelocityFlags.
//!
//! \return
//! An element of ReflexxesAPI::RMLResultValue:\n\n
//!  - ReflexxesAPI::RML_WORKING: \copydoc ReflexxesAPI::RML_WORKING\n\n
//!  - ReflexxesAPI::RML_FINAL_STATE_REACHED: \copydoc ReflexxesAPI::RML_FINAL_STATE_REACHED\n\n
//!  - ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES: \copydoc ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES\n\n
//!  - ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS: \copydoc ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS\n\n
//!  - ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION: \copydoc ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION\n\n
//!  - ReflexxesAPI::RML_ERROR_NULL_POINTER: \copydoc ReflexxesAPI::RML_ERROR_NULL_POINTER\n\n
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
//! \sa TypeIIRMLVelocity
//! \sa RMLVelocityInputParameters
//! \sa RMLVelocityOutputParameters
//! \sa RMLVelocityFlags
//! \sa ReflexxesAPI
//! \sa TypeIIRMLPosition::GetNextStateOfMotion()
//! \sa TypeIIRMLVelocity::GetNextStateOfMotionAtTime()
//! \sa TypeIIRMLMath
//! \sa \ref page_RealTimeBehavior
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    int GetNextStateOfMotion(       const RMLVelocityInputParameters    &InputValues
                                ,   RMLVelocityOutputParameters         *OutputValues
                                ,   const RMLVelocityFlags              &Flags);


//  ---------------------- Doxygen info ----------------------
//! \fn int GetNextStateOfMotionAtTime(const double &TimeValueInSeconds, RMLVelocityOutputParameters *OutputValues) const
//!
//! \brief
//! Once the method of TypeIIRMLVelocity::GetNextStateOfMotion() was
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
//! RMLVelocityOutputParameters).\n
//! \n
//! If the method TypeIIRMLVelocity::GetNextStateOfMotion() returned an
//! error, the same error will be returned by this method. The value of
//! \c TimeValueInSeconds has to be positive and below the values of
//! RML_MAX_EXECUTION_TIME (\f$ 10^{10} \f$ seconds).\n
//! \n
//! For further information, please refer to the documentation of
//! TypeIIRMLVelocity::GetNextStateOfMotion().
//!
//! \param TimeValueInSeconds
//! Time value in seconds, at which the desired state of motion is
//! calculated.
//!
//! \param OutputValues
//! Output values of the velocity-based Type II On-Line Trajectory
//! Generation algorithm. For detailed information, please refer to the
//! class RMLVelocityOutputParameters and to the page
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
//!  - ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE: \copydoc ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE\n\n
//!
//! \sa TypeIIRMLVelocity
//! \sa RMLVelocityOutputParameters
//! \sa RMLVelocityFlags
//! \sa ReflexxesAPI
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa TypeIIRMLPosition::GetNextStateOfMotionAtTime()
//  ----------------------------------------------------------
    int GetNextStateOfMotionAtTime(     const double                        &TimeValueInSeconds
                                    ,   RMLVelocityOutputParameters         *OutputValues       ) const;


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
//! \fn void FallBackStrategy(const RMLVelocityInputParameters &InputValues, RMLVelocityOutputParameters *OutputValues)
//!
//! \brief
//! In case of an error, this method triggers the next layer of the
//! safety concept
//!
//! \details
//! Depending on how the class TypeIIRMLVelocity is used, two different
//! purposes are achieved by this method.
//!
//!  -# If an object of TypeIIRMLVelocity is used independently from the
//!     class TypeIIRMLPosition, this is the second layer of the Reflexxes
//!     safety concept. For the case that the velocity-based On-Line
//!     Trajectory Generation algorithm is not able to compute correct
//!     output values (e.g., because of wrong input values, this fall-back
//!     method is called, such that a valid state of motion is provided in
//!     \em any case.
//!  -# If the object of TypeIIRMLVelocity is an attribute of the class
//!     TypeIIRMLPosition, this is the third layer of the Reflexxes
//!     safety concept. If no trajectory can be calculated by the
//!     position-based On-Line Trajectory Generation algorithm
//!     (TypeIIRMLPosition::GetNextStateOfMotion()), the velocity-based
//!     algorithm (TypeIIRMLVelocity::GetNextStateOfMotion()) is called in
//!     the second safety layer. For the case, that this method is also not
//!     able to compute output values, TypeIIRMLVelocity::FallBackStrategy()
//!     ensures valid output values in \em any case.
//!
//! This method uses the current state of motion \f$ {\bf M}_i \f$ and
//! continues the current motion with \f$ \vec{A}_{i}\,=\,\vec{0} \f$.
//! Concretely, \f$ {\bf M}_{i+1} \f$ is calculated this way:
//!
//!  - \f$ \vec{V}_{i+1}\,=\,\vec{V}_{i} \f$
//!  - \f$ \vec{P}_{i+1}\,=\,\vec{P}_{i}\,+\,\vec{V}_{i}\,\cdot\,T^{\,cycle} \f$
//!
//! \n
//! A detailed description of the three-layered safety mechanism of the
//! Reflexxes Motion Libraries can be found at \ref page_ErrorHandling.
//!
//! \param InputValues
//! Current set of input parameters
//!
//! \param OutputValues
//! Current set of output parameters the will be generated by this method
//!
//! \sa TypeIIRMLVelocity
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa TypeIIRMLPosition::GetNextStateOfMotion()
//! \sa TypeIIRMLVelocity::OutputParameters
//! \sa TypeIIRMLVelocity::ReturnValue
//! \sa TypeIIRMLPosition::FallBackStrategy
//! \sa \ref page_ErrorHandling
//  ----------------------------------------------------------
    void FallBackStrategy(      const RMLVelocityInputParameters    &InputValues
                            ,   RMLVelocityOutputParameters         *OutputValues   );


//  ---------------------- Doxygen info ----------------------
//! \fn void CalculateExecutionTimes(void)
//!
//! \brief
//! Calculates the minimum execution times and corresponding motion
//! profiles of each selected degree of freedom
//!
//! \details
//! This method selects an velocity profile of the set
//! RMLMath::FinalAccelerationProfilesForVelocityCtrl and computes the
//! minimum execution for each single degree of freedom in seconds and
//! writes the values into the attributes
//!
//!  - TypeIIRMLVelocity::MotionProfiles
//!  - TypeIIRMLVelocity::ExecutionTimes
//!
//! This computation step is comparable to Step 1 of the position-based
//! On-line Trajectory Generation algorithm.
//!
//! \sa TypeIIRMLPosition::Step1()
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//  ----------------------------------------------------------
    void CalculateExecutionTimes(void);


//  ---------------------- Doxygen info ----------------------
//! \fn void ComputeTrajectoryParameters(void)
//!
//! \brief
//! Computes all trajectory parameters for non- and phase-
//! synchronized trajectories
//!
//! \details
//! This method computes the coefficients of all pieces of polynomials that
//! are used to represent the trajectory. This part is comparable to
//! Step 2 of the position-based On-Line Trajectory Generation algorithm.
//! Based on the profile that was determined by the method
//! TypeIIRMLVelocity::CalculateExecutionTimes() (and the change of
//! kinematic motion constraints in case of phase-synchronized trajectories;
//! cf. TypeIIRMLVelocity::ComputePhaseSynchronizationParameters()), the
//! actual parameters are set-up, such that a new state of motion can be
//! computed by calling TypeIIRMLVelocity::ComputeAndSetOutputParameters().
//!
//! \sa TypeIIRMLPosition::Step2()
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//  ----------------------------------------------------------
    void ComputeTrajectoryParameters(void);


//  ---------------------- Doxygen info ----------------------
//! \fn void ComputePhaseSynchronizationParameters(void)
//!
//! \brief
//! Checks, whether phase-synchronization is possible. If possible
//! the corresponding vectors are computed
//!
//! \details
//! This method checks, whether a phase-synchronized trajectory can be
//! generated. If all input vectors are collinear, it is checks,
//! a homothetic trajectory can be computed that meets all kinematic
//! motion constraints. If this is the case, the constraints will be
//! adapted such that
//! \f$ \vec{A}_i^{\,max} \f$ is also collinear to all other input
//! values. This vectors is stored in the attribute
//!
//!  - TypeIIRMLVelocity::PhaseSynchronizationMaxAccelerationVector.
//!
//! \sa TypeIIRMLVelocity::IsPhaseSynchronizationPossible()
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//  ----------------------------------------------------------
    void ComputePhaseSynchronizationParameters(void);


//  ---------------------- Doxygen info ----------------------
//! \fn void ComputeTimeSynchronizedTrajectoryParameters(void)
//!
//! \brief
//! Computes all trajectory parameters for time-synchronized trajectories
//!
//! \details
//! This method computes the coefficients of all pieces of polynomials that
//! are used to represent the trajectory. This part is comparable to
//! Step 2 of the position-based On-Line Trajectory Generation algorithm.
//!
//! \sa TypeIIRMLPosition::Step2()
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa TypeIIRMLVelocity::ComputeTrajectoryParameters()
//  ----------------------------------------------------------
    void ComputeTimeSynchronizedTrajectoryParameters(void);


//  ---------------------- Doxygen info ----------------------
//! \fn int ComputeAndSetOutputParameters(const double &TimeValueInSeconds, RMLVelocityOutputParameters *OP) const
//!
//! \brief
//! Computes the output values of the velocity-based On-line Trajectory
//! Generation algorithm, that is, the state of motion for the next control
//! cycle
//!
//! \details
//! After either
//!
//!  - TypeIIRMLVelocity::ComputeTrajectoryParameters() or
//!  - TypeIIRMLVelocity::ComputeTimeSynchronizedTrajectoryParameters() or
//!
//! calculated all coefficients of the polynomials that piecewise describe
//! the desired trajectory, this method computes the actual output values of
//! algorithm, that is, the state of motion at the instant
//! \c TimeValueInSeconds (commonly for the next control cycle). This
//! part is the pendent of TypeIIRMLPosition::Step3() in the position-based
//! On-line Trajectory Generation algorithm. The resulting values are
//! written into the object pointed to by \c OP (output parameters).
//!
//! The methods TypeIIRMLVelocity::GetNextStateOfMotion() and
//! TypeIIRMLVelocity::GetNextStateOfMotionAtTime() make use of this
//! functionality.
//!
//! \param TimeValueInSeconds
//! Time value in seconds, at which the desired state of motion is calculated.
//!
//! \param OP
//! Pointer to an object of the class RMLVelocityOutputParameters, to which
//! the resulting output values will be written to.
//!
//! \sa TypeIIRMLPosition::Step3()
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa TypeIIRMLVelocity::GetNextStateOfMotionAtTime()
//  ----------------------------------------------------------
    int ComputeAndSetOutputParameters(      const double                    &TimeValueInSeconds
                                        ,   RMLVelocityOutputParameters     *OP                 ) const;

//  ---------------------- Doxygen info ----------------------
//! \fn bool IsPhaseSynchronizationPossible(void)
//!
//! \brief
//! Checks, whether the motion trajectory can be phase-synchronized
//!
//! \details
//! It is checked whether the
//! trajectory can be phase-synchronized. Therefore, this method checks
//! whether the input vectors
//!
//!  - current velocity vector \f$ \vec{V}_{i} \f$ and
//!  - target velocity vector \f$ \vec{V}_{i}^{\,trgt} \f$
//!
//! are collinear to each other. If this is the case,
//!
//!  - \c true will be returned, and
//!  - the reference vector \c ReferenceVector (\f$ \vec{\varrho}_i \f$)
//!    will be calculated, and copied to
//!    TypeIIRMLVelocity::PhaseSynchronizationReferenceVector.
//!
//! If this is not the case,
//!
//!  - \c false will be returned, and
//!  - all elements of
//!    TypeIIRMLVelocity::PhaseSynchronizationReferenceVector are set to
//!    zero.
//!
//! For all these computations, the attributes
//!
//!  - TypeIIRMLPosition::PhaseSynchronizationReferenceVector
//!  - TypeIIRMLPosition::PhaseSynchronizationCurrentVelocityVector
//!  - TypeIIRMLPosition::PhaseSynchronizationTargetVelocityVector
//!  - TypeIIRMLPosition::PhaseSynchronizationMaxAccelerationVector
//!
//! are used.
//!
//! \return
//!  - \c true if phase-synchronization is possible
//!  - \c false otherwise
//!
//! \sa TypeIIRMLVelocity::ComputePhaseSynchronizationParameters()
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa TypeIIRMLPosition::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    bool IsPhaseSynchronizationPossible(void);


//  ---------------------- Doxygen info ----------------------
//! \fn void CalculatePositionalExtrems(const double &TimeValueInSeconds, RMLVelocityOutputParameters *OP) const
//!
//! \brief
//! Set all positional extremum parameters of the output values of the
//! algorithm (TypeIIRMLVelocity::OutputParameters)
//!
//! \details
//! After all trajectory parameters \f$ {\cal M}_{i}(t) \f$ have been
//! computed in
//! TypeIIRMLVelocity::GetNextStateOfMotion(), they are stored in the
//! attribute TypeIIRMLVelocity::Polynomials. Using this attribute, this
//! method computes all positional extremum values and corresponding states
//! of motion and writes the results to TypeIIRMLVelocity::OutputParameters.
//! In particular, the following values are calculated:
//!
//!  - RMLVelocityOutputParameters::MaxPosExtremaPositionVectorOnly
//!  - RMLVelocityOutputParameters::MinPosExtremaPositionVectorOnly
//!  - RMLVelocityOutputParameters::MaxExtremaTimesVector
//!  - RMLVelocityOutputParameters::MaxPosExtremaPositionVectorArray
//!  - RMLVelocityOutputParameters::MaxPosExtremaVelocityVectorArray
//!  - RMLVelocityOutputParameters::MinExtremaTimesVector
//!  - RMLVelocityOutputParameters::MinPosExtremaPositionVectorArray
//!  - RMLVelocityOutputParameters::MinPosExtremaVelocityVectorArray
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
//! Pointer to an object of the class RMLVelocityOutputParameters. The
//! positional extreme values will be calculated for these data.
//!
//! \note
//! The calculation of these values can be disabled by setting the flag
//! RMLVelocityFlags::EnableTheCalculationOfTheExtremumMotionStates to
//! \c false when the method TypeIIRMLVelocity::GetNextStateOfMotion() is
//! called.
//!
//! \sa \ref page_OutputValues
//! \sa RMLVelocityOutputParameters
//! \sa RMLVelocityFlags::EnableTheCalculationOfTheExtremumMotionStates
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa TypeIIRMLVelocity::SetPositionalExtremsToZero()
//! \sa TypeIIRMLPosition::CalculatePositionalExtrems
//  ----------------------------------------------------------
    void CalculatePositionalExtrems(    const double                &TimeValueInSeconds
                                    ,   RMLVelocityOutputParameters *OP                 ) const;


//  ---------------------- Doxygen info ----------------------
//! \fn void SetPositionalExtremsToZero(void) const
//!
//! \brief
//! Set all positional extremum parameters of the output values of the
//! algorithm (TypeIIRMLVelocity::OutputParameters) to zero
//!
//! \details
//! If the input flag
//! RMLVelocityFlags::EnableTheCalculationOfTheExtremumMotionStates is set
//! to \c false, this method is used to set all output values that are
//! related to the calculation of the positional extremum values to zero
//! in order to obtain defined output values:
//!
//!  - RMLVelocityOutputParameters::MaxPosExtremaPositionVectorOnly
//!  - RMLVelocityOutputParameters::MinPosExtremaPositionVectorOnly
//!  - RMLVelocityOutputParameters::MaxExtremaTimesVector
//!  - RMLVelocityOutputParameters::MaxPosExtremaPositionVectorArray
//!  - RMLVelocityOutputParameters::MaxPosExtremaVelocityVectorArray
//!  - RMLVelocityOutputParameters::MinExtremaTimesVector
//!  - RMLVelocityOutputParameters::MinPosExtremaPositionVectorArray
//!  - RMLVelocityOutputParameters::MinPosExtremaVelocityVectorArray
//!
//! If the input flag
//! RMLVelocityFlags::EnableTheCalculationOfTheExtremumMotionStates
//! is set to \c true, the method
//! TypeIIRMLVelocity::CalculatePositionalExtrems() is used to compute
//! this part of the output values.
//!
//! \param OP
//! Pointer to an object of the class RMLVelocityOutputParameters. The
//! values of this data structure will be set to zero.
//!
//! \sa \ref page_OutputValues
//! \sa RMLPositionOutputParameters
//! \sa RMLPositionFlags::EnableTheCalculationOfTheExtremumMotionStates
//! \sa TypeIIRMLPosition::GetNextStateOfMotion()
//! \sa TypeIIRMLPosition::CalculatePositionalExtrems()
//  ----------------------------------------------------------
    void SetPositionalExtremsToZero(RMLVelocityOutputParameters *OP) const;


//  ---------------------- Doxygen info ----------------------
//! \fn void SetupPhaseSyncSelectionVector(void)
//!
//! \brief
//! Modify the current selection vector and exclude unnecessary
//! degrees of freedom
//!
//! \details
//! This method modifies the selection vector
//! RMLVelocityInputParameters::SelectionVector of
//! TypeIIRMLVelocity::CurrentInputParameters to
//! TypeIIRMLVelocity::PhaseSyncSelectionVector.
//! Degrees of freedom \f$ k \f$, whose
//!
//!  - current velocity \f$ _kV_i \f$ and
//!  - target velocity \f$ _kV_i^{\,trgt} \f$
//!
//! are zero, can be excluded from the selection vector \f$ \vec{S}_i \f$
//! in order correctly determine, whether phase-synchronization is
//! possible and to ensure numerical stability.
//!
//! \sa RMLVelocityInputParameters::SelectionVector
//! \sa TypeIIRMLVelocity::PhaseSyncSelectionVector
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa TypeIIRMLPosition::SetupModifiedSelectionVector()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    void SetupPhaseSyncSelectionVector(void);


//  ---------------------- Doxygen info ----------------------
//! \var bool CurrentTrajectoryIsPhaseSynchronized
//!
//! \brief
//! Indicates, whether the current trajectory is phase-synchronized
//!
//! \sa TypeIIRMLVelocity::IsPhaseSynchronizationPossible()
//! \sa RMLVelocityOutputParameters::TrajectoryIsPhaseSynchronized
//! \sa TypeIIRMLPosition::CurrentTrajectoryIsPhaseSynchronized
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    bool                        CurrentTrajectoryIsPhaseSynchronized;


//  ---------------------- Doxygen info ----------------------
//! \var bool CurrentTrajectoryIsNotSynchronized
//!
//! \brief
//! Indicates, that the current trajectory is  not synchronized
//!
//! \sa RMLFlags::NO_SYNCHRONIZATION
//! \sa TypeIIRMLPosition::CurrentTrajectoryIsNotSynchronized
//! \sa \ref page_SynchronizationBehavior
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
//! TypeIIRMLVelocity::GetNextStateOfMotion()
//!
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//  ----------------------------------------------------------
    int                         ReturnValue;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int NumberOfDOFs
//!
//! \brief
//! The number of degrees of freedom \f$ K \f$
//!
//! \sa TypeIIRMLVelocity::TypeIIRMLVelocity()
//  ----------------------------------------------------------
    unsigned int                NumberOfDOFs;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int DOFWithGreatestExecutionTime
//!
//! \brief
//! Index of the degree of freedom that requires the greatest execution time
//!
//! \sa TypeIIRMLVelocity::TypeIIRMLVelocity()
//  ----------------------------------------------------------
    unsigned int                DOFWithGreatestExecutionTime;


//  ---------------------- Doxygen info ----------------------
//! \var double CycleTime
//!
//! \brief
//! Contains the cycle time in seconds
//!
//! \sa TypeIIRMLVelocity::TypeIIRMLVelocity()
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
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//  ----------------------------------------------------------
    double                      InternalClockInSeconds;


//  ---------------------- Doxygen info ----------------------
//! \var RMLVelocityFlags OldFlags
//!
//! \brief
//! In order to check, whether a new calculation has to be started, the
//! input values have to be compared to the input and output values
//! of the previous cycle. This variable is used to store the flags of
//! last cycle
//!
//! \sa RMLOutputParameters::ANewCalculationWasPerformed
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa TypeIIRMLPosition::OldFlags
//! \sa OldInputParameters
//! \sa \ref page_InputValues
//  ----------------------------------------------------------
    RMLVelocityFlags            OldFlags;


//  ---------------------- Doxygen info ----------------------
//! \var RMLBoolVector *PhaseSyncSelectionVector
//!
//! \brief
//! Boolean vector, which contains the modified selection vector that is
//! based on the original selection vector \f$ \vec{S}_i \f$
//! in order to enable numerically robust phase-synchronization
//!
//! \sa TypeIIRMLVelocity::SetupPhaseSyncSelectionVector()
//! \sa RMLVelocityInputParameters::SelectionVector
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    RMLBoolVector               *PhaseSyncSelectionVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLIntVector *ExecutionTimes
//!
//! \brief
//! Vector of \c double values, each of which represents an
//! execution time that is used internally
//!
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//  ----------------------------------------------------------
    RMLDoubleVector             *ExecutionTimes;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *PhaseSynchronizationReferenceVector
//!
//! \brief
//! Reference vector for phase-synchronized trajectories,
//! \f$ \vec{\varrho}_i \f$ with \f$ _{\kappa}\varrho_i\,=\,1 \f$
//!
//! \sa TypeIIRMLVelocity::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    RMLDoubleVector             *PhaseSynchronizationReferenceVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *PhaseSynchronizationCurrentVelocityVector
//!
//! \brief
//! Current velocity vector \f$ \vec{V}_i \f$ used for the
//! calculation of phase-synchronized motion trajectories
//!
//! \sa TypeIIRMLVelocity::IsPhaseSynchronizationPossible()
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
//! \sa TypeIIRMLVelocity::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    RMLDoubleVector             *PhaseSynchronizationTargetVelocityVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *PhaseSynchronizationMaxAccelerationVector
//!
//! \brief
//! Contains the adapted maximum acceleration vector
//! \f$ \left.\vec{A}_i^{\,max}\right.' \f$
//! for phase-synchronized trajectories
//!
//! \sa TypeIIRMLVelocity::IsPhaseSynchronizationPossible()
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    RMLDoubleVector             *PhaseSynchronizationMaxAccelerationVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLVelocityInputParameters *OldInputParameters
//!
//! \brief
//! Pointer to an RMLVelocityInputParameters object.
//! In order to check, whether a new calculation has to be started, the
//! input values have to be compared to the input and output values
//! of the previous cycle. This variable is used to store the old input
//! values
//!
//! \sa RMLOutputParameters::ANewCalculationWasPerformed
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa TypeIIRMLPosition::OldInputParameters
//! \sa OldFlags
//! \sa \ref page_InputValues
//  ----------------------------------------------------------
    RMLVelocityInputParameters  *OldInputParameters;


//  ---------------------- Doxygen info ----------------------
//! \var RMLVelocityInputParameters *CurrentInputParameters
//!
//! \brief
//! Pointer to an RMLVelocityInputParameters object. This object contains
//! a complete set of input values \f$ {\bf W}_i \f$
//!
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa \ref page_InputValues
//  ----------------------------------------------------------
    RMLVelocityInputParameters  *CurrentInputParameters;


//  ---------------------- Doxygen info ----------------------
//! \var RMLVelocityOutputParameters *OutputParameters
//!
//! \brief
//! Pointer to an RMLVelocityOutputParameters object. This object contains
//! the output parameters of the method
//! TypeIIRMLVelocity::GetNextStateOfMotion(). Besides the new desired
//! state of motion \f$ {\bf M}_{i+1} \f$, further complementary values
//! for positional extremes are provided.
//!
//! \sa RMLVelocityOutputParameters
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//! \sa \ref page_OutputValues
//  ----------------------------------------------------------
    RMLVelocityOutputParameters *OutputParameters;



//  ---------------------- Doxygen info ----------------------
//! \var MotionPolynomials *Polynomials
//!
//! \brief
//! Pointer to an array of MotionPolynomials objects, which contains the
//! actual trajectory \f$ {\cal M}_i \f$. It is a two-dimensional array of
//! polynomial functions.
//!
//! \sa TypeIIRMLMath::TypeIIRMLPolynomial
//! \sa TypeIIRMLMath::MotionPolynomials
//! \sa TypeIIRMLVelocity::GetNextStateOfMotion()
//  ----------------------------------------------------------
    MotionPolynomials           *Polynomials;


};  // class TypeIIRMLVelocity


#endif


