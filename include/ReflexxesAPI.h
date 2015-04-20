//  ---------------------- Doxygen info ----------------------
//! \file ReflexxesAPI.h
//!
//! \brief
//! Header file for the class ReflexxesAPI (API of the Reflexxes Motion Libraries)
//!
//! \copydetails ReflexxesAPI
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


#ifndef __ReflexxesAPI__
#define __ReflexxesAPI__


#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <RMLPositionFlags.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLVelocityFlags.h>
#include <RMLVector.h>


//  ---------------------- Doxygen info ----------------------
//! \class ReflexxesAPI
//!
//! \brief
//! <b>This class constitutes the API of the Reflexxes Motion
//! Libraries</b>
//!
//! \details
//! The class ReflexxesAPI is the interface between the actual On-Line
//! Trajectory Generation algorithms of the Reflexxes Motion Libraries and
//! the applications using it. It wraps the classes of the algorithms,
//! provides a simple and clean interface, and it
//! hides all parts of the implementation that or not relevant the
//! actual implementation behind its interface.\n
//! \n
//! The only two relevant functions of a <em>Reflexxes Motion Library</em> are the
//! the methods
//!
//!  - ReflexxesAPI::RMLPosition(), which calls TypeIVRMLPosition::GetNextStateOfMotion()
//!    of the class TypeIVRMLPosition, to execute the
//!    <b>position-based</b> On-Line Trajectory Generation algorithm and\n
//!
//!  - ReflexxesAPI::RMLVelocity(), which calls TypeIVRMLVelocity::GetNextStateOfMotion()
//!    of the class TypeIVRMLVelocity, to execute the <b>velocity-based
//!    </b> On-Line Trajectory Generation algorithm,
//!
//! both of which calculate a new state of motion from any arbitrary
//! initial state of motion.\n
//! \n
//! Both methods, require the specification of a set of input values
//!
//!  - RMLPositionInputParameters / RMLVelocityInputParameters
//!    (cf. \ref page_InputValues)
//!
//! and a data structure specifying a certain behavior of the algorithm
//!
//!  - RMLPositionFlags / RMLVelocityFlags.
//!
//! Based on these input values, the result, that is, the state of motion
//! for the succeeding control cycle, is written to an
//!
//!  - RMLPositionOutputParameters / RMLVelocityOutputParameters
//!    (cf. \ref page_OutputValues)
//!
//! object.\n
//!
//! To get an overview about the source code, please refer to the
//! \ref page_SourceCode. A brief survey of the Reflexxes algorithm
//! can be found at the page \ref page_TypeIIAndIVOverview, and error-handling
//! procedures are described at \ref page_ErrorHandling.\n
//!
//! Besides the classes for the input and output parameters, this class is
//! is the only one that has to be embedded in a user application. The
//! interface only consists of the constructor and the two methods
//! RMLPosition() and RMLVelocity(), such that an integration into
//! existing systems can be done in a clear and straight-forward way.
//!
//! \sa RMLPositionInputParameters
//! \sa RMLVelocityInputParameters
//! \sa RMLPositionOutputParameters
//! \sa RMLVelocityOutputParameters
//! \sa RMLPositionFlags
//! \sa RMLVelocityFlags
//! \sa \ref page_GettingStarted
//! \sa \ref page_SourceCode
//! \sa \ref page_TypeIIAndIVOverview
//! \sa \ref page_ErrorHandling
//  ----------------------------------------------------------
class ReflexxesAPI
{
public:

//  ---------------------- Doxygen info ----------------------
//! \fn ReflexxesAPI(const unsigned int &DegreesOfFreedom, const double &CycleTimeInSeconds, const unsigned int &NumberOfAdditionalThreads = 0)
//!
//! \brief
//! Constructor of the API class ReflexxesAPI (API of the Reflexxes Motion
//! Library)
//!
//! \details
//! The constructor of the class ReflexxesAPI sets the respective attributes
//! ReflexxesAPI::CycleTime, ReflexxesAPI::NumberOfDOFs, and
//! ReflexxesAPI::NumberOfOwnThreads, and it
//! creates two objects, one for position-based (TypeIVRMLPosition) and one for
//! velocity-based (TypeIVRMLVelocity) On-Line Trajectory Generation.
//!
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//!
//! \param DegreesOfFreedom
//! Specifies the number of degrees of freedom ReflexxesAPI::NumberOfDOFs
//!
//! \param CycleTimeInSeconds
//! Specifies the cycle time of the On-Line Trajectory Algorithms in seconds
//! ReflexxesAPI::CycleTime
//!
//! \param NumberOfAdditionalThreads
//! \if RMLTYPEIV It can be reasonable to parallelize the position-based On-Line Trajectory
//! Generation algorithm. This number specifies the number of additionally
//! created threads for the calculations of the algorithm
//! (cf. ReflexxesAPI::NumberOfOwnThreads). Depending on the
//! number of degrees of freedom of the robotic system and on the number of
//! available CPU cores a significant performance benefit can be achieved.
//! The best results would be achieved if the number of total threads
//! equals the number of available CPU cores (e.g., three additional threads
//! for four CPU cores). The default number of additional threads is zero,
//! such that only the calling thread is used for algorithmic computations.
//! \endif \if RMLTYPEII
//! This parameter is \em not supported for the Type II Reflexxes Motion
//! Library.
//! \endif
//!
//! \if CUSTOMER
//! \note
//! The automatic creation of additional thread is disabled by default. In
//! order use multiple threads, the compiler flag
//! REFLEXXES_USE_MULTI_THREADING has to be used.
//! \endif
//!
//! \if CUSTOMER \sa TypeIVRMLPosition \endif
//! \if CUSTOMER \sa TypeIVRMLVelocity \endif
//! \if CUSTOMER \sa TypeIVRMLPosition::TypeIVRMLPosition() \endif
//! \if CUSTOMER \sa TypeIVRMLVelocity::TypeIVRMLVelocity() \endif
//! \if RMLTYPEII \sa TypeIIRMLPosition \endif
//! \if RMLTYPEII \sa TypeIIRMLVelocity \endif
//! \if RMLTYPEII \sa TypeIIRMLPosition::TypeIIRMLPosition() \endif
//! \if RMLTYPEII \sa TypeIIRMLVelocity::TypeIIRMLVelocity() \endif
//! \sa ReflexxesAPI::NumberOfDOFs
//! \sa ReflexxesAPI::NumberOfOwnThreads
//! \sa ReflexxesAPI::CycleTime
//! \sa ReflexxesAPI::RMLPosition
//! \sa ReflexxesAPI::RMLVelocity
//! \sa \ref page_RealTimeBehavior
//  ----------------------------------------------------------
    ReflexxesAPI(       const unsigned int  &DegreesOfFreedom
                    ,   const double        &CycleTimeInSeconds
                    ,   const unsigned int  &NumberOfAdditionalThreads = 0);


//  ---------------------- Doxygen info ----------------------
//! \fn ~ReflexxesAPI(void)
//!
//! \brief
//! Destructor of the class ReflexxesAPI
//!
//! \details
//! The destructor deletes the two objects for position-based and for
//! velocity-based On-Line Trajectory Generation. If the constructor
//! of the member object ReflexxesAPI::RMLPosition started additional
//! threads, these threads will join the master thread again.
//!
//! \if RMLTYPEII \sa TypeIIRMLPosition::~TypeIIRMLPosition() \endif
//! \if RMLTYPEII \sa TypeIIRMLVelocity::~TypeIIRMLVelocity() \endif
//! \if CUSTOMER \sa TypeIVRMLPosition::~TypeIVRMLPosition() \endif
//! \if CUSTOMER \sa TypeIVRMLVelocity::~TypeIVRMLVelocity() \endif
//  ----------------------------------------------------------
    ~ReflexxesAPI(void);


//  ---------------------- Doxygen info ----------------------
//! \enum RMLResultValue
//!
//! \brief
//! Result values of the On-Line Trajectory Generation algorithm
//!
//! \details
//! This enumeration contains all possible return values of the methods
//! ReflexxesAPI::RMLPosition() and ReflexxesAPI::RMLVelocity(). Except
//! ReflexxesAPI::RML_WORKING and ReflexxesAPI::RML_FINAL_STATE_REACHED,
//! none the values must appear in a correct
//! implementation. The input values of the algorithm are tested by the
//! methods RMLPositionInputParameters::CheckForValidity() and
//! RMLVelocityInputParameters::CheckForValidity().\n
//! \n
//! If either one of these
//! methods returns \c false, a correct calculation of output values can not
//! necessarily be obtained anymore, but - in any case - a valid and
//! continuous motion trajectory will be generated. In such a case, the
//! library will try to calculate correct output values, and in many cases
//! the library is capable to provide such values successfully, but a
//! guarantee cannot be given (cf. \ref page_ErrorHandling).\n
//! \n
//! If RMLPositionInputParameters::CheckForValidity() or
//! RMLVelocityInputParameters::CheckForValidity() return \c false, it may
//! even be in indication for a wrong implementation of the application
//! using the library, and it may happen that one the error-indicating
//! values will be returned by ReflexxesAPI::RMLPosition() or
//! ReflexxesAPI::RMLVelocity(), respectively. Furtheron, it may happen in
//! this case that the execution time exceeds the value of
//! RML_MAX_EXECUTION_TIME, such that RML_ERROR_EXECUTION_TIME_TOO_BIG
//! will be returned.
//!
//! \note
//! If RMLPositionInputParameters::CheckForValidity() or
//! RMLVelocityInputParameters::CheckForValidity(), respectively, return
//! \c true, either ReflexxesAPI::RML_WORKING or
//! ReflexxesAPI::RML_FINAL_STATE_REACHED will be returned.
//!
//! \sa ReflexxesAPI::RMLPosition()
//! \sa ReflexxesAPI::RMLVelocity()
//! \if RMLTYPEII \sa TypeIIRMLPosition::GetNextStateOfMotion() \endif
//! \if RMLTYPEII \sa TypeIIRMLVelocity::GetNextStateOfMotion() \endif
//! \if CUSTOMER \sa TypeIVRMLPosition::GetNextStateOfMotion() \endif
//! \if CUSTOMER \sa TypeIVRMLVelocity::GetNextStateOfMotion() \endif
//! \sa \ref page_ErrorHandling
//  ----------------------------------------------------------
    enum RMLResultValue
    {
        //! \details
        //! The On-Line Trajectory Generation algorithm is working; the final
        //! state of motion has not been reached yet.
        RML_WORKING                             =   0,
        //! \details
        //! The desired final state of motion has been reached.
        RML_FINAL_STATE_REACHED                 =   1,
        //! \details
        //! This is the initialization value of TypeIVRMLPosition::ReturnValue
        //! and TypeIVRMLVelocity::ReturnValue. In practice, this value.
        //! cannot be returned
        RML_ERROR                               =   -1,
        //! \details
        //! The applied input values are invalid (cf.
        //! RMLPositionInputParameters::CheckForValidity()
        //! RMLVelocityInputParameters::CheckForValidity()).
        RML_ERROR_INVALID_INPUT_VALUES          =   -100,
        //! \details
        //! An error occurred during the first step of
        //! the algorithm (i.e., during the calculation of the synchronization time).
        RML_ERROR_EXECUTION_TIME_CALCULATION    =   -101,
        //! \details
        //! An error occurred during the second step of
        //! the algorithm (i.e., during the synchronization of the trajectory).
        RML_ERROR_SYNCHRONIZATION               =   -102,
        //! \details
        //! The number of degree of freedom of th input parameters, the
        //! output parameters, and the On-Line Trajectory Generation
        //! algorithm do not match.
        RML_ERROR_NUMBER_OF_DOFS                =   -103,
        //! \details
        //! If the input flag RMLFlags::ONLY_PHASE_SYNCHRONIZATION is set
        //! and it is not possible to calculate a physically (and
        //! mathematically) correct phase-synchronized (i.e., homothetic)
        //! trajectory, this error value will be returned. Please note:
        //! Even if this error message is returned, feasible, steady, and
        //! continuous output values will be computed in \em any case.
        RML_ERROR_NO_PHASE_SYNCHRONIZATION      =   -104,
        //! \details
        //! If one of the pointers to objects of the classes
        //!
        //! - RMLPositionInputParameters / RMLVelocityInputParameters
        //! - RMLPositionOutputParameters / RMLVelocityOutputParameters
        //! - RMLPositionFlags / RMLVelocityFlags
        //!
        //! is NULL, this error value will be returned.
        RML_ERROR_NULL_POINTER                  =   -105,
        //! \details
        //! To ensure numerical stability, the value of the minimum trajectory
        //! execution time is limited to a value of RML_MAX_EXECUTION_TIME
        //! (\f$ 10^{10} \f$ seconds).
        RML_ERROR_EXECUTION_TIME_TOO_BIG        =   -106,
        //! \details
        //! If either
        //! <ul>
        //!   <li>the method ReflexxesAPI::RMLPositionAtAGivenSampleTime() or</li>
        //!   <li>the method ReflexxesAPI::RMLVelocityAtAGivenSampleTime()</li>
        //! </ul>
        //! was used, the value of the parameter is negative or larger than
        //! the value of <c>RML_MAX_EXECUTION_TIME</c> (\f$10^{10}\,s\f$).
        RML_ERROR_USER_TIME_OUT_OF_RANGE        =   -107
    };


//  ---------------------- Doxygen info ----------------------
//! \fn int RMLPosition(const RMLPositionInputParameters &InputValues, RMLPositionOutputParameters *OutputValues, const RMLPositionFlags &Flags)
//!
//! \brief
//! This is \b the central method of each Reflexxes Type Motion Library
//!
//! \details
//! This method executes the position-based Reflexxes On-Line
//! Trajectory Generation algorithm.\n
//! \n
//! Based on the current state of motion \f$ {\bf M}_i \f$, the kinematic
//! motion constraints \f$ {\bf B}_i \f$, the desired target state of
//! motion \f$ {\bf M}_i^{\,trgt} \f$, and the selection vector
//! \f$ {\bf S}_i \f$ (all of which are contained in
//! ReflexxesAPI::InputValues, a new
//! state of motion \f$ {\bf M}_{i+1} \f$ is calculated, which can be
//! utilized as a set-point value at instant \f$ T_{i+1} \f$. The time
//! difference between \f$ {\bf M}_{i} \f$ and \f$ {\bf M}_{i+1} \f$ is
//! ReflexxesAPI::CycleTime (given in seconds).\n\n
//!
//! For a detailed description, please refer to the method
//! TypeIVRMLPosition::GetNextStateOfMotion().
//!
//! \param InputValues
//! Specifies the input values of the OTG algorithm, that is, on object of
//! the class RMLPositionInputParameters. Please refer to the
//! \ref page_InputValues for a detailed description.
//!
//! \param OutputValues
//! A pointer to an object of the class RMLPositionOutputParameters. The
//! method will copy the new state of motion as well as further trajectory-
//! relevant data (e.g., minimum and maximum positions and corresponding
//! states of motion) to the object pointed to by \c OutputValues. Please
//! refer to the \ref page_OutputValues for a detailed description.
//!
//! \param Flags
//! An object of the class RMLPositionFlags, which specifies a certain
//! behavior of the called method. For a complete description, please
//! refer to RMLPositionFlags.
//!
//! \return
//! An integer value as specified in ReflexxesAPI::RMLResultValue. If the
//! implementation is correct, the method either returns
//! ReflexxesAPI::RML_WORKING or ReflexxesAPI::RML_FINAL_STATE_REACHED. In
//! general, one of the following values can be returned:\n\n
//!
//!  - ReflexxesAPI::RML_WORKING: \copydoc ReflexxesAPI::RML_WORKING\n\n
//!  - ReflexxesAPI::RML_FINAL_STATE_REACHED: \copydoc ReflexxesAPI::RML_FINAL_STATE_REACHED\n\n
//!  - ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES: \copydoc ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES\n\n
//!  - ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION: \copydoc ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION\n\n
//!  - ReflexxesAPI::RML_ERROR_SYNCHRONIZATION: \copydoc ReflexxesAPI::RML_ERROR_SYNCHRONIZATION\n\n
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
//! \sa ReflexxesAPI::RMLVelocity()
//! \if RMLTYPEII \sa TypeIIRMLPosition::GetNextStateOfMotion() \endif
//! \if RMLTYPEII \sa TypeIIRMLPosition \endif
//! \if CUSTOMER \sa TypeIVRMLPosition::GetNextStateOfMotion() \endif
//! \if CUSTOMER \sa TypeIVRMLPosition \endif
//! \sa \ref page_TypeIIAndIVOverview
//! \sa \ref sec_InputValuesPosition
//! \sa \ref sec_OutputValuesPosition
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    int RMLPosition(    const RMLPositionInputParameters    &InputValues
                    ,   RMLPositionOutputParameters         *OutputValues
                    ,   const RMLPositionFlags              &Flags);


//  ---------------------- Doxygen info ----------------------
//! \fn int RMLPositionAtAGivenSampleTime(const double &TimeValueInSeconds, RMLPositionOutputParameters *OutputValues)
//!
//! \brief
//! After calling ReflexxesAPI::RMLPosition(), this methods returns the
//! state of motion of the computed trajectory at a given time instant.
//!
//! \details
//! If the Reflexxes Motion Library is used for motion profile
//! generation (profiler), this method returns the state of motion at
//! at the instant \c TimeValueInSeconds that was calculated by the last
//! call of ReflexxesAPI::RMLPosition(). \c TimeValueInSeconds is given in
//! seconds and w.r.t. to the instant of the last call of
//! ReflexxesAPI::RMLPosition().
//!
//! \param TimeValueInSeconds
//! Time instant in seconds, at which the state of motion will be
//! calculated.
//!
//! \param OutputValues
//! A pointer to an object of the class RMLPositionOutputParameters. The
//! method will copy the new state of the instant \c TimeValueInSeconds
//! of motion as well as further trajectory-relevant data (e.g., minimum
//! and maximum positions and corresponding states of motion) to the
//! object pointed to by \c OutputValues. Please refer to the
//! \ref page_OutputValues for a detailed description.
//!
//! \return
//! An integer value as specified in ReflexxesAPI::RMLResultValue. If the
//! implementation is correct, the method either returns
//! ReflexxesAPI::RML_WORKING or ReflexxesAPI::RML_FINAL_STATE_REACHED. In
//! general, one of the following values can be returned:\n\n
//!
//!  - ReflexxesAPI::RML_WORKING: \copydoc ReflexxesAPI::RML_WORKING\n\n
//!  - ReflexxesAPI::RML_FINAL_STATE_REACHED: \copydoc ReflexxesAPI::RML_FINAL_STATE_REACHED\n\n
//!  - ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES: \copydoc ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES\n\n
//!  - ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION: \copydoc ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION\n\n
//!  - ReflexxesAPI::RML_ERROR_SYNCHRONIZATION: \copydoc ReflexxesAPI::RML_ERROR_SYNCHRONIZATION\n\n
//!  - ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS: \copydoc ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS\n\n
//!  - ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION: \copydoc ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION\n\n
//!  - ReflexxesAPI::RML_ERROR_NULL_POINTER: \copydoc ReflexxesAPI::RML_ERROR_NULL_POINTER\n\n
//!  - ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG: \copydoc ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG\n\n
//!  - ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE: \copydoc ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE\n\n
//!
//! In case, the call of ReflexxesAPI::RMLPosition() returned an error
//! value, the same error value will be returned by this method.
//!
//! \sa ReflexxesAPI::RMLPosition()
//! \sa ReflexxesAPI::RMLVelocityAtAGivenSampleTime()
//! \sa \ref page_TypeIIAndIVOverview
//! \sa \ref sec_InputValuesPosition
//! \sa \ref sec_OutputValuesPosition
//  ----------------------------------------------------------
    int RMLPositionAtAGivenSampleTime(      const double                        &TimeValueInSeconds
                                        ,   RMLPositionOutputParameters         *OutputValues);


//  ---------------------- Doxygen info ----------------------
//! \fn int RMLVelocity(const RMLVelocityInputParameters &InputValues, RMLVelocityOutputParameters *OutputValues, const RMLVelocityFlags &Flags)
//!
//! \brief
//! This is the method for velocity-based On-Line Trajectory Generation.
//!
//! \details
//! This method executes the velocity-based Reflexxes On-Line
//! Trajectory Generation algorithm.\n
//! \n
//! The implementation and the required mathematical back-end is much simpler
//! than for the position-based method (ReflexxesAPI::RMLPosition()).
//! The position-based algorithm is implemented in the class
//! TypeIVRMLPosition, and the velocity-based algorithm is implemented in
//! TypeIVRMLVelocity. Basically, a call of
//! this method behaves in the very same way than a call of RMLPosition(),
//! but it is not possible to specify a desired target position vector
//! \f$ \vec{P}_i{\,trgt} \f$ and a maximum velocity vector
//! \f$ \vec{V}_i{\,max} \f$. Internally, the method
//! TypeIVRMLVelocity::GetNextStateOfMotion() is called. For further
//! details, please refer to this method.\n
//! \n
//! Based on the current state of motion \f$ {\bf M}_i \f$, the kinematic
//! motion constraints \f$ {\bf B}_i \f$, the desired target velocity
//! vector \f$ \vec{V}_i^{\,trgt} \f$, and the selection vector
//! \f$ {\bf S}_i \f$ (all of which are contained in \c InputValues, a new
//! state of motion \f$ {\bf M}_{i+1} \f$ is calculated, which can be
//! utilized as a set-point value at the next control cycle. The time
//! difference between \f$ {\bf M}_{i} \f$ and \f$ {\bf M}_{i+1} \f$ is
//! ReflexxesAPI::CycleTime (given in seconds).
//!
//! \param InputValues
//! Specifies the input values of the velocity-based OTG algorithm, that is,
//! on object of the class RMLVelocityInputParameters. Please refer to the
//! \ref page_InputValues for a detailed description.
//!
//! \param OutputValues
//! A pointer to an object of the class RMLVelocityOutputParameters. The
//! method will copy the new state of motion as well as further trajectory-
//! relevant data (e.g., minimum and maximum Velocities and corresponding
//! states of motion) to the object pointed to by \c OutputValues. Please
//! refer to the \ref page_OutputValues for a detailed description.
//!
//! \param Flags
//! An object of the class RMLVelocityFlags, which specifies a certain
//! behavior of the called method. For a complete description, please
//! refer to RMLVelocityFlags.
//!
//! \return
//! An integer value as specified in ReflexxesAPI::RMLResultValue. In a
//! correct implementation, the method either returns
//! ReflexxesAPI::RML_WORKING or ReflexxesAPI::RML_FINAL_STATE_REACHED.
//! In general, one of the following values can be returned:\n\n
//!
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
//! \sa ReflexxesAPI::RMLPosition()
//! \if RMLTYPEII \sa TypeIIRMLVelocity::GetNextStateOfMotion() \endif
//! \if RMLTYPEII \sa TypeIIRMLVelocity \endif
//! \if CUSTOMER \sa TypeIVRMLVelocity::GetNextStateOfMotion() \endif
//! \if CUSTOMER \sa TypeIVRMLVelocity \endif
//! \sa \ref sec_InputValuesVelocity
//! \sa \ref sec_OutputValuesVelocity
//! \sa \ref page_SynchronizationBehavior
//  ----------------------------------------------------------
    int RMLVelocity(    const RMLVelocityInputParameters    &InputValues
                    ,   RMLVelocityOutputParameters         *OutputValues
                    ,   const RMLVelocityFlags              &Flags);



//  ---------------------- Doxygen info ----------------------
//! \fn int RMLVelocityAtAGivenSampleTime(const double &TimeValueInSeconds, RMLVelocityOutputParameters *OutputValues)
//!
//! \brief
//! After calling ReflexxesAPI::RMLVelocity(), this methods returns the
//! state of motion of the computed trajectory at a given time instant.
//!
//! \details
//! If the Reflexxes Motion Library is used for motion profile
//! generation (profiler), this method returns the state of motion at
//! at the instant \c TimeValueInSeconds that was calculated by the last
//! call of ReflexxesAPI::RMLVelocity(). \c TimeValueInSeconds is given in
//! seconds and w.r.t. to the instant of the last call of
//! ReflexxesAPI::RMLVelocity().
//!
//! \param TimeValueInSeconds
//! Time instant in seconds, at which the state of motion will be
//! calculated.
//!
//! \param OutputValues
//! A pointer to an object of the class RMLVelocityOutputParameters. The
//! method will copy the new state of the instant \c TimeValueInSeconds
//! of motion as well as further trajectory-relevant data (e.g., minimum
//! and maximum positions and corresponding states of motion) to the
//! object pointed to by \c OutputValues. Please refer to the
//! \ref page_OutputValues for a detailed description.
//!
//! \return
//! An integer value as specified in ReflexxesAPI::RMLResultValue. If the
//! implementation is correct, the method either returns
//! ReflexxesAPI::RML_WORKING or ReflexxesAPI::RML_FINAL_STATE_REACHED. In
//! general, one of the following values can be returned:\n\n
//!
//!  - ReflexxesAPI::RML_WORKING: \copydoc ReflexxesAPI::RML_WORKING\n\n
//!  - ReflexxesAPI::RML_FINAL_STATE_REACHED: \copydoc ReflexxesAPI::RML_FINAL_STATE_REACHED\n\n
//!  - ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES: \copydoc ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES\n\n
//!  - ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS: \copydoc ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS\n\n
//!  - ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION: \copydoc ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION\n\n
//!  - ReflexxesAPI::RML_ERROR_NULL_POINTER: \copydoc ReflexxesAPI::RML_ERROR_NULL_POINTER\n\n
//!  - ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE: \copydoc ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE\n\n
//!
//! In case, the call of ReflexxesAPI::RMLVelocity() returned an error
//! value, the same error value will be returned by this method.
//!
//! \sa ReflexxesAPI::RMLVelocity()
//! \sa ReflexxesAPI::RMLPositionAtAGivenSampleTime()
//! \sa \ref page_TypeIIAndIVOverview
//! \sa \ref sec_InputValuesVelocity
//! \sa \ref sec_OutputValuesVelocity
//  ----------------------------------------------------------
    int RMLVelocityAtAGivenSampleTime(      const double                        &TimeValueInSeconds
                                        ,   RMLVelocityOutputParameters         *OutputValues);

protected:

//  ---------------------- Doxygen info ----------------------
//! \var unsigned int NumberOfDOFs
//!
//! \brief
//! Positive integer number of degrees of freedom as specified by the constructor ReflexxesAPI()
//!
//! \sa ReflexxesAPI::ReflexxesAPI()
//! \if RMLTYPEII \sa TypeIIRMLPosition::NumberOfDOFs \endif
//! \if RMLTYPEII \sa TypeIIRMLVelocity::NumberOfDOFs \endif
//! \if CUSTOMER \sa TypeIVRMLPosition::NumberOfDOFs \endif
//! \if CUSTOMER \sa TypeIVRMLVelocity::NumberOfDOFs \endif
//  ----------------------------------------------------------
    unsigned int        NumberOfDOFs;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int NumberOfOwnThreads
//!
//! \brief
//! Positive integer number of additional threads as specified by the constructor ReflexxesAPI()
//!
//! \details
//! The default value of this parameter is zero, that is, the algorithms of
//! of the Reflexxes Motion Library are executed single-threaded.
//! To achieve the best performance, it is recommended to use the Reflexxes
//! Motion Library in this mode.\n
//! \n
//! Depending on the number of degrees of freedom it can make sense to
//! increase this number. In such a case, the constructor of the class
//! ReflexxesAPI() creates a number of additional working threads. These
//! threads are commonly suspended and are only woken up if the main thread
//! has parallelized computations to do.
//!
//! \sa ReflexxesAPI::ReflexxesAPI()
//! \if CUSTOMER \sa TypeIVRMLPosition::NumberOfOwnThreads \endif
//! \sa \ref page_RealTimeBehavior
//  ----------------------------------------------------------
    unsigned int        NumberOfOwnThreads;


//  ---------------------- Doxygen info ----------------------
//! \var double CycleTime
//!
//! \brief
//! Cycle time in seconds as specified by the constructor ReflexxesAPI()
//!
//! \sa ReflexxesAPI::ReflexxesAPI()
//! \if RMLTYPEII \sa TypeIIRMLPosition::CycleTime \endif
//! \if RMLTYPEII \sa TypeIIRMLVelocity::CycleTime \endif
//! \if CUSTOMER \sa TypeIVRMLPosition::CycleTime \endif
//! \if CUSTOMER \sa TypeIVRMLVelocity::CycleTime \endif
//  ----------------------------------------------------------
    double              CycleTime;


//  ---------------------- Doxygen info ----------------------
//! \var void *RMLPositionObject
//!
//! \brief
//! A pointer to an object for position-based On-Line Trajectory Generation
//!
//! \details
//! Depending on the type of Reflexxes Motion Library, an object containing
//! the functionality for the position-based On-Line Trajectory Generation
//! algorithm is created by the constructor ReflexxesAPI::ReflexxesAPI().
//! The method ReflexxesAPI::RMLPosition() accesses the functionality
//! of this object.
//!
//! \sa ReflexxesAPI::ReflexxesAPI()
//! \sa ReflexxesAPI::RMLVelocityObject
//! \if RMLTYPEII \sa TypeIIRMLPosition \endif
//! \if CUSTOMER \sa TypeIVRMLPosition \endif
//  ----------------------------------------------------------
    void                *RMLPositionObject;


//  ---------------------- Doxygen info ----------------------
//! \var void *RMLVelocityObject
//!
//! \brief
//! A pointer to an object for velocity-based On-Line Trajectory Generation
//!
//! \details
//! Depending on the type of Reflexxes Motion Library, an object containing
//! the functionality for the velocity-based On-Line Trajectory Generation
//! algorithm is created by the constructor ReflexxesAPI::ReflexxesAPI().
//! The method ReflexxesAPI::RMLVelocity() accesses the functionality
//! of this object.
//!
//! \sa ReflexxesAPI::ReflexxesAPI()
//! \sa ReflexxesAPI::RMLPositionObject
//! \if RMLTYPEII \sa TypeIIRMLVelocity \endif
//! \if CUSTOMER \sa TypeIVRMLVelocity \endif
//  ----------------------------------------------------------
    void                *RMLVelocityObject;

};  // class ReflexxesAPI


#endif
