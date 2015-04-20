//  ---------------------- Doxygen info ----------------------
//! \file 04_RMLVelocitySampleApplication.cpp
//!
//! \brief
//! Test application number 1 for the Reflexxes Motion Libraries
//! (basic velocity-based interface)
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


#include <stdio.h>
#include <stdlib.h>

#include <ReflexxesAPI.h>
#include <RMLVelocityFlags.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>


//*************************************************************************
// defines

#define CYCLE_TIME_IN_SECONDS                   0.001
#define NUMBER_OF_DOFS                          3


//*************************************************************************
// Main function to run the process that contains the test application
//
// This function contains source code to get started with the Type II
// Reflexxes Motion Library. Only a minimum amount of functionality is
// contained in this program: a simple trajectory for a
// three-degree-of-freedom system is executed. This code snippet
// directly corresponds to the example trajectories shown in the
// documentation.
//*************************************************************************
int main()
{
    // ********************************************************************
    // Variable declarations and definitions

    int                         ResultValue                 =   0       ;

    ReflexxesAPI                *RML                        =   NULL    ;

    RMLVelocityInputParameters  *IP                         =   NULL    ;

    RMLVelocityOutputParameters *OP                         =   NULL    ;

    RMLVelocityFlags            Flags                                   ;

    // ********************************************************************
    // Creating all relevant objects of the Type II Reflexxes Motion Library

    RML =   new ReflexxesAPI(                   NUMBER_OF_DOFS
                                            ,   CYCLE_TIME_IN_SECONDS   );

    IP  =   new RMLVelocityInputParameters(     NUMBER_OF_DOFS          );

    OP  =   new RMLVelocityOutputParameters(    NUMBER_OF_DOFS          );

    // ********************************************************************
    // Set-up a timer with a period of one millisecond
    // (not implemented in this example in order to keep it simple)
    // ********************************************************************

    printf("-------------------------------------------------------\n"  );
    printf("Reflexxes Motion Libraries                             \n"  );
    printf("Example: 04_RMLVelocitySampleApplication               \n\n");
    printf("This example demonstrates the most basic use of the    \n"  );
    printf("Reflexxes API (class ReflexxesAPI) using the velocity- \n"  );
    printf("based Type II Online Trajectory Generation algorithm.  \n\n");
    printf("Copyright (C) 2015 Google, Inc.                      \n"  );
    printf("-------------------------------------------------------\n"  );

    // ********************************************************************
    // Set-up the input parameters

    // In this test program, arbitrary values are chosen. If executed on a
    // real robot or mechanical system, the position is read and stored in
    // an RMLVelocityInputParameters::CurrentPositionVector vector object.
    // For the very first motion after starting the controller, velocities
    // and acceleration are commonly set to zero. The desired target state
    // of motion and the motion constraints depend on the robot and the
    // current task/application.
    // The internal data structures make use of native C data types
    // (e.g., IP->CurrentPositionVector->VecData is a pointer to
    // an array of NUMBER_OF_DOFS double values), such that the Reflexxes
    // Library can be used in a universal way.

    IP->CurrentPositionVector->VecData      [0] =   -200.0      ;
    IP->CurrentPositionVector->VecData      [1] =    100.0      ;
    IP->CurrentPositionVector->VecData      [2] =   -300.0      ;

    IP->CurrentVelocityVector->VecData      [0] =   -150.0      ;
    IP->CurrentVelocityVector->VecData      [1] =    100.0      ;
    IP->CurrentVelocityVector->VecData      [2] =     50.0      ;

    IP->CurrentAccelerationVector->VecData  [0] =    350.0      ;
    IP->CurrentAccelerationVector->VecData  [1] =   -500.0      ;
    IP->CurrentAccelerationVector->VecData  [2] =      0.0      ;

    IP->MaxAccelerationVector->VecData      [0] =    500.0      ;
    IP->MaxAccelerationVector->VecData      [1] =    500.0      ;
    IP->MaxAccelerationVector->VecData      [2] =   1000.0      ;

    IP->MaxJerkVector->VecData              [0] =   1000.0      ;
    IP->MaxJerkVector->VecData              [1] =    700.0      ;
    IP->MaxJerkVector->VecData              [2] =    500.0      ;

    IP->TargetVelocityVector->VecData       [0] =   150.0       ;
    IP->TargetVelocityVector->VecData       [1] =    75.0       ;
    IP->TargetVelocityVector->VecData       [2] =   100.0       ;

    IP->SelectionVector->VecData            [0] =   true        ;
    IP->SelectionVector->VecData            [1] =   true        ;
    IP->SelectionVector->VecData            [2] =   true        ;

    // ********************************************************************
    // Starting the control loop

    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {

        // ****************************************************************
        // Wait for the next timer tick
        // (not implemented in this example in order to keep it simple)
        // ****************************************************************

        // Calling the Reflexxes OTG algorithm
        ResultValue =   RML->RMLVelocity(       *IP
                                            ,   OP
                                            ,   Flags       );

        if (ResultValue < 0)
        {
            printf("An error occurred (%d).\n", ResultValue );
            break;
        }

        // ****************************************************************
        // Here, the new state of motion, that is
        //
        // - OP->NewPositionVector
        // - OP->NewVelocityVector
        // - OP->NewAccelerationVector
        //
        // can be used as input values for lower level controllers. In the
        // most simple case, a position controller in actuator space is
        // used, but the computed state can be applied to many other
        // controllers (e.g., Cartesian impedance controllers,
        // operational space controllers).
        // ****************************************************************

        // ****************************************************************
        // Feed the output values of the current control cycle back to
        // input values of the next control cycle

        *IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
        *IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
        *IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;
    }

    // ********************************************************************
    // Deleting the objects of the Reflexxes Motion Library end terminating
    // the process

    delete  RML         ;
    delete  IP          ;
    delete  OP          ;

    exit(EXIT_SUCCESS)  ;
}
