//  ---------------------- Doxygen info ----------------------
//! \file 07_RMLPositionSampleApplication.cpp
//!
//! \brief
//! Test application number 2 for the Reflexxes Motion Libraries
//! (position-based interface, complete description of output values)
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
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>


//*************************************************************************
// defines

#define CYCLE_TIME_IN_SECONDS                   0.001
#define NUMBER_OF_DOFS                          3


//*************************************************************************
// Main function to run the process that contains the test application
//
// This function contains source code to get started with the Type II
// Reflexxes Motion Library. Based on the program
// 01_RMLPositionSampleApplication.cpp, this sample code becomes extended by
// using (and describing) all available output values of the algorithm.
// As in the former example, we compute a trajectory for a system with
// three degrees of freedom starting from an arbitrary state of motion.
// This code snippet again directly corresponds to the example trajectories
// shown in the documentation.
//*************************************************************************
int main()
{
    // ********************************************************************
    // Variable declarations and definitions

    bool                        FirstCycleCompleted         =   false   ;

    int                         ResultValue                 =   0
                            ,   i                           =   0
                            ,   j                           =   0       ;

    ReflexxesAPI                *RML                        =   NULL    ;

    RMLPositionInputParameters  *IP                         =   NULL    ;

    RMLPositionOutputParameters *OP                         =   NULL    ;

    RMLPositionFlags            Flags                                   ;

    // ********************************************************************
    // Creating all relevant objects of the Type II Reflexxes Motion Library

    RML =   new ReflexxesAPI(                   NUMBER_OF_DOFS
                                            ,   CYCLE_TIME_IN_SECONDS   );

    IP  =   new RMLPositionInputParameters(     NUMBER_OF_DOFS          );

    OP  =   new RMLPositionOutputParameters(    NUMBER_OF_DOFS          );

    // ********************************************************************
    // Set-up a timer with a period of one millisecond
    // (not implemented in this example in order to keep it simple)
    // ********************************************************************

    printf("-------------------------------------------------------\n"  );
    printf("Reflexxes Motion Libraries                             \n"  );
    printf("Example: 07_RMLPositionSampleApplication.cpp           \n\n");
    printf("This example demonstrates the use of the entire output \n"  );
    printf("values of the position-based Online Trajectory         \n"  );
    printf("Generation algorithm of the Type II Reflexxes Motion   \n"  );
    printf("Library.                                               \n\n");
    printf("Copyright (C) 2015 Google, Inc.                      \n"  );
    printf("-------------------------------------------------------\n"  );

    // ********************************************************************
    // Set-up the input parameters

    // In this test program, arbitrary values are chosen. If executed on a
    // real robot or mechanical system, the position is read and stored in
    // an RMLPositionInputParameters::CurrentPositionVector vector object.
    // For the very first motion after starting the controller, velocities
    // and acceleration are commonly set to zero. The desired target state
    // of motion and the motion constraints depend on the robot and the
    // current task/application.
    // The internal data structures make use of native C data types
    // (e.g., IP->CurrentPositionVector->VecData is a pointer to
    // an array of NUMBER_OF_DOFS double values), such that the Reflexxes
    // Library can be used in a universal way.

    IP->CurrentPositionVector->VecData      [0] =    100.0      ;
    IP->CurrentPositionVector->VecData      [1] =      0.0      ;
    IP->CurrentPositionVector->VecData      [2] =     50.0      ;

    IP->CurrentVelocityVector->VecData      [0] =    100.0      ;
    IP->CurrentVelocityVector->VecData      [1] =   -220.0      ;
    IP->CurrentVelocityVector->VecData      [2] =    -50.0      ;

    IP->CurrentAccelerationVector->VecData  [0] =   -150.0      ;
    IP->CurrentAccelerationVector->VecData  [1] =    250.0      ;
    IP->CurrentAccelerationVector->VecData  [2] =    -50.0      ;

    IP->MaxVelocityVector->VecData          [0] =    300.0      ;
    IP->MaxVelocityVector->VecData          [1] =    100.0      ;
    IP->MaxVelocityVector->VecData          [2] =    300.0      ;

    IP->MaxAccelerationVector->VecData      [0] =    300.0      ;
    IP->MaxAccelerationVector->VecData      [1] =    200.0      ;
    IP->MaxAccelerationVector->VecData      [2] =    100.0      ;

    IP->MaxJerkVector->VecData              [0] =    400.0      ;
    IP->MaxJerkVector->VecData              [1] =    300.0      ;
    IP->MaxJerkVector->VecData              [2] =    200.0      ;

    IP->TargetPositionVector->VecData       [0] =   -600.0      ;
    IP->TargetPositionVector->VecData       [1] =   -200.0      ;
    IP->TargetPositionVector->VecData       [2] =   -350.0      ;

    IP->TargetVelocityVector->VecData       [0] =    50.0       ;
    IP->TargetVelocityVector->VecData       [1] =   -50.0       ;
    IP->TargetVelocityVector->VecData       [2] =  -200.0       ;

    IP->SelectionVector->VecData            [0] =   true        ;
    IP->SelectionVector->VecData            [1] =   true        ;
    IP->SelectionVector->VecData            [2] =   true        ;

    // ********************************************************************
    // Specifying the minimum synchronization time

    IP->MinimumSynchronizationTime              =   6.5         ;

    // ********************************************************************
    // Checking the input parameters (optional)

    if (IP->CheckForValidity())
    {
        printf("Input values are valid!\n");
    }
    else
    {
        printf("Input values are INVALID!\n");
    }

    // ********************************************************************
    // Starting the control loop

    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {

        // ****************************************************************
        // Wait for the next timer tick
        // (not implemented in this example in order to keep it simple)
        // ****************************************************************

        // Calling the Reflexxes OTG algorithm
        ResultValue =   RML->RMLPosition(       *IP
                                            ,   OP
                                            ,   Flags       );

        if (ResultValue < 0)
        {
            printf("An error occurred (%d).\n", ResultValue );
            break;
        }

        // ****************************************************************
        // The following part completely describes all output values
        // of the Reflexxes Type II Online Trajectory Generation
        // algorithm.

        if (!FirstCycleCompleted)
        {
            FirstCycleCompleted =   true;

            printf("-------------------------------------------------------\n");
            printf("General information:\n\n");

            printf("The execution time of the current trajectory is %.3lf seconds.\n", OP->GetSynchronizationTime());

            if (OP->IsTrajectoryPhaseSynchronized())
            {
                printf("The current trajectory is phase-synchronized.\n");
            }
            else
            {
                printf("The current trajectory is time-synchronized.\n");
            }
            if (OP->WasACompleteComputationPerformedDuringTheLastCycle())
            {
                printf("The trajectory was computed during the last computation cycle.\n");
            }
            else
            {
                printf("The input values did not change, and a new computation of the trajectory parameters was not required.\n");
            }

            printf("-------------------------------------------------------\n");
            printf("New state of motion:\n\n");

            printf("New position/pose vector                  : ");
            for ( j = 0; j < NUMBER_OF_DOFS; j++)
            {
                printf("%10.3lf ", OP->NewPositionVector->VecData[j]);
            }
            printf("\n");
            printf("New velocity vector                       : ");
            for ( j = 0; j < NUMBER_OF_DOFS; j++)
            {
                printf("%10.3lf ", OP->NewVelocityVector->VecData[j]);
            }
            printf("\n");
            printf("New acceleration vector                   : ");
            for ( j = 0; j < NUMBER_OF_DOFS; j++)
            {
                printf("%10.3lf ", OP->NewAccelerationVector->VecData[j]);
            }
            printf("\n");
            printf("-------------------------------------------------------\n");
            printf("Extremes of the current trajectory:\n");

            for ( i = 0; i < NUMBER_OF_DOFS; i++)
            {
                printf("\n");
                printf("Degree of freedom                         : %d\n", i);
                printf("Minimum position                          : %10.3lf\n", OP->MinPosExtremaPositionVectorOnly->VecData[i]);
                printf("Time, at which the minimum will be reached: %10.3lf\n", OP->MinExtremaTimesVector->VecData[i]);
                printf("Position/pose vector at this time         : ");
                for ( j = 0; j < NUMBER_OF_DOFS; j++)
                {
                    printf("%10.3lf ", OP->MinPosExtremaPositionVectorArray[i]->VecData[j]);
                }
                printf("\n");
                printf("Velocity vector at this time              : ");
                for ( j = 0; j < NUMBER_OF_DOFS; j++)
                {
                    printf("%10.3lf ", OP->MinPosExtremaVelocityVectorArray[i]->VecData[j]);
                }
                printf("\n");
                printf("Acceleration vector at this time          : ");
                for ( j = 0; j < NUMBER_OF_DOFS; j++)
                {
                    printf("%10.3lf ", OP->MinPosExtremaAccelerationVectorArray[i]->VecData[j]);
                }
                printf("\n");
                printf("Maximum position                          : %10.3lf\n", OP->MaxPosExtremaPositionVectorOnly->VecData[i]);
                printf("Time, at which the maximum will be reached: %10.3lf\n", OP->MaxExtremaTimesVector->VecData[i]);
                printf("Position/pose vector at this time         : ");
                for ( j = 0; j < NUMBER_OF_DOFS; j++)
                {
                    printf("%10.3lf ", OP->MaxPosExtremaPositionVectorArray[i]->VecData[j]);
                }
                printf("\n");
                printf("Velocity vector at this time              : ");
                for ( j = 0; j < NUMBER_OF_DOFS; j++)
                {
                    printf("%10.3lf ", OP->MaxPosExtremaVelocityVectorArray[i]->VecData[j]);
                }
                printf("\n");
                printf("Acceleration vector at this time          : ");
                for ( j = 0; j < NUMBER_OF_DOFS; j++)
                {
                    printf("%10.3lf ", OP->MaxPosExtremaAccelerationVectorArray[i]->VecData[j]);
                }
                printf("\n");
            }
            printf("-------------------------------------------------------\n");
        }
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
