//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLStep2PhaseSynchronization.cpp
//!
//! \brief
//! Implementation file for the method TypeIIRMLPosition::Step2PhaseSynchronization()
//!
//! \details
//! For further information, please refer to the file TypeIIRMLPosition.h.
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


#include <TypeIIRMLPosition.h>
#include <TypeIIRMLMath.h>
#include <TypeIIRMLDecisionTree2.h>

using namespace TypeIIRMLMath;


//****************************************************************************
// Step2PhaseSynchronization()

void TypeIIRMLPosition::Step2PhaseSynchronization(void)
{
    unsigned int        i                                   =   0
                    ,   j                                   =   0   ;

    double              P_a0                                =   0.0
                    ,   P_a1                                =   0.0
                    ,   P_a2                                =   0.0
                    ,   V_a0                                =   0.0
                    ,   V_a1                                =   0.0
                    ,   V_a2                                =   0.0
                    ,   A_a0                                =   0.0
                    ,   A_a1                                =   0.0
                    ,   A_a2                                =   0.0
                    ,   DeltaT                              =   0.0
                    ,   ScalingValueFromReferenceVector     =   0.0
                    ,   V_ErrorAtBeginning                  =   0.0
                    ,   P_ErrorAtEnd                        =   0.0
                    ,   V_ErrorAtEnd                        =   0.0 ;

    // Calculate the trajectory of the reference DOF

    TypeIIRMLDecisionTree2(     (this->CurrentInputParameters->CurrentPositionVector->VecData   )[this->GreatestDOFForPhaseSynchronization]
                            ,   (this->CurrentInputParameters->CurrentVelocityVector->VecData   )[this->GreatestDOFForPhaseSynchronization]
                            ,   (this->CurrentInputParameters->TargetPositionVector->VecData    )[this->GreatestDOFForPhaseSynchronization]
                            ,   (this->CurrentInputParameters->TargetVelocityVector->VecData    )[this->GreatestDOFForPhaseSynchronization]
                            ,   (this->CurrentInputParameters->MaxVelocityVector->VecData       )[this->GreatestDOFForPhaseSynchronization]
                            ,   (this->CurrentInputParameters->MaxAccelerationVector->VecData   )[this->GreatestDOFForPhaseSynchronization]
                            ,   this->SynchronizationTime
                            ,   &((this->Polynomials)[this->GreatestDOFForPhaseSynchronization])    );



    // Adapt the synchronization time value to the time value, at which the reference DOF reaches
    // its final state of motion. Although, this should not differ from the originally calculated
    // synchronization time value, it differs due to numerical inaccuracies. In oder to compensate
    // these inaccuracies, we use the Step 2 result value of the reference DOF.
    this->SynchronizationTime   =   (((this->Polynomials)[this->GreatestDOFForPhaseSynchronization]).PolynomialTimes)[((this->Polynomials)[this->GreatestDOFForPhaseSynchronization]).ValidPolynomials - 2];

    // Calculate the trajectory of all other selected DOFs

    for (i = 0; i < this->NumberOfDOFs; i++)
    {
        if ( (this->ModifiedSelectionVector->VecData)[i] && (i != this->GreatestDOFForPhaseSynchronization) )
        {
            ScalingValueFromReferenceVector =   (this->PhaseSynchronizationReferenceVector->VecData)[i]
                                                    / (this->PhaseSynchronizationReferenceVector->VecData)[this->GreatestDOFForPhaseSynchronization];

            for (j = 0; j < ((this->Polynomials)[this->GreatestDOFForPhaseSynchronization]).ValidPolynomials; j++)
            {
                ((this->Polynomials)[this->GreatestDOFForPhaseSynchronization]).PositionPolynomial[j].GetCoefficients       (&P_a2, &P_a1,  &P_a0,  &DeltaT);
                ((this->Polynomials)[this->GreatestDOFForPhaseSynchronization]).VelocityPolynomial[j].GetCoefficients       (&V_a2, &V_a1,  &V_a0,  &DeltaT);
                ((this->Polynomials)[this->GreatestDOFForPhaseSynchronization]).AccelerationPolynomial[j].GetCoefficients   (&A_a2, &A_a1,  &A_a0,  &DeltaT);

                P_a2    *=  ScalingValueFromReferenceVector;
                P_a1    *=  ScalingValueFromReferenceVector;
                P_a0    =   ((this->CurrentInputParameters->CurrentPositionVector->VecData)[i]
                            + (P_a0
                            - (this->CurrentInputParameters->CurrentPositionVector->VecData)[this->GreatestDOFForPhaseSynchronization])
                            * ScalingValueFromReferenceVector);

                V_a2    *=  ScalingValueFromReferenceVector;
                V_a1    *=  ScalingValueFromReferenceVector;
                V_a0    *=  ScalingValueFromReferenceVector;

                A_a2    *=  ScalingValueFromReferenceVector;
                A_a1    *=  ScalingValueFromReferenceVector;
                A_a0    *=  ScalingValueFromReferenceVector;

                ((this->Polynomials)[i]).PositionPolynomial[j].SetCoefficients      (P_a2, P_a1, P_a0, DeltaT);
                ((this->Polynomials)[i]).VelocityPolynomial[j].SetCoefficients      (V_a2, V_a1, V_a0, DeltaT);
                ((this->Polynomials)[i]).AccelerationPolynomial[j].SetCoefficients  (A_a2, A_a1, A_a0, DeltaT);

                ((this->Polynomials)[i]).PolynomialTimes[j] = ((this->Polynomials)[this->GreatestDOFForPhaseSynchronization]).PolynomialTimes[j];
            }

            ((this->Polynomials)[i]).ValidPolynomials   =   ((this->Polynomials)[this->GreatestDOFForPhaseSynchronization]).ValidPolynomials;

            // ----------------------------------------------------------
            // Correcting numerical errors by adding a polynomial of degree one to the existing polynomials
            // ----------------------------------------------------------

            if (this->SynchronizationTime > this->CycleTime)
            {
                V_ErrorAtBeginning  =   (this->CurrentInputParameters->CurrentVelocityVector->VecData)[i]
                                        -   ((this->Polynomials)[i]).VelocityPolynomial[0].CalculateValue(0.0);

                V_ErrorAtEnd        =   (this->CurrentInputParameters->TargetVelocityVector->VecData)[i]
                                        -   ((this->Polynomials)[i]).VelocityPolynomial[((this->Polynomials)[i]).ValidPolynomials - 1].CalculateValue(this->SynchronizationTime);

                for (j = 0; j < ((this->Polynomials)[i]).ValidPolynomials; j++)
                {
                    ((this->Polynomials)[i]).PositionPolynomial[j].GetCoefficients      (&P_a2, &P_a1,  &P_a0,  &DeltaT);
                    ((this->Polynomials)[i]).VelocityPolynomial[j].GetCoefficients      (&V_a2, &V_a1,  &V_a0,  &DeltaT);

                    V_a1    +=  (V_ErrorAtEnd - V_ErrorAtBeginning) / this->SynchronizationTime;
                    V_a0    +=  V_ErrorAtBeginning - DeltaT * (V_ErrorAtEnd - V_ErrorAtBeginning) / this->SynchronizationTime;

                    P_a1    =   V_a0;

                    ((this->Polynomials)[i]).PositionPolynomial[j].SetCoefficients      (P_a2, P_a1, P_a0, DeltaT);
                    ((this->Polynomials)[i]).VelocityPolynomial[j].SetCoefficients      (V_a2, V_a1, V_a0, DeltaT);
                }

                P_ErrorAtEnd        =   (this->CurrentInputParameters->TargetPositionVector->VecData)[i]
                                        -   ((this->Polynomials)[i]).PositionPolynomial[((this->Polynomials)[i]).ValidPolynomials - 1].CalculateValue(this->SynchronizationTime);

                for (j = 0; j < ((this->Polynomials)[i]).ValidPolynomials; j++)
                {
                    ((this->Polynomials)[i]).PositionPolynomial[j].GetCoefficients      (&P_a2, &P_a1,  &P_a0,  &DeltaT);

                    P_a1    +=  P_ErrorAtEnd / this->SynchronizationTime;
                    P_a0    -=  DeltaT * P_ErrorAtEnd / this->SynchronizationTime;

                    ((this->Polynomials)[i]).PositionPolynomial[j].SetCoefficients      (P_a2, P_a1, P_a0, DeltaT);
                }
            }
            // ----------------------------------------------------------
        }
    }

}
