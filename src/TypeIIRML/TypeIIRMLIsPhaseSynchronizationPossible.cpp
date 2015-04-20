//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLIsPhaseSynchronizationPossible.cpp
//!
//! \brief
//! Implementation file for the Type II On-Line Trajectory
//! Generation algorithm
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
#include <TypeIIRMLStep1IntermediateProfiles.h>
#include <TypeIIRMLStep1Profiles.h>
#include <RMLPositionInputParameters.h>

using namespace TypeIIRMLMath;


//*******************************************************************************************
// IsPhaseSynchronizationPossible

bool TypeIIRMLPosition::IsPhaseSynchronizationPossible(RMLDoubleVector* ReferenceVector)
{
    bool                    Result                          =   true
                        ,   SignSwitch                      =   false;

    unsigned int            i                               =   0;

    double                  LengthOfDirectionVector         =   0.0
                        ,   LengthOfCurrentVelocityVector   =   0.0
                        ,   LengthOfTargetVelocityVector    =   0.0
                        ,   LengthOfReferenceVector         =   0.0;


    for (i = 0; i < this->NumberOfDOFs; i++)
    {
        if ((this->ModifiedSelectionVector->VecData)[i])
        {
            (this->PhaseSynchronizationPositionDifferenceVector->VecData)[i]
                =       (this->CurrentInputParameters->TargetPositionVector->VecData)[i]
                     -  (this->CurrentInputParameters->CurrentPositionVector->VecData)[i];
            (this->PhaseSynchronizationCurrentVelocityVector->VecData)[i]
                =   (this->CurrentInputParameters->CurrentVelocityVector->VecData)[i];
            (this->PhaseSynchronizationTargetVelocityVector->VecData)[i]
                =   (this->CurrentInputParameters->TargetVelocityVector->VecData)[i];

            LengthOfDirectionVector         += pow2((this->PhaseSynchronizationPositionDifferenceVector->VecData)[i]);
            LengthOfCurrentVelocityVector   += pow2((this->PhaseSynchronizationCurrentVelocityVector->VecData)[i]);
            LengthOfTargetVelocityVector    += pow2((this->PhaseSynchronizationTargetVelocityVector->VecData)[i]);
        }
        else
        {
            (this->PhaseSynchronizationPositionDifferenceVector->VecData)   [i]     =   0.0;
            (this->PhaseSynchronizationCurrentVelocityVector->VecData)      [i]     =   0.0;
            (this->PhaseSynchronizationTargetVelocityVector->VecData)       [i]     =   0.0;
        }
    }

    LengthOfDirectionVector         =   RMLSqrt(LengthOfDirectionVector);
    LengthOfCurrentVelocityVector   =   RMLSqrt(LengthOfCurrentVelocityVector);
    LengthOfTargetVelocityVector    =   RMLSqrt(LengthOfTargetVelocityVector);


    if ( (LengthOfDirectionVector != POSITIVE_ZERO) && (LengthOfDirectionVector != 0.0) )
    {
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if ((this->ModifiedSelectionVector->VecData)[i])
            {
                (this->PhaseSynchronizationPositionDifferenceVector->VecData)[i] /= LengthOfDirectionVector;
            }
        }
    }
    else
    {
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if ((this->ModifiedSelectionVector->VecData)[i])
            {
                (this->PhaseSynchronizationPositionDifferenceVector->VecData)[i] = 0.0;
            }
        }
    }

    if ( (LengthOfCurrentVelocityVector != POSITIVE_ZERO) && (LengthOfCurrentVelocityVector != 0.0) )
    {
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if ((this->ModifiedSelectionVector->VecData)[i])
            {
                (this->PhaseSynchronizationCurrentVelocityVector->VecData)[i] /= LengthOfCurrentVelocityVector;
            }
        }
    }
    else
    {
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if ((this->ModifiedSelectionVector->VecData)[i])
            {
                (this->PhaseSynchronizationCurrentVelocityVector->VecData)[i] = 0.0;
            }
        }
    }

    if ( (LengthOfTargetVelocityVector != POSITIVE_ZERO) && (LengthOfTargetVelocityVector != 0.0) )
    {
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if ((this->ModifiedSelectionVector->VecData)[i])
            {
                (this->PhaseSynchronizationTargetVelocityVector->VecData)[i] /= LengthOfTargetVelocityVector;
            }
        }
    }
    else
    {
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if ((this->ModifiedSelectionVector->VecData)[i])
            {
                (this->PhaseSynchronizationTargetVelocityVector->VecData)[i] = 0.0;
            }
        }
    }

    // Determine a reference vector.

    LengthOfReferenceVector =   ABSOLUTE_PHASE_SYNC_EPSILON;

    if (    (   LengthOfDirectionVector     >=  LengthOfCurrentVelocityVector   )
        &&  (   LengthOfDirectionVector     >=  LengthOfTargetVelocityVector    )
        &&  (   LengthOfDirectionVector     >=  ABSOLUTE_PHASE_SYNC_EPSILON     ))
    {
        *(this->PhaseSynchronizationCheckVector)        =   *(this->PhaseSynchronizationPositionDifferenceVector);
        LengthOfReferenceVector                         =   LengthOfDirectionVector;
        this->PhaseSynchronizationMagnitude             =   TypeIIRMLPosition::POSITION;
    }

    if (    (   LengthOfCurrentVelocityVector   >=  LengthOfDirectionVector         )
        &&  (   LengthOfCurrentVelocityVector   >=  LengthOfTargetVelocityVector    )
        &&  (   LengthOfCurrentVelocityVector   >=  ABSOLUTE_PHASE_SYNC_EPSILON     ))
    {
        *(this->PhaseSynchronizationCheckVector)        =   *(this->PhaseSynchronizationCurrentVelocityVector);
        LengthOfReferenceVector                         =   LengthOfCurrentVelocityVector;
        this->PhaseSynchronizationMagnitude             =   TypeIIRMLPosition::CURRENT_VELOCITY;
    }

    if (    (   LengthOfTargetVelocityVector        >=  LengthOfDirectionVector         )
        &&  (   LengthOfTargetVelocityVector        >=  LengthOfCurrentVelocityVector   )
        &&  (   LengthOfTargetVelocityVector        >=  ABSOLUTE_PHASE_SYNC_EPSILON     ))
    {
        *(this->PhaseSynchronizationCheckVector)        =   *(this->PhaseSynchronizationTargetVelocityVector);
        LengthOfReferenceVector                         =   LengthOfTargetVelocityVector;
        this->PhaseSynchronizationMagnitude             =   TypeIIRMLPosition::TARGET_VELOCITY;
    }

    if (LengthOfReferenceVector > ABSOLUTE_PHASE_SYNC_EPSILON)
    {
        // Switch vector orientations

        SignSwitch = true;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if ((this->ModifiedSelectionVector->VecData)[i])
            {
                if ((Sign((this->PhaseSynchronizationPositionDifferenceVector->VecData)[i])
                            == Sign((this->PhaseSynchronizationCheckVector->VecData)[i]))
                    && (fabs((this->PhaseSynchronizationPositionDifferenceVector->VecData)[i])
                            > ABSOLUTE_PHASE_SYNC_EPSILON))
                {
                    SignSwitch = false;
                    break;
                }
            }
        }

        if (SignSwitch)
        {
            for (i = 0; i < this->NumberOfDOFs; i++)
            {
                if ((this->ModifiedSelectionVector->VecData)[i])
                {
                    (this->PhaseSynchronizationPositionDifferenceVector->VecData)[i]
                        =   -(this->PhaseSynchronizationPositionDifferenceVector->VecData)[i];
                }
            }
        }

        SignSwitch = true;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if ((this->ModifiedSelectionVector->VecData)[i])
            {
                if ((Sign((this->PhaseSynchronizationCurrentVelocityVector->VecData)[i])
                            == Sign((this->PhaseSynchronizationCheckVector->VecData)[i]))
                    && (fabs((this->PhaseSynchronizationCurrentVelocityVector->VecData)[i])
                            > ABSOLUTE_PHASE_SYNC_EPSILON))
                {
                    SignSwitch = false;
                    break;
                }
            }
        }

        if (SignSwitch)
        {
            for (i = 0; i < this->NumberOfDOFs; i++)
            {
                if ((this->ModifiedSelectionVector->VecData)[i])
                {
                    (this->PhaseSynchronizationCurrentVelocityVector->VecData)[i]
                        =   -(this->PhaseSynchronizationCurrentVelocityVector->VecData)[i];
                }
            }
        }

        SignSwitch = true;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if ((this->ModifiedSelectionVector->VecData)[i])
            {
                if ((Sign((this->PhaseSynchronizationTargetVelocityVector->VecData)[i])
                            == Sign((this->PhaseSynchronizationCheckVector->VecData)[i]))
                    && (fabs((this->PhaseSynchronizationTargetVelocityVector->VecData)[i])
                            > ABSOLUTE_PHASE_SYNC_EPSILON))
                {
                    SignSwitch = false;
                    break;
                }
            }
        }

        if (SignSwitch)
        {
            for (i = 0; i < this->NumberOfDOFs; i++)
            {
                if ((this->ModifiedSelectionVector->VecData)[i])
                {
                    (this->PhaseSynchronizationTargetVelocityVector->VecData)[i]
                        =   -(this->PhaseSynchronizationTargetVelocityVector->VecData)[i];
                }
            }
        }

        // Check for collinearity

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if ( (this->ModifiedSelectionVector->VecData)[i] )
            {
                if (    ((fabs((this->PhaseSynchronizationCheckVector->VecData)[i] - (this->PhaseSynchronizationPositionDifferenceVector->VecData)[i])
                            > (PHASE_SYNC_COLLINEARITY_REL_EPSILON * fabs((this->PhaseSynchronizationCheckVector->VecData)[i]) ) )
                        &&  (   LengthOfDirectionVector     >=  ABSOLUTE_PHASE_SYNC_EPSILON         ))
                    ||  ((fabs((this->PhaseSynchronizationCheckVector->VecData)[i] - (this->PhaseSynchronizationCurrentVelocityVector->VecData)[i])
                            > (PHASE_SYNC_COLLINEARITY_REL_EPSILON * fabs((this->PhaseSynchronizationCheckVector->VecData)[i]) ) )
                        &&  (   LengthOfCurrentVelocityVector   >=  ABSOLUTE_PHASE_SYNC_EPSILON     ))
                    ||  ((fabs((this->PhaseSynchronizationCheckVector->VecData)[i] - (this->PhaseSynchronizationTargetVelocityVector->VecData)[i])
                            > (PHASE_SYNC_COLLINEARITY_REL_EPSILON * fabs((this->PhaseSynchronizationCheckVector->VecData)[i]) ) )
                        &&  (   LengthOfTargetVelocityVector    >=  ABSOLUTE_PHASE_SYNC_EPSILON     )))
                {
                    Result  =   false;
                    break;
                }
            }
        }
    }
    else
    {
        Result  =   false;
    }

    if (Result)
    {
        *ReferenceVector    =   *(this->PhaseSynchronizationCheckVector);
    }
    else
    {
        ReferenceVector->Set(0.0);
    }

    return(Result);
}
