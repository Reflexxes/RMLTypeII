//  ---------------------- Doxygen info ----------------------
//! \file RMLOutputParameters.h
//!
//! \brief
//! Header file for the class RMLOutputParameters
//!
//! \details
//! The class RMLOutputParameters constitutes the basis class for the
//! actual interface classes RMLPositionOutputParameters and
//! RMLVelocityOutputParameters, which are both derived from this one.
//!
//! \sa RMLInputParameters.h
//! \sa RMLVelocityOutputParameters.h
//! \sa RMLPositionOutputParameters.h
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


#ifndef __RMLOutputParameters__
#define __RMLOutputParameters__


#include <RMLVector.h>
#include <string.h>


//  ---------------------- Doxygen info ----------------------
//! \class RMLOutputParameters
//!
//! \brief
//! Class for the output parameters of the On-Line
//! Trajectory Generation algorithm
//!
//! \details
//! The class RMLOutputParameters constitutes the basis class for the
//! actual interface classes RMLPositionOutputParameters and
//! RMLVelocityOutputParameters, which are both derived from this one.
//!
//! \sa ReflexxesAPI
//! \sa RMLPositionOutputParameters
//! \sa RMLVelocityOutputParameters
//! \sa RMLInputParameters
//! \sa \ref page_OutputValues
//  ----------------------------------------------------------
class RMLOutputParameters
{
public:


//  ---------------------- Doxygen info ----------------------
//! \enum ReturnValue
//!
//! \brief
//! Return values for the methods of the class RMLOutputParameters
//  ----------------------------------------------------------
    enum ReturnValue
    {
        RETURN_SUCCESS  =   0,
        RETURN_ERROR    =   -1
    };


protected:

//  ---------------------- Doxygen info ----------------------
//! \fn RMLOutputParameters(const unsigned int DegreesOfFreedom)
//!
//! \brief
//! Constructor of class RMLOutputParameters
//!
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//!
//! \param DegreesOfFreedom
//! Specifies the number of degrees of freedom
//!
//! \note
//! This is only the base class for the classes\n\n
//! <ul>
//!  <li>RMLPositionOutputParameters and</li>
//!  <li>RMLVelocityOutputParameters,\n\n</li>
//! </ul>
//! such that the constructor is declared \c protected.
//  ----------------------------------------------------------
    RMLOutputParameters(const unsigned int DegreesOfFreedom)
    {
        unsigned int    i                           =   0                                       ;

        this->TrajectoryIsPhaseSynchronized         =   false                                   ;

        this->NumberOfDOFs                          =   DegreesOfFreedom                        ;

        this->SynchronizationTime                   =   0.0                                     ;

        this->ANewCalculationWasPerformed           =   false                                   ;

        this->DOFWithTheGreatestExecutionTime       =   0                                       ;

        this->NewPositionVector                     =   new RMLDoubleVector(DegreesOfFreedom)   ;
        this->NewVelocityVector                     =   new RMLDoubleVector(DegreesOfFreedom)   ;
        this->NewAccelerationVector                 =   new RMLDoubleVector(DegreesOfFreedom)   ;
        this->MinExtremaTimesVector                 =   new RMLDoubleVector(DegreesOfFreedom)   ;
        this->MaxExtremaTimesVector                 =   new RMLDoubleVector(DegreesOfFreedom)   ;
        this->MinPosExtremaPositionVectorOnly       =   new RMLDoubleVector(DegreesOfFreedom)   ;
        this->MaxPosExtremaPositionVectorOnly       =   new RMLDoubleVector(DegreesOfFreedom)   ;
        this->ExecutionTimes                        =   new RMLDoubleVector(DegreesOfFreedom)   ;

        memset(this->NewPositionVector->VecData                 ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
        memset(this->NewVelocityVector->VecData                 ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
        memset(this->NewAccelerationVector->VecData             ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
        memset(this->MinExtremaTimesVector->VecData             ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
        memset(this->MaxExtremaTimesVector->VecData             ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
        memset(this->MinPosExtremaPositionVectorOnly->VecData   ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
        memset(this->MaxPosExtremaPositionVectorOnly->VecData   ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
        memset(this->ExecutionTimes->VecData                    ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;

        this->MinPosExtremaPositionVectorArray      =   new RMLDoubleVector*[DegreesOfFreedom]  ;
        this->MinPosExtremaVelocityVectorArray      =   new RMLDoubleVector*[DegreesOfFreedom]  ;
        this->MinPosExtremaAccelerationVectorArray  =   new RMLDoubleVector*[DegreesOfFreedom]  ;
        this->MaxPosExtremaPositionVectorArray      =   new RMLDoubleVector*[DegreesOfFreedom]  ;
        this->MaxPosExtremaVelocityVectorArray      =   new RMLDoubleVector*[DegreesOfFreedom]  ;
        this->MaxPosExtremaAccelerationVectorArray  =   new RMLDoubleVector*[DegreesOfFreedom]  ;

        for (i = 0; i < DegreesOfFreedom; i++)
        {
            (this->MinPosExtremaPositionVectorArray)        [i] =   new RMLDoubleVector(DegreesOfFreedom);
            (this->MinPosExtremaVelocityVectorArray)        [i] =   new RMLDoubleVector(DegreesOfFreedom);
            (this->MinPosExtremaAccelerationVectorArray)    [i] =   new RMLDoubleVector(DegreesOfFreedom);
            (this->MaxPosExtremaPositionVectorArray)        [i] =   new RMLDoubleVector(DegreesOfFreedom);
            (this->MaxPosExtremaVelocityVectorArray)        [i] =   new RMLDoubleVector(DegreesOfFreedom);
            (this->MaxPosExtremaAccelerationVectorArray)    [i] =   new RMLDoubleVector(DegreesOfFreedom);

            memset(((this->MinPosExtremaPositionVectorArray)            [i])->VecData       ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
            memset(((this->MinPosExtremaVelocityVectorArray)            [i])->VecData       ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
            memset(((this->MinPosExtremaAccelerationVectorArray)        [i])->VecData       ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
            memset(((this->MaxPosExtremaPositionVectorArray)            [i])->VecData       ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
            memset(((this->MaxPosExtremaVelocityVectorArray)            [i])->VecData       ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
            memset(((this->MaxPosExtremaAccelerationVectorArray)        [i])->VecData       ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
        }

    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLOutputParameters(const RMLOutputParameters &OP)
//!
//! \brief
//! Copy constructor of class RMLPositionOutputParameters
//!
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//!
//! \param OP
//! Object to be copied
//!
//! \note
//! This is only the base class for the classes\n\n
//! <ul>
//!  <li>RMLPositionOutputParameters and</li>
//!  <li>RMLVelocityOutputParameters,\n\n</li>
//! </ul>
//! such that the constructor is declared \c protected.
//  ----------------------------------------------------------
    RMLOutputParameters(const RMLOutputParameters &OP)
    {
        unsigned int    i                           =   0                                                       ;

        this->TrajectoryIsPhaseSynchronized         =   OP.IsTrajectoryPhaseSynchronized()                      ;

        this->NumberOfDOFs                          =   OP.GetNumberOfDOFs()                                    ;

        this->SynchronizationTime                   =   OP.GetSynchronizationTime()                             ;

        this->ANewCalculationWasPerformed           =   OP.WasACompleteComputationPerformedDuringTheLastCycle() ;

        this->DOFWithTheGreatestExecutionTime       =   OP.GetDOFWithTheGreatestExecutionTime()                 ;

        this->NewPositionVector                     =   new RMLDoubleVector(this->NumberOfDOFs)                 ;
        this->NewVelocityVector                     =   new RMLDoubleVector(this->NumberOfDOFs)                 ;
        this->NewAccelerationVector                 =   new RMLDoubleVector(this->NumberOfDOFs)                 ;
        this->MinExtremaTimesVector                 =   new RMLDoubleVector(this->NumberOfDOFs)                 ;
        this->MaxExtremaTimesVector                 =   new RMLDoubleVector(this->NumberOfDOFs)                 ;
        this->MinPosExtremaPositionVectorOnly       =   new RMLDoubleVector(this->NumberOfDOFs)                 ;
        this->MaxPosExtremaPositionVectorOnly       =   new RMLDoubleVector(this->NumberOfDOFs)                 ;
        this->ExecutionTimes                        =   new RMLDoubleVector(this->NumberOfDOFs)                 ;

        this->MinPosExtremaPositionVectorArray      =   new RMLDoubleVector *[this->NumberOfDOFs]               ;
        this->MinPosExtremaVelocityVectorArray      =   new RMLDoubleVector *[this->NumberOfDOFs]               ;
        this->MinPosExtremaAccelerationVectorArray  =   new RMLDoubleVector *[this->NumberOfDOFs]               ;
        this->MaxPosExtremaPositionVectorArray      =   new RMLDoubleVector *[this->NumberOfDOFs]               ;
        this->MaxPosExtremaVelocityVectorArray      =   new RMLDoubleVector *[this->NumberOfDOFs]               ;
        this->MaxPosExtremaAccelerationVectorArray  =   new RMLDoubleVector *[this->NumberOfDOFs]               ;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            (this->MinPosExtremaPositionVectorArray)        [i] =   new RMLDoubleVector(this->NumberOfDOFs)     ;
            (this->MinPosExtremaVelocityVectorArray)        [i] =   new RMLDoubleVector(this->NumberOfDOFs)     ;
            (this->MinPosExtremaAccelerationVectorArray)    [i] =   new RMLDoubleVector(this->NumberOfDOFs)     ;
            (this->MaxPosExtremaPositionVectorArray)        [i] =   new RMLDoubleVector(this->NumberOfDOFs)     ;
            (this->MaxPosExtremaVelocityVectorArray)        [i] =   new RMLDoubleVector(this->NumberOfDOFs)     ;
            (this->MaxPosExtremaAccelerationVectorArray)    [i] =   new RMLDoubleVector(this->NumberOfDOFs)     ;
        }

        *this               =   OP                                                                              ;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn void Echo(FILE* FileHandler = stdout) const
//!
//! \brief
//! Prints the new state of motion of the output parameters to *FileHandler
//!
//! \param FileHandler
//! File handler for the output
//!
//! \warning
//! The usage of this method is \b not real-time capable.
//  ----------------------------------------------------------
    void Echo(FILE* FileHandler = stdout) const
    {
        unsigned int        i   =   0;

        if (FileHandler == NULL)
        {
            return;
        }

        fprintf(FileHandler,   "New position vector        : ");
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            fprintf(FileHandler, " %.20le ", this->NewPositionVector->VecData[i]);
        }
        fprintf(FileHandler, "\nNew velocity vector        : ");
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            fprintf(FileHandler, " %.20le ", this->NewVelocityVector->VecData[i]);
        }
        fprintf(FileHandler, "\nNew acceleration vector    : ");
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            fprintf(FileHandler, " %.20le ", this->NewAccelerationVector->VecData[i]);
        }
        fprintf(FileHandler, "\n");
        return;
    }


public:

//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLOutputParameters(void)
//!
//! \brief
//! Destructor of class RMLOutputParameters
//  ----------------------------------------------------------
    ~RMLOutputParameters(void)
    {
        unsigned int        i   =   0;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            delete ((this->MinPosExtremaPositionVectorArray)        [i])    ;
            delete ((this->MinPosExtremaVelocityVectorArray)        [i])    ;
            delete ((this->MinPosExtremaAccelerationVectorArray)    [i])    ;
            delete ((this->MaxPosExtremaPositionVectorArray)        [i])    ;
            delete ((this->MaxPosExtremaVelocityVectorArray)        [i])    ;
            delete ((this->MaxPosExtremaAccelerationVectorArray)    [i])    ;
        }

        delete this->NewPositionVector                                      ;
        delete this->NewVelocityVector                                      ;
        delete this->NewAccelerationVector                                  ;
        delete this->MinPosExtremaPositionVectorArray                       ;
        delete this->MinPosExtremaVelocityVectorArray                       ;
        delete this->MinPosExtremaAccelerationVectorArray                   ;
        delete this->MaxPosExtremaPositionVectorArray                       ;
        delete this->MaxPosExtremaVelocityVectorArray                       ;
        delete this->MaxPosExtremaAccelerationVectorArray                   ;
        delete this->MinExtremaTimesVector                                  ;
        delete this->MaxExtremaTimesVector                                  ;
        delete this->MinPosExtremaPositionVectorOnly                        ;
        delete this->MaxPosExtremaPositionVectorOnly                        ;
        delete this->ExecutionTimes                                         ;

        this->NewPositionVector                     =   NULL                ;
        this->NewVelocityVector                     =   NULL                ;
        this->NewAccelerationVector                 =   NULL                ;
        this->MinPosExtremaPositionVectorArray      =   NULL                ;
        this->MinPosExtremaVelocityVectorArray      =   NULL                ;
        this->MinPosExtremaAccelerationVectorArray  =   NULL                ;
        this->MaxPosExtremaPositionVectorArray      =   NULL                ;
        this->MaxPosExtremaVelocityVectorArray      =   NULL                ;
        this->MaxPosExtremaAccelerationVectorArray  =   NULL                ;
        this->MinExtremaTimesVector                 =   NULL                ;
        this->MaxExtremaTimesVector                 =   NULL                ;
        this->MinPosExtremaPositionVectorOnly       =   NULL                ;
        this->MaxPosExtremaPositionVectorOnly       =   NULL                ;
        this->ExecutionTimes                        =   NULL                ;

        this->NumberOfDOFs                          =   0                   ;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLOutputParameters &operator = (const RMLOutputParameters &OP)
//!
//! \brief
//! Copy operator
//!
//! \param OP
//! RMLOutputParameters object to be copied
//  ----------------------------------------------------------
    RMLOutputParameters &operator = (const RMLOutputParameters &OP)
    {
        unsigned int        i                   =   0                                   ;

        this->NumberOfDOFs                      =   OP.NumberOfDOFs                     ;
        this->TrajectoryIsPhaseSynchronized     =   OP.TrajectoryIsPhaseSynchronized    ;
        this->ANewCalculationWasPerformed       =   OP.ANewCalculationWasPerformed      ;
        this->SynchronizationTime               =   OP.SynchronizationTime              ;
        this->DOFWithTheGreatestExecutionTime   =   OP.DOFWithTheGreatestExecutionTime  ;

        OP.GetNewPositionVector     (this->NewPositionVector                    )       ;
        OP.GetNewVelocityVector     (this->NewVelocityVector                    )       ;
        OP.GetNewAccelerationVector (this->NewAccelerationVector                )       ;
        OP.GetTimesAtMinPosition    (this->MinExtremaTimesVector                )       ;
        OP.GetTimesAtMaxPosition    (this->MaxExtremaTimesVector                )       ;
        OP.GetExecutionTimes        (this->ExecutionTimes                       )       ;

        OP.GetPositionalExtrema(    this->MinPosExtremaPositionVectorOnly
                                ,   this->MaxPosExtremaPositionVectorOnly       )       ;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            OP.GetMotionStateAtMinPosForOneDOF(     i
                                                ,   (this->MinPosExtremaPositionVectorArray)[i]
                                                ,   (this->MinPosExtremaVelocityVectorArray)[i]
                                                ,   (this->MinPosExtremaAccelerationVectorArray)[i]);

            OP.GetMotionStateAtMaxPosForOneDOF(     i
                                                ,   (this->MaxPosExtremaPositionVectorArray)[i]
                                                ,   (this->MaxPosExtremaVelocityVectorArray)[i]
                                                ,   (this->MaxPosExtremaAccelerationVectorArray)[i]);
        }

        return(*this);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewPositionVector(RMLDoubleVector *OutputVector) const
//!
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! new position vector \f$ \vec{P}_{i+1} \f$ to the \c RMLDoubleVector
//! object referred to by \c OutputVector
//!
//! \param OutputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be
//! copied
//!
//! \sa GetNewPositionVector(double *OutputVector, const unsigned int &SizeInBytes) const
//! \sa GetNewPositionVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewPositionVector(RMLDoubleVector *OutputVector) const
    {
        *OutputVector   =   *(this->NewPositionVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewPositionVector(double *OutputVector, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the array of \c double values representing the new
//! position vector \f$ \vec{P}_{i+1} \f$ to the memory pointed to by
//! \c OutputVector
//!
//! \param OutputVector
//! A pointer to an array of \c double values, to which the data will be
//! copied
//!
//! \param SizeInBytes
//! The size of available memory at the location pointed to by
//! \c OutputVector. To assure safety and to prevent from prohibited writing
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//!
//! \sa GetNewPositionVector(RMLDoubleVector *OutputVector) const
//! \sa GetNewPositionVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewPositionVector(       double              *OutputVector
                                        ,   const unsigned int  &SizeInBytes) const
    {
        memcpy(     (void*)OutputVector
                ,   (void*)this->NewPositionVector->VecData
                ,   SizeInBytes );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewPositionVectorElement(double *OutputValue, const unsigned int &Index) const
//!
//! \brief
//! Copies one element of the new selection vector
//! \f$ \vec{P}_{i+1} \f$ to the memory pointed to by \c OutputValue
//!
//! \param OutputValue
//! A pointer to one \c double value, to which the desired vector element
//! will be copied
//!
//! \param Index
//! Specifies the desired element of the vector. The element numbering
//! starts with \em 0 (zero). If this value is greater the number
//! of vector elements, a value of \em 0.0 will be written to the memory
//! pointed to by \c OutputValue.
//!
//! \sa GetNewPositionVector(RMLDoubleVector *OutputVector) const
//! \sa GetNewPositionVectorElement(double *OutputValue, const unsigned int &Index) const
//! \sa GetNewPositionVectorElement(const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewPositionVectorElement(    double              *OutputValue
                                            ,   const unsigned int  &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->NewPositionVector->GetVecDim() ) )
        {
            *OutputValue    =   0.0;
        }
        else
        {
            *OutputValue    =   (*this->NewPositionVector)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetNewPositionVectorElement(const unsigned int &Index) const
//!
//! \brief
//! Returns one single element of the new selection vector
//! \f$ \vec{P}_{i+1} \f$
//!
//! \param Index
//! Specifies the desired element of the vector. The index of the first
//! vector element is \em 0 (zero). If the value of \c Index value is
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c OutputValue.
//!
//! \sa GetNewPositionVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline double GetNewPositionVectorElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->NewPositionVector->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {
            return( (*this->NewPositionVector)[Index] );
        }
    }


// #############################################################################


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewVelocityVector(RMLDoubleVector *OutputVector) const
//!
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! new velocity vector \f$ \vec{V}_{i+1} \f$ to the \c RMLDoubleVector
//! object referred to by \c OutputVector
//!
//! \param OutputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be
//! copied
//!
//! \sa GetNewVelocityVector(double *OutputVector, const unsigned int &SizeInBytes) const
//! \sa GetNewVelocityVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewVelocityVector(RMLDoubleVector *OutputVector) const
    {
        *OutputVector   =   *(this->NewVelocityVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewVelocityVector(double *OutputVector, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the array of \c double values representing the new
//! velocity vector \f$ \vec{V}_{i+1} \f$ to the memory pointed to by
//! \c OutputVector
//!
//! \param OutputVector
//! A pointer to an array of \c double values, to which the data will be
//! copied
//!
//! \param SizeInBytes
//! The size of available memory at the location pointed to by
//! \c OutputVector. To assure safety and to prevent from prohibited writing
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//!
//! \sa GetNewVelocityVector(RMLDoubleVector *OutputVector) const
//! \sa GetNewVelocityVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewVelocityVector(       double              *OutputVector
                                        ,   const unsigned int  &SizeInBytes) const
    {
        memcpy(     (void*)OutputVector
                ,   (void*)this->NewVelocityVector->VecData
                ,   SizeInBytes );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewVelocityVectorElement(double *OutputValue, const unsigned int &Index) const
//!
//! \brief
//! Copies one element of the new selection vector
//! \f$ \vec{V}_{i+1} \f$ to the memory pointed to by \c OutputValue
//!
//! \param OutputValue
//! A pointer to one \c double value, to which the desired vector element
//! will be copied
//!
//! \param Index
//! Specifies the desired element of the vector. The element numbering
//! starts with \em 0 (zero). If this value is greater the number
//! of vector elements, a value of \em 0.0 will be written to the memory
//! pointed to by \c OutputValue.
//!
//! \sa GetNewVelocityVector(RMLDoubleVector *OutputVector) const
//! \sa GetNewVelocityVectorElement(double *OutputValue, const unsigned int &Index) const
//! \sa GetNewVelocityVectorElement(const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewVelocityVectorElement(    double              *OutputValue
                                            ,   const unsigned int  &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->NewVelocityVector->GetVecDim() ) )
        {
            *OutputValue    =   0.0;
        }
        else
        {
            *OutputValue    =   (*this->NewVelocityVector)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetNewVelocityVectorElement(const unsigned int &Index) const
//!
//! \brief
//! Returns one single element of the new selection vector
//! \f$ \vec{V}_{i+1} \f$
//!
//! \param Index
//! Specifies the desired element of the vector. The index of the first
//! vector element is \em 0 (zero). If the value of \c Index value is
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c OutputValue.
//!
//! \sa GetNewVelocityVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline double GetNewVelocityVectorElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->NewVelocityVector->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {
            return( (*this->NewVelocityVector)[Index] );
        }
    }


// #############################################################################


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewAccelerationVector(RMLDoubleVector *OutputVector) const
//!
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! new acceleration vector \f$ \vec{A}_{i+1} \f$ to the \c RMLDoubleVector
//! object referred to by \c OutputVector
//!
//! \param OutputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be
//! copied
//!
//! \sa GetNewAccelerationVector(double *OutputVector, const unsigned int &SizeInBytes) const
//! \sa GetNewAccelerationVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewAccelerationVector(RMLDoubleVector *OutputVector) const
    {
        *OutputVector   =   *(this->NewAccelerationVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewAccelerationVector(double *OutputVector, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the array of \c double values representing the new
//! acceleration vector \f$ \vec{A}_{i+1} \f$ to the memory pointed to by
//! \c OutputVector
//!
//! \param OutputVector
//! A pointer to an array of \c double values, to which the data will be
//! copied
//!
//! \param SizeInBytes
//! The size of available memory at the location pointed to by
//! \c OutputVector. To assure safety and to prevent from prohibited writing
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//!
//! \sa GetNewAccelerationVector(RMLDoubleVector *OutputVector) const
//! \sa GetNewAccelerationVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewAccelerationVector(       double              *OutputVector
                                            ,   const unsigned int  &SizeInBytes) const
    {
        memcpy(     (void*)OutputVector
                ,   (void*)this->NewAccelerationVector->VecData
                ,   SizeInBytes );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewAccelerationVectorElement(double *OutputValue, const unsigned int &Index) const
//!
//! \brief
//! Copies one element of the new selection vector
//! \f$ \vec{A}_{i+1} \f$ to the memory pointed to by \c OutputValue
//!
//! \param OutputValue
//! A pointer to one \c double value, to which the desired vector element
//! will be copied
//!
//! \param Index
//! Specifies the desired element of the vector. The element numbering
//! starts with \em 0 (zero). If this value is greater the number
//! of vector elements, a value of \em 0.0 will be written to the memory
//! pointed to by \c OutputValue.
//!
//! \sa GetNewAccelerationVector(RMLDoubleVector *OutputVector) const
//! \sa GetNewAccelerationVectorElement(double *OutputValue, const unsigned int &Index) const
//! \sa GetNewAccelerationVectorElement(const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewAccelerationVectorElement(    double              *OutputValue
                                                ,   const unsigned int  &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->NewAccelerationVector->GetVecDim() ) )
        {
            *OutputValue    =   0.0;
        }
        else
        {
            *OutputValue    =   (*this->NewAccelerationVector)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetNewAccelerationVectorElement(const unsigned int &Index) const
//!
//! \brief
//! Returns one single element of the new selection vector
//! \f$ \vec{A}_{i+1} \f$
//!
//! \param Index
//! Specifies the desired element of the vector. The index of the first
//! vector element is \em 0 (zero). If the value of \c Index value is
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c OutputValue.
//!
//! \sa GetNewAccelerationVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline double GetNewAccelerationVectorElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->NewAccelerationVector->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {
            return( (*this->NewAccelerationVector)[Index] );
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const
//!
//! \brief
//! Copies two \c RMLDoubleVector objects that contain the minimum and
//! maximum positions, which are reached until the target state of motion
//! is reached.
//!
//! \param MinimumPositionVector
//! A pointer to an \c RMLDoubleVector object, to which the vector of
//! minimum positions will be copied to.
//!
//! \param MaximumPositionVector
//! A pointer to an \c RMLDoubleVector object, to which the vector of
//! maximum positions will be copied to.
//!
//! \sa GetPositionalExtrema(double *MinimumPositionVector, double *MaximumPositionVector, const unsigned int &SizeInBytes) const
//! \sa RMLVector
//! \sa RMLDoubleVector
//  ----------------------------------------------------------
    inline void GetPositionalExtrema(       RMLDoubleVector     *MinimumPositionVector
                                        ,   RMLDoubleVector     *MaximumPositionVector) const
    {
        *MinimumPositionVector  =   *(this->MinPosExtremaPositionVectorOnly);
        *MaximumPositionVector  =   *(this->MaxPosExtremaPositionVectorOnly);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetPositionalExtrema(double *MinimumPositionVector, double *MaximumPositionVector, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies two \c RMLDoubleVector objects that contain the minimum and
//! maximum positions, which are reached until the target state of motion
//! is reached.
//!
//! \param MinimumPositionVector
//! A pointer to a \c double array, to which the elements of the vector of
//! minimum positions will be copied to.
//!
//! \param MaximumPositionVector
//! A pointer to a \c double array, to which the elements of the vector of
//! maximum positions will be copied to.
//!
//! \param SizeInBytes
//! The size of available memory at the location pointed to by
//! \c MinimumPositionVector or MaximumPositionVector, respectively.
//! To assure safety and to prevent from prohibited writing
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//!
//! \sa GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const
//  ----------------------------------------------------------
    inline void GetPositionalExtrema(       double              *MinimumPositionVector
                                        ,   double              *MaximumPositionVector
                                        ,   const unsigned int  &SizeInBytes)  const
    {
        memcpy(     (void*)MinimumPositionVector
                ,   (void*)(this->MinPosExtremaPositionVectorOnly->VecData)
                ,   SizeInBytes                                                     );

        memcpy(     (void*)MaximumPositionVector
                ,   (void*)(this->MaxPosExtremaPositionVectorOnly->VecData)
                ,   SizeInBytes                                                     );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetTimesAtMinPosition(RMLDoubleVector *ExtremaTimes) const
//!
//! \brief
//! Copies the contents of a \c RMLDoubleVector object that contains the
//! times (in seconds) at which the minimum positions are reached to the
//! \c RMLDoubleVector object referred to by \c ExtremaTimes
//!
//! \param ExtremaTimes
//! A pointer to an \c RMLDoubleVector object, to which the data will be
//! copied
//!
//! \sa GetTimesAtMaxPosition(RMLDoubleVector *ExtremaTimes) const
//! \sa GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const
//! \sa GetPositionalExtrema(double *MinimumPositionVector, double *MaximumPositionVector, const unsigned int &SizeInBytes) const
//! \sa RMLDoubleVector
//  ----------------------------------------------------------
    inline void GetTimesAtMinPosition(RMLDoubleVector *ExtremaTimes) const
    {
        *ExtremaTimes   =   *(this->MinExtremaTimesVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetTimesAtMaxPosition(RMLDoubleVector *ExtremaTimes) const
//!
//! \brief
//! Copies the contents of a \c RMLDoubleVector object that contains the
//! times (in seconds) at which the maximum positions are reached to the
//! \c RMLDoubleVector object referred to by \c ExtremaTimes
//!
//! \param ExtremaTimes
//! A pointer to an \c RMLDoubleVector object, to which the data will be
//! copied
//!
//! \sa GetTimesAtMinPosition(RMLDoubleVector *ExtremaTimes) const
//! \sa GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const
//! \sa GetPositionalExtrema(double *MinimumPositionVector, double *MaximumPositionVector, const unsigned int &SizeInBytes) const
//! \sa RMLDoubleVector
//  ----------------------------------------------------------
    inline void GetTimesAtMaxPosition(RMLDoubleVector *ExtremaTimes) const
    {
        *ExtremaTimes   =   *(this->MaxExtremaTimesVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetTimesAtMinPosition(double*ExtremaTimes, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the array of \c double values that contain the time values
//! (in seconds) at which the minimum positions are reached to the array
//! of \c double values referred to by \c ExtremaTimes
//!
//! \param ExtremaTimes
//! A pointer to an array of \c double values, to which the data will be
//! copied
//!
//! \param SizeInBytes
//! The size of available memory at the location pointed to by
//! \c ExtremaTimes. To assure safety and to prevent from prohibited writing
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//!
//! \sa GetTimesAtMinPosition(RMLDoubleVector *ExtremaTimes) const
//! \sa GetTimesAtMaxPosition(double*ExtremaTimes, const unsigned int &SizeInBytes) const
//  ----------------------------------------------------------
    inline void GetTimesAtMinPosition(      double              *ExtremaTimes
                                        ,   const unsigned int  &SizeInBytes) const
    {
        memcpy(     (void*)ExtremaTimes
                ,   (void*)(this->MinExtremaTimesVector->VecData)
                ,   SizeInBytes                                         );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetTimesAtMaxPosition(double*ExtremaTimes, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the array of \c double values that contain the time values
//! (in seconds) at which the maximum positions are reached to the array
//! of \c double values referred to by \c ExtremaTimes
//!
//! \param ExtremaTimes
//! A pointer to an array of \c double values, to which the data will be
//! copied
//!
//! \param SizeInBytes
//! The size of available memory at the location pointed to by
//! \c ExtremaTimes. To assure safety and to prevent from prohibited writing
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//!
//! \sa GetTimesAtMaxPosition(RMLDoubleVector *ExtremaTimes) const
//! \sa GetTimesAtMinPosition(double*ExtremaTimes, const unsigned int &SizeInBytes) const
//  ----------------------------------------------------------
    inline void GetTimesAtMaxPosition(      double              *ExtremaTimes
                                        ,   const unsigned int  &SizeInBytes) const
    {
        memcpy(     (void*)ExtremaTimes
                ,   (void*)(this->MaxExtremaTimesVector->VecData)
                ,   SizeInBytes                                         );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const
//!
//! \brief
//! Copies the motion state, at which the position of the degree of freedom
//! with the index \c DOF has reached its minimum, to the three referred
//! \c RMLDoubleVector objects.
//!
//! \param DOF
//! The index of the degree of freedom, whose minimum position is regarded.
//! The motion state of the time instant, at which the minimum position
//! value of this degree of freedom is reached, will be copied to
//! \c PositionVector, \c VelocityVector, and \c AccelerationVector.
//!
//! \param PositionVector
//! A pointer to an \c RMLDoubleVector object, to which the position
//! vector at the instant, at which the minimum position for the degree of
//! freedom \c DOF is reached, will be copied.
//!
//! \param VelocityVector
//! A pointer to an \c RMLDoubleVector object, to which the velocity
//! vector at the instant, at which the minimum position for the degree of
//! freedom \c DOF is reached, will be copied.
//!
//! \param AccelerationVector
//! A pointer to an \c RMLDoubleVector object, to which the acceleration
//! vector at the instant, at which the minimum position for the degree of
//! freedom \c DOF is reached, will be copied.
//!
//! \return
//!  - RMLOutputParameters::RETURN_ERROR, if the value of \c DOF is greater
//!    than or equal to the number of degrees of freedom
//!  - RMLOutputParameters::RETURN_SUCCESS, else
//!
//! \sa GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const
//! \sa GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const
//! \sa GetTimesAtMinPosition(RMLDoubleVector *ExtremaTimes) const
//! \sa GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const
//! \sa RMLDoubleVector
//  ----------------------------------------------------------
    inline int GetMotionStateAtMinPosForOneDOF(     const unsigned int  &DOF
                                                ,   RMLDoubleVector     *PositionVector
                                                ,   RMLDoubleVector     *VelocityVector
                                                ,   RMLDoubleVector     *AccelerationVector) const
    {
        if (DOF >= this->NumberOfDOFs)
        {
            return(RMLOutputParameters::RETURN_ERROR);
        }

        *PositionVector     =   *((this->MinPosExtremaPositionVectorArray       )[DOF]) ;
        *VelocityVector     =   *((this->MinPosExtremaVelocityVectorArray       )[DOF]) ;
        *AccelerationVector =   *((this->MinPosExtremaAccelerationVectorArray   )[DOF]) ;

        return(RMLOutputParameters::RETURN_SUCCESS);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the motion state, at which the position of the degree of freedom
//! with the index \c DOF has reached its minimum, to the three referred
//! arrays of \c double values.
//!
//! \param DOF
//! The index of the degree of freedom, whose minimum position is regarded.
//! The motion state of the time instant, at which the minimum position
//! value of this degree of freedom is reached, will be copied to
//! \c PositionVector, \c VelocityVector, and \c AccelerationVector.
//!
//! \param PositionVector
//! A pointer to an array of \c double values, to which the position
//! vector at the instant, at which the minimum position for the degree of
//! freedom \c DOF is reached, will be copied.
//!
//! \param VelocityVector
//! A pointer to an array of \c double values, to which the velocity
//! vector at the instant, at which the minimum position for the degree of
//! freedom \c DOF is reached, will be copied.
//!
//! \param AccelerationVector
//! A pointer to an array of \c double values, to which the acceleration
//! vector at the instant, at which the minimum position for the degree of
//! freedom \c DOF is reached, will be copied.
//!
//! \param SizeInBytes
//! The size of available memory at the each of the locations pointed to by
//! \c PositionVector, \c VelocityVector, and \c AccelerationVector.
//! To assure safety and to prevent from prohibited writing
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//!
//! \return
//!  - RMLOutputParameters::RETURN_ERROR, if the value of \c DOF is greater
//!    than or equal to the number of degrees of freedom
//!  - RMLOutputParameters::RETURN_SUCCESS, else
//!
//! \sa GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const
//! \sa GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const
//! \sa GetTimesAtMinPosition(RMLDoubleVector *ExtremaTimes) const
//! \sa GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const
//! \sa RMLDoubleVector
//  ----------------------------------------------------------
    inline int GetMotionStateAtMinPosForOneDOF(     const unsigned int  &DOF
                                                ,   double              *PositionVector
                                                ,   double              *VelocityVector
                                                ,   double              *AccelerationVector
                                                ,   const unsigned int  &SizeInBytes) const
    {
        if (DOF >= this->NumberOfDOFs)
        {
            return(RMLOutputParameters::RETURN_ERROR);
        }

        memcpy(     (void*)PositionVector
                ,   (void*)(((this->MinPosExtremaPositionVectorArray)[DOF])->VecData)
                ,   SizeInBytes                                                             );

        memcpy(     (void*)VelocityVector
                ,   (void*)(((this->MinPosExtremaVelocityVectorArray)[DOF])->VecData)
                ,   SizeInBytes                                                             );

        memcpy(     (void*)AccelerationVector
                ,   (void*)(((this->MinPosExtremaAccelerationVectorArray)[DOF])->VecData)
                ,   SizeInBytes                                                                 );

        return(RMLOutputParameters::RETURN_SUCCESS);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const
//!
//! \brief
//! Copies the motion state, at which the position of the degree of freedom
//! with the index \c DOF has reached its maximum, to the three referred
//! \c RMLDoubleVector objects.
//!
//! \param DOF
//! The index of the degree of freedom, whose maximum position is regarded.
//! The motion state of the time instant, at which the maximum position
//! value of this degree of freedom is reached, will be copied to
//! \c PositionVector, \c VelocityVector, and \c AccelerationVector.
//!
//! \param PositionVector
//! A pointer to an \c RMLDoubleVector object, to which the position
//! vector at the instant, at which the maximum position for the degree of
//! freedom \c DOF is reached, will be copied.
//!
//! \param VelocityVector
//! A pointer to an \c RMLDoubleVector object, to which the velocity
//! vector at the instant, at which the maximum position for the degree of
//! freedom \c DOF is reached, will be copied.
//!
//! \param AccelerationVector
//! A pointer to an \c RMLDoubleVector object, to which the acceleration
//! vector at the instant, at which the maximum position for the degree of
//! freedom \c DOF is reached, will be copied.
//!
//! \return
//!  - RMLOutputParameters::RETURN_ERROR, if the value of \c DOF is greater
//!    than or equal to the number of degrees of freedom
//!  - RMLOutputParameters::RETURN_SUCCESS, else
//!
//! \sa GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const
//! \sa GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const
//! \sa GetTimesAtMaxPosition(RMLDoubleVector *ExtremaTimes) const
//! \sa GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const
//! \sa RMLDoubleVector
//  ----------------------------------------------------------
    inline int GetMotionStateAtMaxPosForOneDOF(     const unsigned int  &DOF
                                                ,   RMLDoubleVector     *PositionVector
                                                ,   RMLDoubleVector     *VelocityVector
                                                ,   RMLDoubleVector     *AccelerationVector) const
    {
        if (DOF >= this->NumberOfDOFs)
        {
            return(RMLOutputParameters::RETURN_ERROR);
        }

        *PositionVector     =   *((this->MaxPosExtremaPositionVectorArray       )[DOF]) ;
        *VelocityVector     =   *((this->MaxPosExtremaVelocityVectorArray       )[DOF]) ;
        *AccelerationVector =   *((this->MaxPosExtremaAccelerationVectorArray   )[DOF]) ;

        return(RMLOutputParameters::RETURN_SUCCESS);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the motion state, at which the position of the degree of freedom
//! with the index \c DOF has reached its maximum, to the three referred
//! arrays of \c double values.
//!
//! \param DOF
//! The index of the degree of freedom, whose maximum position is regarded.
//! The motion state of the time instant, at which the maximum position
//! value of this degree of freedom is reached, will be copied to
//! \c PositionVector, \c VelocityVector, and \c AccelerationVector.
//!
//! \param PositionVector
//! A pointer to an array of \c double values, to which the position
//! vector at the instant, at which the maximum position for the degree of
//! freedom \c DOF is reached, will be copied.
//!
//! \param VelocityVector
//! A pointer to an array of \c double values, to which the velocity
//! vector at the instant, at which the maximum position for the degree of
//! freedom \c DOF is reached, will be copied.
//!
//! \param AccelerationVector
//! A pointer to an array of \c double values, to which the acceleration
//! vector at the instant, at which the maximum position for the degree of
//! freedom \c DOF is reached, will be copied.
//!
//! \param SizeInBytes
//! The size of available memory at the each of the locations pointed to by
//! \c PositionVector, \c VelocityVector, and \c AccelerationVector.
//! To assure safety and to prevent from prohibited writing
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//!
//! \return
//!  - RMLOutputParameters::RETURN_ERROR, if the value of \c DOF is greater
//!    than or equal to the number of degrees of freedom
//!  - RMLOutputParameters::RETURN_SUCCESS, else
//!
//! \sa GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const
//! \sa GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const
//! \sa GetTimesAtMaxPosition(RMLDoubleVector *ExtremaTimes) const
//! \sa GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const
//! \sa RMLDoubleVector
//  ----------------------------------------------------------
    inline int GetMotionStateAtMaxPosForOneDOF(     const unsigned int  &DOF
                                                ,   double              *PositionVector
                                                ,   double              *VelocityVector
                                                ,   double              *AccelerationVector
                                                ,   const unsigned int  &SizeInBytes) const
    {
        if (DOF >= this->NumberOfDOFs)
        {
            return(RMLOutputParameters::RETURN_ERROR);
        }

        memcpy(     (void*)PositionVector
                ,   (void*)(((this->MaxPosExtremaPositionVectorArray)[DOF])->VecData)
                ,   SizeInBytes                                                             );

        memcpy(     (void*)VelocityVector
                ,   (void*)(((this->MaxPosExtremaVelocityVectorArray)[DOF])->VecData)
                ,   SizeInBytes                                                             );

        memcpy(     (void*)AccelerationVector
                ,   (void*)(((this->MaxPosExtremaAccelerationVectorArray)[DOF])->VecData)
                ,   SizeInBytes                                                                 );

        return(RMLOutputParameters::RETURN_SUCCESS);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline unsigned int GetNumberOfDOFs(void) const
//!
//! \brief
//! Returns the number of degrees of freedom
//!
//! \return
//! The number of degrees of freedom.
//  ----------------------------------------------------------
    inline unsigned int GetNumberOfDOFs(void) const
    {
        return(this->NumberOfDOFs);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline bool WasACompleteComputationPerformedDuringTheLastCycle(void) const
//!
//! \brief
//! Indicates, whether a new computation was performed in the last cycle
//!
//! \details
//! \copydetails RMLOutputParameters::ANewCalculationWasPerformed
//!
//! \returns
//! - \c true if a new computation was performed
//! - \c false if the previously calculated trajectory parameters did not
//! change and were used.
//  ----------------------------------------------------------
    inline bool WasACompleteComputationPerformedDuringTheLastCycle(void) const
    {
        return(this->ANewCalculationWasPerformed);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline bool IsTrajectoryPhaseSynchronized(void) const
//!
//! \brief
//! Indicates whether the currently calculated trajectory is phase-
//! synchronized or only time-synchronized
//!
//! \return
//! The method returns \c true if the trajectory is phase-synchronized
//! and \c false if it is time-synchronized.
//  ----------------------------------------------------------
    inline bool IsTrajectoryPhaseSynchronized(void) const
    {
        return(this->TrajectoryIsPhaseSynchronized);
    }

//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetSynchronizationTime(void) const
//!
//! \brief
//! Returns the synchronization time
//!
//! \details
//! The position-based On-Line Trajectory Generation algorithm transfers
//! all selected degrees of freedom into their desired target state of
//! motion, such that all of them reach the target state of motion
//! \f$_{k}\vec{M}_{i}^{\,trgt} \f$ at the very same time instant, that is,
//! at the minimum possible synchronization time \f$ t_{i}^{,sync} \f$.
//! If this value is used as an output value of the velocity-based
//! On-Line Trajectory Generation algorithm, this time determines when
//! all selected degree of freedom will reach the desired target velocity.
//!
//! \return
//! The value of the synchronization time in seconds
//!
//! \sa RMLOutputParameters::SynchronizationTime
//  ----------------------------------------------------------
    inline double GetSynchronizationTime(void) const
    {
        return(this->SynchronizationTime);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline unsigned int GetDOFWithTheGreatestExecutionTime(void) const
//!
//! \brief
//! Returns the index of the degree of freedom with the greatest trajectory
//! execution time
//!
//! \sa RMLOutputParameters::GetGreatestExecutionTime()
//  ----------------------------------------------------------
    inline unsigned int GetDOFWithTheGreatestExecutionTime(void) const
    {
        return(this->DOFWithTheGreatestExecutionTime);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetExecutionTimes(RMLDoubleVector *OutputVector) const
//!
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! execution times for each degree of freedom, at which the
//! desired target velocity \f$\ _{k}V_{i}^{\,trgt} \f$ is reached, to the
//! \c RMLDoubleVector object referred to by \c OutputVector
//!
//! \param OutputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be
//! copied
//!
//! \sa GetExecutionTimes(double *OutputVector, const unsigned int &SizeInBytes) const
//! \sa GetExecutionTimesElement(double *OutputValue, const unsigned int &Index) const
//! \sa RMLOutputParameters::ExecutionTimes
//  ----------------------------------------------------------
    inline void GetExecutionTimes(RMLDoubleVector *OutputVector) const
    {
        *OutputVector   =   *(this->ExecutionTimes);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetExecutionTimes(double *OutputVector, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the array of \c double values representing the execution times
//! for each degree of freedom, at which the desired target velocity
//! \f$\ _{k}V_{i}^{\,trgt} \f$ is reached, to the memory pointed to by
//! \c OutputVector
//!
//! \param OutputVector
//! A pointer to an array of \c double values, to which the data will be
//! copied
//!
//! \param SizeInBytes
//! The size of available memory at the location pointed to by
//! \c OutputVector. To assure safety and to prevent from prohibited writing
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//!
//! \sa GetExecutionTimes(RMLDoubleVector *OutputVector) const
//! \sa GetExecutionTimesElement(double *OutputValue, const unsigned int &Index) const
//! \sa RMLOutputParameters::ExecutionTimes
//  ----------------------------------------------------------
    inline void GetExecutionTimes(      double              *OutputVector
                                    ,   const unsigned int  &SizeInBytes) const
    {
        memcpy(     (void*)OutputVector
                ,   (void*)this->ExecutionTimes->VecData
                ,   SizeInBytes );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetExecutionTimesElement(double *OutputValue, const unsigned int &Index) const
//!
//! \brief
//! Copies one element of the execution times for each degree of freedom,
//! at which the desired target velocity \f$\ _{k}V_{i}^{\,trgt} \f$
//! is reached, to the memory pointed to by \c OutputValue
//!
//! \param OutputValue
//! A pointer to one \c double value, to which the desired vector element
//! will be copied
//!
//! \param Index
//! Specifies the desired element of the vector. The element numbering
//! starts with \em 0 (zero). If this value is greater the number
//! of vector elements, a value of \em 0.0 will be written to the memory
//! pointed to by \c OutputValue.
//!
//! \sa GetExecutionTimes(RMLDoubleVector *OutputVector) const
//! \sa GetExecutionTimesElement(double *OutputValue, const unsigned int &Index) const
//! \sa GetExecutionTimesElement(const unsigned int &Index) const
//! \sa RMLOutputParameters::ExecutionTimes
//  ----------------------------------------------------------
    inline void GetExecutionTimesElement(       double              *OutputValue
                                            ,   const unsigned int  &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->ExecutionTimes->GetVecDim() ) )
        {
            *OutputValue    =   0.0;
        }
        else
        {
            *OutputValue    =   (*this->ExecutionTimes)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetExecutionTimesElement(const unsigned int &Index) const
//!
//! \brief
//! Returns one single element of the execution times
//! for each degree of freedom, at which the desired target velocity
//! \f$\ _{k}V_{i}^{\,trgt} \f$ is reached
//!
//! \param Index
//! Specifies the desired element of the vector. The index of the first
//! vector element is \em 0 (zero). If the value of \c Index value is
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c OutputValue.
//!
//! \sa GetExecutionTimesElement(double *OutputValue, const unsigned int &Index) const
//! \sa RMLOutputParameters::ExecutionTimes
//  ----------------------------------------------------------
    inline double GetExecutionTimesElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->ExecutionTimes->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {
            return( (*this->ExecutionTimes)[Index] );
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetGreatestExecutionTime(void) const
//!
//! \brief
//! Returns the time value in seconds which is required by the degree
//! with the greatest execution to reach its desired target velocity
//!
//! \sa GetGreatestExecutionTime (void) const
//  ----------------------------------------------------------
    inline double GetGreatestExecutionTime(void) const
    {
        return((this->ExecutionTimes->VecData)[this->DOFWithTheGreatestExecutionTime]);
    }


//  ---------------------- Doxygen info ----------------------
//! \var bool ANewCalculationWasPerformed
//!
//! \brief
//! Indicates, whether a new computation was performed in the last cycle
//!
//! \details
//! If the computation of completely new trajectory parameters
//! was performed, this flag will be set to \c true. If the input values
//! remained constant and the output parameters of the last computation
//! cycle were directly fed back to the input parameters, such that the
//! previously computed trajectory did not change, the flag will be set
//! to \c false.
//!
//! \details
//! This attribute can be accessed directly or by using one of the
//! following methods:\n\n
//!  - WasACompleteComputationPerformedDuringTheLastCycle(void) const\n\n
//  ----------------------------------------------------------
    bool                    ANewCalculationWasPerformed;


//  ---------------------- Doxygen info ----------------------
//! \var bool TrajectoryIsPhaseSynchronized
//!
//! \brief
//! Boolean flag that indicates whether the current trajectory is
//! phase-synchronized
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - IsTrajectoryPhaseSynchronized(void) const\n\n
//  ----------------------------------------------------------
    bool                    TrajectoryIsPhaseSynchronized;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int NumberOfDOFs
//!
//! \brief
//! The number of degrees of freedom \f$ K \f$
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - RMLOutputParameters::RMLOutputParameters()\n\n
//!  - GetNumberOfDOFs(void) const\n\n
//  ----------------------------------------------------------
    unsigned int            NumberOfDOFs;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int DOFWithTheGreatestExecutionTime
//!
//! \brief
//! Index of the degree of freedom that requires the greatest execution
//! time to reach its desired target velocity value
//!
//! \details
//! <ul>
//!   <li>In case of non-synchronized trajectories, this integer value
//!       specifies the index of the degree-of-freedom with the greatest
//!       execution time.</li>
//!   <li>In case of time-synchronized trajectories, this integer value
//!       specifies the degree of freedom that determined the
//!       synchronization time.</li>
//!   <li>In case of time-synchronized trajectories, this integer value
//!       contains the lowest index index number of all selected degrees of
//!       freedom.</li>
//!   <li>If more that one degree of freedom feature the (same)
//!       execution time, the lowest index will be used.\n\n</li>
//! </ul>
//!
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetDOFWithTheGreatestExecutionTime(void) const\n\n
//  ----------------------------------------------------------
    unsigned int            DOFWithTheGreatestExecutionTime;


//  ---------------------- Doxygen info ----------------------
//! \var double SynchronizationTime
//!
//! \brief
//! The synchronization time \f$ t_{i}^{\,sync} \f$ in seconds
//!
//! \details
//! If the trajectory is time- or phase-synchronized, this attribute
//! will contain the synchronization time. Otherwise, it is set to zero.
//!
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetSynchronizationTime(void) const\n\n
//  ----------------------------------------------------------
    double                  SynchronizationTime;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *NewPositionVector
//!
//! \brief
//! A pointer to the new position vector \f$ \vec{P}_{i+1} \f$
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetNewPositionVector(RMLDoubleVector *OutputVector) const\n\n
//!  - GetNewPositionVector(double *OutputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetNewPositionVectorElement(double *OutputValue, const unsigned int &Index) const\n\n
//!  - GetNewPositionVectorElement(const unsigned int &Index) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         *NewPositionVector                      ;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *NewVelocityVector
//!
//! \brief
//! A pointer to the new velocity vector \f$ \vec{V}_{i+1} \f$
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetNewVelocityVector(RMLDoubleVector *OutputVector) const\n\n
//!  - GetNewVelocityVector(double *OutputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetNewVelocityVectorElement(double *OutputValue, const unsigned int &Index) const\n\n
//!  - GetNewVelocityVectorElement(const unsigned int &Index) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         *NewVelocityVector                      ;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *NewAccelerationVector
//!
//! \brief
//! A pointer to the new acceleration vector \f$ \vec{A}_{i+1} \f$
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetNewAccelerationVector(RMLDoubleVector *OutputVector) const\n\n
//!  - GetNewAccelerationVector(double *OutputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetNewAccelerationVectorElement(double *OutputValue, const unsigned int &Index) const\n\n
//!  - GetNewAccelerationVectorElement(const unsigned int &Index) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         *NewAccelerationVector                  ;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MinExtremaTimesVector
//!
//! \brief
//! A pointer to an \c RMLDoubleVector object that contains
//! the times at which each degree of freedom reaches its minimum
//! position during the execution of the calculated trajectory.
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetTimesAtMinPosition(RMLDoubleVector *ExtremaTimes) const\n\n
//!  - GetTimesAtMinPosition(double *ExtremaTimes, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         *MinExtremaTimesVector                  ;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MaxExtremaTimesVector
//!
//! \brief
//! A pointer to an \c RMLDoubleVector object that contains
//! the times at which each degree of freedom reaches its maximum
//! position during the execution of the calculated trajectory.
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetTimesAtMaxPosition(RMLDoubleVector *ExtremaTimes) const\n\n
//!  - GetTimesAtMaxPosition(double *ExtremaTimes, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         *MaxExtremaTimesVector                  ;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MinPosExtremaPositionVectorOnly
//!
//! \brief
//! A pointer to an \c RMLDoubleVector object that contains
//! the maximum positions for all degrees of freedom that
//! occur during the execution of the calculated trajectory.
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const\n\n
//!  - GetPositionalExtrema(double *MinimumPositionVector, double *MaximumPositionVector, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         *MinPosExtremaPositionVectorOnly        ;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MaxPosExtremaPositionVectorOnly
//!
//! \brief
//! A pointer to an \c RMLDoubleVector object that contains
//! the maximum positions for all degrees of freedom that
//! occur during the execution of the calculated trajectory.
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const\n\n
//!  - GetPositionalExtrema(double *MinimumPositionVector, double *MaximumPositionVector, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         *MaxPosExtremaPositionVectorOnly        ;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *ExecutionTimes
//!
//! \brief
//! A pointer to an \c RMLDoubleVector object that contains
//! the execution times of all selected degrees of freedom in the case
//! non-synchronized trajectories
//!
//! \details
//! <ul>
//! <li>In case of non-synchronized trajectories, this vector contains
//!     the execution times of all selected degrees of freedom.</li>
//! <li>In the case of time- and phase-synchronized trajectories, this
//!     vector contains the synchronization time for all selected
//!     degree of freedom.</li>
//! <li>The values non-selected degrees of freedom is zero.\n\n</li>
//! </ul>
//!
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetExecutionTimes(RMLDoubleVector *OutputVector) const\n\n
//!  - GetExecutionTimes(double *OutputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetExecutionTimesElement(double *OutputValue, const unsigned int &Index) const\n\n
//!  - GetExecutionTimesElement(const unsigned int &Index) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         *ExecutionTimes;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MinPosExtremaPositionVectorArray
//!
//! \brief
//! A pointer to an array of pointers to \c RMLDoubleVector
//! objects. The number of array elements equals the number of
//! degrees of freedom, and each of these \c RMLDOubleVector
//! objects consists of the same number of entries, such that it
//! is a square matrix. A single vector contains the position
//! vector, that will be achieved when the respective degree of
//! freedom reaches its minimum position.
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const\n\n
//!  - GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         **MinPosExtremaPositionVectorArray      ;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MinPosExtremaVelocityVectorArray
//!
//! \brief
//! A pointer to an array of pointers to \c RMLDoubleVector
//! objects. The number of array elements equals the number of
//! degrees of freedom, and each of these \c RMLDOubleVector
//! objects consists of the same number of entries, such that it
//! is a square matrix. A single vector contains the velocity
//! vector, that will be achieved when the respective degree of
//! freedom reaches its minimum position.
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const\n\n
//!  - GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         **MinPosExtremaVelocityVectorArray      ;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector **MinPosExtremaAccelerationVectorArray
//!
//! \brief
//! A pointer to an array of pointers to \c RMLDoubleVector
//! objects. The number of array elements equals the number of
//! degrees of freedom, and each of these \c RMLDOubleVector
//! objects consists of the same number of entries, such that it
//! is a square matrix. A single vector contains the position
//! vector, that will be achieved when the respective degree of
//! freedom reaches its minimum position.
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const\n\n
//!  - GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         **MinPosExtremaAccelerationVectorArray  ;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MaxPosExtremaPositionVectorArray
//!
//! \brief
//! A pointer to an array of pointers to \c RMLDoubleVector
//! objects. The number of array elements equals the number of
//! degrees of freedom, and each of these \c RMLDOubleVector
//! objects consists of the same number of entries, such that it
//! is a square matrix. A single vector contains the position
//! vector, that will be achieved when the respective degree of
//! freedom reaches its maximum position.
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const\n\n
//!  - GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         **MaxPosExtremaPositionVectorArray      ;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MaxPosExtremaVelocityVectorArray
//!
//! \brief
//! A pointer to an array of pointers to \c RMLDoubleVector
//! objects. The number of array elements equals the number of
//! degrees of freedom, and each of these \c RMLDOubleVector
//! objects consists of the same number of entries, such that it
//! is a square matrix. A single vector contains the velocity
//! vector, that will be achieved when the respective degree of
//! freedom reaches its maximum position.
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const\n\n
//!  - GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         **MaxPosExtremaVelocityVectorArray      ;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MaxPosExtremaAccelerationVectorArray
//!
//! \brief
//! A pointer to an array of pointers to \c RMLDoubleVector
//! objects. The number of array elements equals the number of
//! degrees of freedom, and each of these \c RMLDOubleVector
//! objects consists of the same number of entries, such that it
//! is a square matrix. A single vector contains the acceleration
//! vector, that will be achieved when the respective degree of
//! freedom reaches its maximum position.
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const\n\n
//!  - GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         **MaxPosExtremaAccelerationVectorArray  ;

};// class RMLOutputParameters



#endif


