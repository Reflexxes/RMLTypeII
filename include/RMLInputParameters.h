//  ---------------------- Doxygen info ----------------------
//! \file RMLInputParameters.h
//!
//! \brief
//! Header file for the class RMLInputParameters
//!
//! \details
//! The class RMLInputParameters constitutes the basis class for the
//! actual interface classes RMLPositionInputParameters and
//! RMLVelocityInputParameters, which are both derived from this one.
//!
//! \sa RMLOutputParameters.h
//! \sa RMLVelocityInputParameters.h
//! \sa RMLPositionInputParameters.h
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


#ifndef __RMLInputParameters__
#define __RMLInputParameters__


#include <RMLVector.h>
#include <string.h>
#include <stdio.h>


//  ---------------------- Doxygen info ----------------------
//! \class RMLInputParameters
//!
//! \brief
//! Class for the input parameters of the On-Line
//! Trajectory Generation algorithm
//!
//! \details
//! The class RMLInputParameters constitutes the basis class for the
//! actual interface classes RMLPositionInputParameters and
//! RMLVelocityInputParameters, which are both derived from this one.
//! For detailed descrpition of the input parameters, please refer to
//! \ref page_InputValues.
//!
//! \sa ReflexxesAPI
//! \sa RMLPositionInputParameters
//! \sa RMLVelocityInputParameters
//! \sa RMLOutputParameters
//! \sa \ref page_InputValues
//  ----------------------------------------------------------
class RMLInputParameters
{

protected:

//  ---------------------- Doxygen info ----------------------
//! \fn RMLInputParameters(const unsigned int DegreesOfFreedom)
//!
//! \brief
//! Constructor of class RMLInputParameters
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
//!  <li>RMLPositionInputParameters and</li>
//!  <li>RMLVelocityInputParameters,\n\n</li>
//! </ul>
//! such that the constructor is declared \c protected.
//  ----------------------------------------------------------
    RMLInputParameters(const unsigned int DegreesOfFreedom)
    {
        this->NumberOfDOFs                  =   DegreesOfFreedom                        ;
        this->MinimumSynchronizationTime    =   0.0                                     ;
        this->SelectionVector               =   new RMLBoolVector   (DegreesOfFreedom)  ;
        this->CurrentPositionVector         =   new RMLDoubleVector (DegreesOfFreedom)  ;
        this->CurrentVelocityVector         =   new RMLDoubleVector (DegreesOfFreedom)  ;
        this->CurrentAccelerationVector     =   new RMLDoubleVector (DegreesOfFreedom)  ;
        this->MaxAccelerationVector         =   new RMLDoubleVector (DegreesOfFreedom)  ;
        this->MaxJerkVector                 =   new RMLDoubleVector (DegreesOfFreedom)  ;
        this->TargetVelocityVector          =   new RMLDoubleVector (DegreesOfFreedom)  ;

        memset(this->SelectionVector->VecData               ,   0x0 ,       DegreesOfFreedom * sizeof(bool))    ;
        memset(this->CurrentPositionVector->VecData         ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
        memset(this->CurrentVelocityVector->VecData         ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
        memset(this->CurrentAccelerationVector->VecData     ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
        memset(this->MaxAccelerationVector->VecData         ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
        memset(this->MaxJerkVector->VecData                 ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
        memset(this->TargetVelocityVector->VecData          ,   0x0 ,       DegreesOfFreedom * sizeof(double))  ;
    }

public:

//  ---------------------- Doxygen info ----------------------
//! \fn RMLInputParameters(const RMLInputParameters& IP)
//!
//! \brief
//! Copy constructor of class RMLInputParameters
//!
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//!
//! \param IP
//! Object to be copied
//  ----------------------------------------------------------
    RMLInputParameters(const RMLInputParameters &IP)
    {
        this->NumberOfDOFs              =   IP.GetNumberOfDOFs()                                            ;
        this->SelectionVector           =   new RMLBoolVector   ((IP.CurrentPositionVector)->GetVecDim())   ;
        this->CurrentPositionVector     =   new RMLDoubleVector ((IP.CurrentPositionVector)->GetVecDim())   ;
        this->CurrentVelocityVector     =   new RMLDoubleVector ((IP.CurrentPositionVector)->GetVecDim())   ;
        this->CurrentAccelerationVector =   new RMLDoubleVector ((IP.CurrentPositionVector)->GetVecDim())   ;
        this->MaxAccelerationVector     =   new RMLDoubleVector ((IP.CurrentPositionVector)->GetVecDim())   ;
        this->MaxJerkVector             =   new RMLDoubleVector ((IP.CurrentPositionVector)->GetVecDim())   ;
        this->TargetVelocityVector      =   new RMLDoubleVector ((IP.CurrentPositionVector)->GetVecDim())   ;

        *this                           =   IP                                                              ;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLInputParameters(void)
//!
//! \brief
//! Destructor of class RMLInputParameters
//  ----------------------------------------------------------
    ~RMLInputParameters(void)
    {
        delete  this->SelectionVector               ;
        delete  this->CurrentPositionVector         ;
        delete  this->CurrentVelocityVector         ;
        delete  this->CurrentAccelerationVector     ;
        delete  this->MaxAccelerationVector         ;
        delete  this->MaxJerkVector                 ;
        delete  this->TargetVelocityVector          ;

        this->SelectionVector           =   NULL    ;
        this->CurrentPositionVector     =   NULL    ;
        this->CurrentVelocityVector     =   NULL    ;
        this->CurrentAccelerationVector =   NULL    ;
        this->MaxAccelerationVector     =   NULL    ;
        this->MaxJerkVector             =   NULL    ;
        this->TargetVelocityVector      =   NULL    ;
        this->NumberOfDOFs              =   0       ;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLInputParameters &operator = (const RMLInputParameters &IP)
//!
//! \brief
//! Copy operator
//!
//! \param IP
//! RMLInputParameters object to be copied
//  ----------------------------------------------------------
    RMLInputParameters &operator = (const RMLInputParameters &IP)
    {
        this->NumberOfDOFs                      =   IP.NumberOfDOFs                 ;
        this->MinimumSynchronizationTime        =   IP.MinimumSynchronizationTime   ;
        *(this->SelectionVector             )   =   *(IP.SelectionVector            );
        *(this->CurrentPositionVector       )   =   *(IP.CurrentPositionVector      );
        *(this->CurrentVelocityVector       )   =   *(IP.CurrentVelocityVector      );
        *(this->CurrentAccelerationVector   )   =   *(IP.CurrentAccelerationVector  );
        *(this->MaxAccelerationVector       )   =   *(IP.MaxAccelerationVector      );
        *(this->MaxJerkVector               )   =   *(IP.MaxJerkVector              );
        *(this->TargetVelocityVector        )   =   *(IP.TargetVelocityVector       );

        return(*this);
    }


// #############################################################################


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetSelectionVector(const RMLBoolVector &InputVector)
//!
//! \brief
//! Sets the current selection vector \f$ \vec{S}_{i} \f$ by using the
//! an RMLBoolVector object
//!
//! \param InputVector
//! The input vector, whose elements are copied to the attributes of this
//! class.
//!
//! \sa SetSelectionVector(const bool *InputVector)
//! \sa SetSelectionVectorElement(const bool &InputValue, const unsigned int &Index)
//! \sa GetSelectionVector(RMLBoolVector *InputVector) const
//  ----------------------------------------------------------
    inline void SetSelectionVector(const RMLBoolVector &InputVector)
    {
        *(this->SelectionVector) = InputVector;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetSelectionVector(const bool *InputVector)
//!
//! \brief
//! Sets the current selection vector \f$ \vec{S}_{i} \f$ by using a
//! native C++ \c bool array
//!
//! \param InputVector
//! The input vector to an array of \c bool values, whose elements are
//! copied to the attributes of this class.
//!
//! \sa SetSelectionVector(const RMLBoolVector &InputVector)
//! \sa SetSelectionVectorElement(const bool &InputValue, const unsigned int &Index)
//! \sa GetSelectionVector(bool *InputVector, const unsigned int &SizeInBytes) const
//  ----------------------------------------------------------
    inline void SetSelectionVector(const bool *InputVector)
    {
        memcpy(     (void*)this->SelectionVector->VecData
                ,   (void*)InputVector
                ,   (this->SelectionVector->GetVecDim() * sizeof(bool)) );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetSelectionVectorElement(const bool &InputValue, const unsigned int &Index)
//!
//! \brief
//! Sets one element of the current selection vector \f$ \vec{S}_{i} \f$
//!
//! \param InputValue
//! The input value that is copied to the element \c Index of the current
//! position input vector attribute of this class.
//!
//! \param Index
//! The \em index of the element to be copied
//!
//! \sa SetSelectionVector(const RMLBoolVector &InputVector)
//! \sa SetSelectionVector(const bool *InputVector)
//! \sa GetSelectionVectorElement(bool *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void SetSelectionVectorElement(      const bool          &InputValue
                                            ,   const unsigned int  &Index)
    {
        (*this->SelectionVector)[Index] =   InputValue;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetSelectionVector(RMLBoolVector *InputVector) const
//!
//! \brief
//! Copies the contents of the \c RMLBoolVector object containing the
//! current position vector \f$ \vec{S}_{i} \f$ to the \c RMLBoolVector
//! object referred to by \c InputVector
//!
//! \param InputVector
//! A pointer to an \c RMLBoolVector object, to which the data will be
//! copied
//!
//! \sa SetSelectionVector(const RMLBoolVector &InputVector)
//! \sa GetSelectionVector(bool *InputVector, const unsigned int &SizeInBytes) const
//! \sa GetSelectionVectorElement(bool *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetSelectionVector(RMLBoolVector *InputVector) const
    {
        *InputVector    =   *(this->SelectionVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetSelectionVector(bool *InputVector, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the array of \c bool values representing the current
//! position vector \f$ \vec{S}_{i} \f$ to the memory pointed to by
//! \c InputVector
//!
//! \param InputVector
//! A pointer to an array of \c bool values, to which the data will be
//! copied
//!
//! \param SizeInBytes
//! The size of available memory at the location pointed to by
//! \c InputVector. To assure safety and to prevent from prohibited writing
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c bool value.
//!
//! \sa SetSelectionVector(const bool *InputVector)
//! \sa GetSelectionVector(RMLBoolVector *InputVector) const
//! \sa GetSelectionVectorElement(bool *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetSelectionVector(     bool                    *InputVector
                                    ,   const unsigned int      &SizeInBytes) const
    {
        memcpy(     (void*)InputVector
                ,   (void*)this->SelectionVector->VecData
                ,   SizeInBytes );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetSelectionVectorElement(bool *InputValue, const unsigned int &Index) const
//!
//! \brief
//! Copies one element of the current selection vector
//! \f$ \vec{S}_{i} \f$ to the memory pointed to by \c InputValue
//!
//! \param InputValue
//! A pointer to one \c bool value, to which the desired vector element
//! will be copied
//!
//! \param Index
//! Specifies the desired element of the vector. The element numbering
//! starts with \em 0 (zero). If this value is greater the number
//! of vector elements, a value of \em 0.0 will be written to the memory
//! pointed to by \c InputValue.
//!
//! \sa SetSelectionVectorElement(const bool &InputValue, const unsigned int &Index)
//! \sa GetSelectionVector(RMLBoolVector *InputVector) const
//! \sa GetSelectionVectorElement(bool *InputValue, const unsigned int &Index) const
//! \sa GetSelectionVectorElement(const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetSelectionVectorElement(      bool                *InputValue
                                            ,   const unsigned int  &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->SelectionVector->GetVecDim() ) )
        {
            *InputValue =   0.0;
        }
        else
        {
            *InputValue =   (*this->SelectionVector)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline bool GetSelectionVectorElement(const unsigned int &Index) const
//!
//! \brief
//! Returns one single element of the current selection vector
//! \f$ \vec{S}_{i} \f$
//!
//! \param Index
//! Specifies the desired element of the vector. The index of the first
//! vector element is \em 0 (zero). If the value of \c Index value is
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c InputValue.
//!
//! \sa SetSelectionVectorElement(const bool &InputValue, const unsigned int &Index)
//! \sa GetSelectionVectorElement(bool *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline bool GetSelectionVectorElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->SelectionVector->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {
            return( (*this->SelectionVector)[Index] );
        }
    }


// #############################################################################


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCurrentPositionVector(const RMLDoubleVector &InputVector)
//!
//! \brief
//! Sets the current selection vector \f$ \vec{P}_{i} \f$ by using the
//! an \c RMLDoubleVector object
//!
//! \param InputVector
//! The input vector, whose elements are copied to the attributes of this
//! class.
//!
//! \sa SetCurrentPositionVector(const double *InputVector)
//! \sa SetCurrentPositionVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetCurrentPositionVector(RMLDoubleVector *InputVector) const
//  ----------------------------------------------------------
    inline void SetCurrentPositionVector(const RMLDoubleVector &InputVector)
    {
        *(this->CurrentPositionVector) = InputVector;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCurrentPositionVector(const double *InputVector)
//!
//! \brief
//! Sets the current selection vector \f$ \vec{P}_{i} \f$ by using a
//! native C \c double array
//!
//! \param InputVector
//! The input vector to an array of \c double values, whose elements are
//! copied to the attributes of this class.
//!
//! \sa SetCurrentPositionVector(const RMLDoubleVector &InputVector)
//! \sa SetCurrentPositionVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetCurrentPositionVector(double *InputVector, const unsigned int &SizeInBytes) const
//  ----------------------------------------------------------
    inline void SetCurrentPositionVector(const double *InputVector)
    {
        memcpy(     (void*)this->CurrentPositionVector->VecData
                ,   (void*)InputVector
                ,   (this->CurrentPositionVector->GetVecDim() * sizeof(double)) );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCurrentPositionVectorElement(const double &InputValue, const unsigned int &Index)
//!
//! \brief
//! Sets one element of the current selection vector \f$ \vec{P}_{i} \f$
//!
//! \param InputValue
//! The input value that is copied to the element \c Index of the current
//! position input vector attribute of this class.
//!
//! \param Index
//! The \em index of the element to be copied
//!
//! \sa SetCurrentPositionVector(const RMLDoubleVector &InputVector)
//! \sa SetCurrentPositionVector(const double *InputVector)
//! \sa GetCurrentPositionVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void SetCurrentPositionVectorElement(        const double        &InputValue
                                                    ,   const unsigned int  &Index)
    {
        (*this->CurrentPositionVector)[Index]   =   InputValue;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetCurrentPositionVector(RMLDoubleVector *InputVector) const
//!
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! current position vector \f$ \vec{P}_{i} \f$ to the \c RMLDoubleVector
//! object referred to by \c InputVector
//!
//! \param InputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be
//! copied
//!
//! \sa SetCurrentPositionVector(const RMLDoubleVector &InputVector)
//! \sa GetCurrentPositionVector(double *InputVector, const unsigned int &SizeInBytes) const
//! \sa GetCurrentPositionVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetCurrentPositionVector(RMLDoubleVector *InputVector) const
    {
        *InputVector    =   *(this->CurrentPositionVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetCurrentPositionVector(double *InputVector, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the array of \c double values representing the current
//! position vector \f$ \vec{P}_{i} \f$ to the memory pointed to by
//! \c InputVector
//!
//! \param InputVector
//! A pointer to an array of \c double values, to which the data will be
//! copied
//!
//! \param SizeInBytes
//! The size of available memory at the location pointed to by
//! \c InputVector. To assure safety and to prevent from prohibited writing
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//!
//! \sa SetCurrentPositionVector(const double *InputVector)
//! \sa GetCurrentPositionVector(RMLDoubleVector *InputVector) const
//! \sa GetCurrentPositionVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetCurrentPositionVector(       double              *InputVector
                                            ,   const unsigned int  &SizeInBytes) const
    {
        memcpy(     (void*)InputVector
                ,   (void*)this->CurrentPositionVector->VecData
                ,   SizeInBytes );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetCurrentPositionVectorElement(double *InputValue, const unsigned int &Index) const
//!
//! \brief
//! Copies one element of the current selection vector
//! \f$ \vec{P}_{i} \f$ to the memory pointed to by \c InputValue
//!
//! \param InputValue
//! A pointer to one \c double value, to which the desired vector element
//! will be copied
//!
//! \param Index
//! Specifies the desired element of the vector. The element numbering
//! starts with \em 0 (zero). If this value is greater the number
//! of vector elements, a value of \em 0.0 will be written to the memory
//! pointed to by \c InputValue.
//!
//! \sa SetCurrentPositionVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetCurrentPositionVector(RMLDoubleVector *InputVector) const
//! \sa GetCurrentPositionVectorElement(double *InputValue, const unsigned int &Index) const
//! \sa GetCurrentPositionVectorElement(const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetCurrentPositionVectorElement(    double              *InputValue
                                                ,   const unsigned int  &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->CurrentPositionVector->GetVecDim() ) )
        {
            *InputValue =   0.0;
        }
        else
        {
            *InputValue =   (*this->CurrentPositionVector)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetCurrentPositionVectorElement(const unsigned int &Index) const
//!
//! \brief
//! Returns one single element of the current selection vector
//! \f$ \vec{P}_{i} \f$
//!
//! \param Index
//! Specifies the desired element of the vector. The index of the first
//! vector element is \em 0 (zero). If the value of \c Index value is
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c InputValue.
//!
//! \sa SetCurrentPositionVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetCurrentPositionVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline double GetCurrentPositionVectorElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->CurrentPositionVector->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {
            return( (*this->CurrentPositionVector)[Index] );
        }
    }


// #############################################################################


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCurrentVelocityVector(const RMLDoubleVector &InputVector)
//!
//! \brief
//! Sets the current velocity vector \f$ \vec{V}_{i} \f$ by using the
//! an \c RMLDoubleVector object
//!
//! \param InputVector
//! The input vector, whose elements are copied to the attributes of this
//! class.
//!
//! \sa SetCurrentVelocityVector(const double *InputVector)
//! \sa SetCurrentVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetCurrentVelocityVector(RMLDoubleVector *InputVector) const
//  ----------------------------------------------------------
    inline void SetCurrentVelocityVector(const RMLDoubleVector &InputVector)
    {
        *(this->CurrentVelocityVector) = InputVector;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCurrentVelocityVector(const double *InputVector)
//!
//! \brief
//! Sets the current velocity vector \f$ \vec{V}_{i} \f$ by using a
//! native C \c double array
//!
//! \param InputVector
//! The input vector to an array of \c double values, whose elements are
//! copied to the attributes of this class.
//!
//! \sa SetCurrentVelocityVector(const RMLDoubleVector &InputVector)
//! \sa SetCurrentVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetCurrentVelocityVector(double *InputVector, const unsigned int &SizeInBytes) const
//  ----------------------------------------------------------
    inline void SetCurrentVelocityVector(const double *InputVector)
    {
        memcpy(     (void*)this->CurrentVelocityVector->VecData
                ,   (void*)InputVector
                ,   (this->CurrentVelocityVector->GetVecDim() * sizeof(double)) );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCurrentVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//!
//! \brief
//! Sets one element of the current velocity vector \f$ \vec{V}_{i} \f$
//!
//! \param InputValue
//! The input value that is copied to the element \c Index of the current
//! velocity input vector attribute of this class.
//!
//! \param Index
//! The \em index of the element to be copied
//!
//! \sa SetCurrentVelocityVector(const RMLDoubleVector &InputVector)
//! \sa SetCurrentVelocityVector(const double *InputVector)
//! \sa GetCurrentVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void SetCurrentVelocityVectorElement(    const double        &InputValue
                                                ,   const unsigned int  &Index)
    {
        (*this->CurrentVelocityVector)[Index]   =   InputValue;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetCurrentVelocityVector(RMLDoubleVector *InputVector) const
//!
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! current velocity vector \f$ \vec{V}_{i} \f$ to the \c RMLDoubleVector
//! object referred to by \c InputVector
//!
//! \param InputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be
//! copied
//!
//! \sa SetCurrentVelocityVector(const RMLDoubleVector &InputVector)
//! \sa GetCurrentVelocityVector(double *InputVector, const unsigned int &SizeInBytes) const
//! \sa GetCurrentVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetCurrentVelocityVector(RMLDoubleVector *InputVector) const
    {
        *InputVector    =   *(this->CurrentVelocityVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetCurrentVelocityVector(double *InputVector, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the array of \c double values representing the current
//! velocity vector \f$ \vec{V}_{i} \f$ to the memory pointed to by
//! \c InputVector
//!
//! \param InputVector
//! A pointer to an array of \c double values, to which the data will be
//! copied
//!
//! \param SizeInBytes
//! The size of available memory at the location pointed to by
//! \c InputVector. To assure safety and to prevent from prohibited writing
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//!
//! \sa SetCurrentVelocityVector(const double *InputVector)
//! \sa GetCurrentVelocityVector(RMLDoubleVector *InputVector) const
//! \sa GetCurrentVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetCurrentVelocityVector(       double              *InputVector
                                            ,   const unsigned int  &SizeInBytes) const
    {
        memcpy(     (void*)InputVector
                ,   (void*)this->CurrentVelocityVector->VecData
                ,   SizeInBytes );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetCurrentVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//!
//! \brief
//! Copies one element of the current velocity vector
//! \f$ \vec{V}_{i} \f$ to the memory pointed to by \c InputValue
//!
//! \param InputValue
//! A pointer to one \c double value, to which the desired vector element
//! will be copied
//!
//! \param Index
//! Specifies the desired element of the vector. The element numbering
//! starts with \em 0 (zero). If this value is greater the number
//! of vector elements, a value of \em 0.0 will be written to the memory
//! pointed to by \c InputValue.
//!
//! \sa SetCurrentVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetCurrentVelocityVector(RMLDoubleVector *InputVector) const
//! \sa GetCurrentVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//! \sa GetCurrentVelocityVectorElement(const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetCurrentVelocityVectorElement(    double              *InputValue
                                                ,   const unsigned int  &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->CurrentVelocityVector->GetVecDim() ) )
        {
            *InputValue =   0.0;
        }
        else
        {
            *InputValue =   (*this->CurrentVelocityVector)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetCurrentVelocityVectorElement(const unsigned int &Index) const
//!
//! \brief
//! Returns one single element of the current velocity vector
//! \f$ \vec{V}_{i} \f$
//!
//! \param Index
//! Specifies the desired element of the vector. The index of the first
//! vector element is \em 0 (zero). If the value of \c Index value is
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c InputValue.
//!
//! \sa SetCurrentVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetCurrentVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline double GetCurrentVelocityVectorElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->CurrentVelocityVector->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {
            return( (*this->CurrentVelocityVector)[Index] );
        }
    }


// #############################################################################


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCurrentAccelerationVector(const RMLDoubleVector &InputVector)
//!
//! \brief
//! Sets the current acceleration vector \f$ \vec{A}_{i} \f$ by using the
//! an \c RMLDoubleVector object
//!
//! \param InputVector
//! The input vector, whose elements are copied to the attributes of this
//! class.
//!
//! \sa SetCurrentAccelerationVector(const double *InputVector)
//! \sa SetCurrentAccelerationVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetCurrentAccelerationVector(RMLDoubleVector *InputVector) const
//  ----------------------------------------------------------
    inline void SetCurrentAccelerationVector(const RMLDoubleVector &InputVector)
    {
        *(this->CurrentAccelerationVector) = InputVector;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCurrentAccelerationVector(const double *InputVector)
//!
//! \brief
//! Sets the current acceleration vector \f$ \vec{A}_{i} \f$ by using a
//! native C \c double array
//!
//! \param InputVector
//! The input vector to an array of \c double values, whose elements are
//! copied to the attributes of this class.
//!
//! \sa SetCurrentAccelerationVector(const RMLDoubleVector &InputVector)
//! \sa SetCurrentAccelerationVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetCurrentAccelerationVector(double *InputVector, const unsigned int &SizeInBytes) const
//  ----------------------------------------------------------
    inline void SetCurrentAccelerationVector(const double *InputVector)
    {
        memcpy(     (void*)this->CurrentAccelerationVector->VecData
                ,   (void*)InputVector
                ,   (this->CurrentAccelerationVector->GetVecDim() * sizeof(double)) );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCurrentAccelerationVectorElement(const double &InputValue, const unsigned int &Index)
//!
//! \brief
//! Sets one element of the current acceleration vector \f$ \vec{A}_{i} \f$
//!
//! \param InputValue
//! The input value that is copied to the element \c Index of the current
//! acceleration input vector attribute of this class.
//!
//! \param Index
//! The \em index of the element to be copied
//!
//! \sa SetCurrentAccelerationVector(const RMLDoubleVector &InputVector)
//! \sa SetCurrentAccelerationVector(const double *InputVector)
//! \sa GetCurrentAccelerationVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void SetCurrentAccelerationVectorElement(    const double        &InputValue
                                                    ,   const unsigned int  &Index)
    {
        (*this->CurrentAccelerationVector)[Index]   =   InputValue;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetCurrentAccelerationVector(RMLDoubleVector *InputVector) const
//!
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! current acceleration vector \f$ \vec{A}_{i} \f$ to the
//! \c RMLDoubleVector object referred to by \c InputVector
//!
//! \param InputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be
//! copied
//!
//! \sa SetCurrentAccelerationVector(const RMLDoubleVector &InputVector)
//! \sa GetCurrentAccelerationVector(double *InputVector, const unsigned int &SizeInBytes) const
//! \sa GetCurrentAccelerationVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetCurrentAccelerationVector(RMLDoubleVector *InputVector) const
    {
        *InputVector    =   *(this->CurrentAccelerationVector);
    }



//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetCurrentAccelerationVector(double *InputVector, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the array of \c double values representing the current
//! acceleration vector \f$ \vec{A}_{i} \f$ to the memory pointed to by
//! \c InputVector
//!
//! \param InputVector
//! A pointer to an array of \c double values, to which the data will be
//! copied
//!
//! \param SizeInBytes
//! The size of available memory at the location pointed to by
//! \c InputVector. To assure safety and to prevent from prohibited writing
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//!
//! \sa SetCurrentAccelerationVector(const double *InputVector)
//! \sa GetCurrentAccelerationVector(RMLDoubleVector *InputVector) const
//! \sa GetCurrentAccelerationVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetCurrentAccelerationVector(       double              *InputVector
                                                ,   const unsigned int  &SizeInBytes) const
    {
        memcpy(     (void*)InputVector
                ,   (void*)this->CurrentAccelerationVector->VecData
                ,   SizeInBytes );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetCurrentAccelerationVectorElement(double *InputValue, const unsigned int &Index) const
//!
//! \brief
//! Copies one element of the current acceleration vector
//! \f$ \vec{A}_{i} \f$ to the memory pointed to by \c InputValue
//!
//! \param InputValue
//! A pointer to one \c double value, to which the desired vector element
//! will be copied
//!
//! \param Index
//! Specifies the desired element of the vector. The element numbering
//! starts with \em 0 (zero). If this value is greater the number
//! of vector elements, a value of \em 0.0 will be written to the memory
//! pointed to by \c InputValue.
//!
//! \sa SetCurrentAccelerationVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetCurrentAccelerationVector(RMLDoubleVector *InputVector) const
//! \sa GetCurrentAccelerationVectorElement(double *InputValue, const unsigned int &Index) const
//! \sa GetCurrentAccelerationVectorElement(const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetCurrentAccelerationVectorElement(    double              *InputValue
                                                    ,   const unsigned int  &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->CurrentAccelerationVector->GetVecDim() ) )
        {
            *InputValue =   0.0;
        }
        else
        {
            *InputValue =   (*this->CurrentAccelerationVector)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetCurrentAccelerationVectorElement(const unsigned int &Index) const
//!
//! \brief
//! Returns one single element of the current acceleration vector
//! \f$ \vec{A}_{i} \f$
//!
//! \param Index
//! Specifies the desired element of the vector. The index of the first
//! vector element is \em 0 (zero). If the value of \c Index value is
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c InputValue.
//!
//! \sa SetCurrentAccelerationVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetCurrentAccelerationVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline double GetCurrentAccelerationVectorElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->CurrentAccelerationVector->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {
            return( (*this->CurrentAccelerationVector)[Index] );
        }
    }


// #############################################################################


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetMaxAccelerationVector(const RMLDoubleVector &InputVector)
//!
//! \brief
//! Sets the maximum acceleration vector \f$ \vec{A}_{i}^{\,max} \f$ by using the
//! an \c RMLDoubleVector object
//!
//! \param InputVector
//! The input vector, whose elements are copied to the attributes of this
//! class.
//!
//! \sa SetMaxAccelerationVector(const double *InputVector)
//! \sa SetMaxAccelerationVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetMaxAccelerationVector(RMLDoubleVector *InputVector) const
//  ----------------------------------------------------------
    inline void SetMaxAccelerationVector(const RMLDoubleVector &InputVector)
    {
        *(this->MaxAccelerationVector) = InputVector;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetMaxAccelerationVector(const double *InputVector)
//!
//! \brief
//! Sets the maximum acceleration vector \f$ \vec{A}_{i}^{\,max} \f$ by using a
//! native C \c double array
//!
//! \param InputVector
//! The input vector to an array of \c double values, whose elements are
//! copied to the attributes of this class.
//!
//! \sa SetMaxAccelerationVector(const RMLDoubleVector &InputVector)
//! \sa SetMaxAccelerationVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetMaxAccelerationVector(double *InputVector, const unsigned int &SizeInBytes) const
//  ----------------------------------------------------------
    inline void SetMaxAccelerationVector(const double *InputVector)
    {
        memcpy(     (void*)this->MaxAccelerationVector->VecData
                ,   (void*)InputVector
                ,   (this->MaxAccelerationVector->GetVecDim() * sizeof(double)) );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetMaxAccelerationVectorElement(const double &InputValue, const unsigned int &Index)
//!
//! \brief
//! Sets one element of the maximum acceleration vector \f$ \vec{A}_{i}^{\,max} \f$
//!
//! \param InputValue
//! The input value that is copied to the element \c Index of the
//! maximum acceleration input vector attribute of this class.
//!
//! \param Index
//! The \em index of the element to be copied
//!
//! \sa SetMaxAccelerationVector(const RMLDoubleVector &InputVector)
//! \sa SetMaxAccelerationVector(const double *InputVector)
//! \sa GetMaxAccelerationVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void SetMaxAccelerationVectorElement(    const double        &InputValue
                                                ,   const unsigned int  &Index)
    {
        (*this->MaxAccelerationVector)[Index]   =   InputValue;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetMaxAccelerationVector(RMLDoubleVector *InputVector) const
//!
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! maximum acceleration vector \f$ \vec{A}_{i}^{\,max} \f$ to the
//! \c RMLDoubleVector object referred to by \c InputVector
//!
//! \param InputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be
//! copied
//!
//! \sa SetMaxAccelerationVector(const RMLDoubleVector &InputVector)
//! \sa GetMaxAccelerationVector(double *InputVector, const unsigned int &SizeInBytes) const
//! \sa GetMaxAccelerationVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetMaxAccelerationVector(RMLDoubleVector *InputVector) const
    {
        *InputVector    =   *(this->MaxAccelerationVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetMaxAccelerationVector(double *InputVector, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the array of \c double values representing the maximum
//! acceleration vector \f$ \vec{A}_{i}^{\,max} \f$ to the memory pointed
//! to by \c InputVector
//!
//! \param InputVector
//! A pointer to an array of \c double values, to which the data will be
//! copied
//!
//! \param SizeInBytes
//! The size of available memory at the location pointed to by
//! \c InputVector. To assure safety and to prevent from prohibited writing
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//!
//! \sa SetMaxAccelerationVector(const double *InputVector)
//! \sa GetMaxAccelerationVector(RMLDoubleVector *InputVector) const
//! \sa GetMaxAccelerationVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetMaxAccelerationVector(       double              *InputVector
                                            ,   const unsigned int  &SizeInBytes) const
    {
        memcpy(     (void*)InputVector
                ,   (void*)this->MaxAccelerationVector->VecData
                ,   SizeInBytes );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetMaxAccelerationVectorElement(double *InputValue, const unsigned int &Index) const
//!
//! \brief
//! Copies one element of the maximum acceleration vector
//! \f$ \vec{A}_{i}^{\,max} \f$ to the memory pointed to by \c InputValue
//!
//! \param InputValue
//! A pointer to one \c double value, to which the desired vector element
//! will be copied
//!
//! \param Index
//! Specifies the desired element of the vector. The element numbering
//! starts with \em 0 (zero). If this value is greater the number
//! of vector elements, a value of \em 0.0 will be written to the memory
//! pointed to by \c InputValue.
//!
//! \sa SetMaxAccelerationVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetMaxAccelerationVector(RMLDoubleVector *InputVector) const
//! \sa GetMaxAccelerationVectorElement(double *InputValue, const unsigned int &Index) const
//! \sa GetMaxAccelerationVectorElement(const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetMaxAccelerationVectorElement(    double              *InputValue
                                                ,   const unsigned int  &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->MaxAccelerationVector->GetVecDim() ) )
        {
            *InputValue =   0.0;
        }
        else
        {
            *InputValue =   (*this->MaxAccelerationVector)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetMaxAccelerationVectorElement(const unsigned int &Index) const
//!
//! \brief
//! Returns one single element of the maximum acceleration vector
//! \f$ \vec{A}_{i}^{\,max} \f$
//!
//! \param Index
//! Specifies the desired element of the vector. The index of the first
//! vector element is \em 0 (zero). If the value of \c Index value is
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c InputValue.
//!
//! \sa SetMaxAccelerationVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetMaxAccelerationVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline double GetMaxAccelerationVectorElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->MaxAccelerationVector->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {
            return( (*this->MaxAccelerationVector)[Index] );
        }
    }


// #############################################################################


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetMaxJerkVector(const RMLDoubleVector &InputVector)
//!
//! \brief
//! Sets the maximum jerk vector \f$ \vec{J}_{i}^{\,max} \f$ by using the
//! an \c RMLDoubleVector object
//!
//! \param InputVector
//! The input vector, whose elements are copied to the attributes of this
//! class.
//!
//! \sa SetMaxJerkVector(const double *InputVector)
//! \sa SetMaxJerkVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetMaxJerkVector(RMLDoubleVector *InputVector) const
//  ----------------------------------------------------------
    inline void SetMaxJerkVector(const RMLDoubleVector &InputVector)
    {
        *(this->MaxJerkVector) = InputVector;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetMaxJerkVector(const double *InputVector)
//!
//! \brief
//! Sets the maximum jerk vector \f$ \vec{J}_{i}^{\,max} \f$ by using a
//! native C \c double array
//!
//! \param InputVector
//! The input vector to an array of \c double values, whose elements are
//! copied to the attributes of this class.
//!
//! \sa SetMaxJerkVector(const RMLDoubleVector &InputVector)
//! \sa SetMaxJerkVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetMaxJerkVector(double *InputVector, const unsigned int &SizeInBytes) const
//  ----------------------------------------------------------
    inline void SetMaxJerkVector(const double *InputVector)
    {
        memcpy(     (void*)this->MaxJerkVector->VecData
                ,   (void*)InputVector
                ,   (this->MaxJerkVector->GetVecDim() * sizeof(double)) );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetMaxJerkVectorElement(const double &InputValue, const unsigned int &Index)
//!
//! \brief
//! Sets one element of the maximum jerk vector \f$ \vec{J}_{i}^{\,max} \f$
//!
//! \param InputValue
//! The input value that is copied to the element \c Index of the
//! maximum jerk input vector attribute of this class.
//!
//! \param Index
//! The \em index of the element to be copied
//!
//! \sa SetMaxJerkVector(const RMLDoubleVector &InputVector)
//! \sa SetMaxJerkVector(const double *InputVector)
//! \sa GetMaxJerkVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void SetMaxJerkVectorElement(    const double        &InputValue
                                        ,   const unsigned int  &Index)
    {
        (*this->MaxJerkVector)[Index]   =   InputValue;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetMaxJerkVector(RMLDoubleVector *InputVector) const
//!
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! maximum jerk vector \f$ \vec{J}_{i}^{\,max} \f$ to the \c RMLDoubleVector
//! object referred to by \c InputVector
//!
//! \param InputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be
//! copied
//!
//! \sa SetMaxJerkVector(const RMLDoubleVector &InputVector)
//! \sa GetMaxJerkVector(double *InputVector, const unsigned int &SizeInBytes) const
//! \sa GetMaxJerkVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetMaxJerkVector(RMLDoubleVector *InputVector) const
    {
        *InputVector    =   *(this->MaxJerkVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetMaxJerkVector(double *InputVector, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the array of \c double values representing the maximum
//! jerk vector \f$ \vec{J}_{i}^{\,max} \f$ to the memory pointed to by
//! \c InputVector
//!
//! \param InputVector
//! A pointer to an array of \c double values, to which the data will be
//! copied
//!
//! \param SizeInBytes
//! The size of available memory at the location pointed to by
//! \c InputVector. To assure safety and to prevent from prohibited writing
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//!
//! \sa SetMaxJerkVector(const double *InputVector)
//! \sa GetMaxJerkVector(RMLDoubleVector *InputVector) const
//! \sa GetMaxJerkVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetMaxJerkVector(       double              *InputVector
                                    ,   const unsigned int  &SizeInBytes) const
    {
        memcpy(     (void*)InputVector
                ,   (void*)this->MaxJerkVector->VecData
                ,   SizeInBytes );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetMaxJerkVectorElement(double *InputValue, const unsigned int &Index) const
//!
//! \brief
//! Copies one element of the maximum jerk vector
//! \f$ \vec{J}_{i}^{\,max} \f$ to the memory pointed to by \c InputValue
//!
//! \param InputValue
//! A pointer to one \c double value, to which the desired vector element
//! will be copied
//!
//! \param Index
//! Specifies the desired element of the vector. The element numbering
//! starts with \em 0 (zero). If this value is greater the number
//! of vector elements, a value of \em 0.0 will be written to the memory
//! pointed to by \c InputValue.
//!
//! \sa SetMaxJerkVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetMaxJerkVector(RMLDoubleVector *InputVector) const
//! \sa GetMaxJerkVectorElement(double *InputValue, const unsigned int &Index) const
//! \sa GetMaxJerkVectorElement(const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetMaxJerkVectorElement(    double              *InputValue
                                        ,   const unsigned int  &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->MaxJerkVector->GetVecDim() ) )
        {
            *InputValue =   0.0;
        }
        else
        {
            *InputValue =   (*this->MaxJerkVector)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetMaxJerkVectorElement(const unsigned int &Index) const
//!
//! \brief
//! Returns one single element of the maximum jerk vector
//! \f$ \vec{J}_{i}^{\,max} \f$
//!
//! \param Index
//! Specifies the desired element of the vector. The index of the first
//! vector element is \em 0 (zero). If the value of \c Index value is
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c InputValue.
//!
//! \sa SetMaxJerkVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetMaxJerkVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline double GetMaxJerkVectorElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->MaxJerkVector->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {
            return( (*this->MaxJerkVector)[Index] );
        }
    }


// #############################################################################


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetTargetVelocityVector(const RMLDoubleVector &InputVector)
//!
//! \brief
//! Sets the target velocity vector \f$ \vec{V}_{i}^{\,trgt} \f$ by using the
//! an \c RMLDoubleVector object
//!
//! \param InputVector
//! The input vector, whose elements are copied to the attributes of this
//! class.
//!
//! \sa SetTargetVelocityVector(const double *InputVector)
//! \sa SetTargetVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetTargetVelocityVector(RMLDoubleVector *InputVector) const
//  ----------------------------------------------------------
    inline void SetTargetVelocityVector(const RMLDoubleVector &InputVector)
    {
        *(this->TargetVelocityVector) = InputVector;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetTargetVelocityVector(const double *InputVector)
//!
//! \brief
//! Sets the target velocity vector \f$ \vec{V}_{i}^{\,trgt} \f$ by using a
//! native C \c double array
//!
//! \param InputVector
//! The input vector to an array of \c double values, whose elements are
//! copied to the attributes of this class.
//!
//! \sa SetTargetVelocityVector(const RMLDoubleVector &InputVector)
//! \sa SetTargetVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetTargetVelocityVector(double *InputVector, const unsigned int &SizeInBytes) const
//  ----------------------------------------------------------
    inline void SetTargetVelocityVector(const double *InputVector)
    {
        memcpy(     (void*)this->TargetVelocityVector->VecData
                ,   (void*)InputVector
                ,   (this->TargetVelocityVector->GetVecDim() * sizeof(double))  );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetTargetVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//!
//! \brief
//! Sets one element of the target velocity vector \f$ \vec{V}_{i}^{\,trgt} \f$
//!
//! \param InputValue
//! The input value that is copied to the element \c Index of the target
//! velocity input vector attribute of this class.
//!
//! \param Index
//! The \em index of the element to be copied
//!
//! \sa SetTargetVelocityVector(const RMLDoubleVector &InputVector)
//! \sa SetTargetVelocityVector(const double *InputVector)
//! \sa GetTargetVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void SetTargetVelocityVectorElement(     const double        &InputValue
                                                ,   const unsigned int  &Index)
    {
        (*this->TargetVelocityVector)[Index]    =   InputValue;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetTargetVelocityVector(RMLDoubleVector *InputVector) const
//!
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! target velocity vector \f$ \vec{V}_{i}^{\,trgt} \f$ to the \c RMLDoubleVector
//! object referred to by \c InputVector
//!
//! \param InputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be
//! copied
//!
//! \sa SetTargetVelocityVector(const RMLDoubleVector &InputVector)
//! \sa GetTargetVelocityVector(double *InputVector, const unsigned int &SizeInBytes) const
//! \sa GetTargetVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetTargetVelocityVector(RMLDoubleVector *InputVector) const
    {
        *InputVector    =   *(this->TargetVelocityVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetTargetVelocityVector(double *InputVector, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the array of \c double values representing the target
//! velocity vector \f$ \vec{V}_{i}^{\,trgt} \f$ to the memory pointed to by
//! \c InputVector
//!
//! \param InputVector
//! A pointer to an array of \c double values, to which the data will be
//! copied
//!
//! \param SizeInBytes
//! The size of available memory at the location pointed to by
//! \c InputVector. To assure safety and to prevent from prohibited writing
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//!
//! \sa SetTargetVelocityVector(const double *InputVector)
//! \sa GetTargetVelocityVector(RMLDoubleVector *InputVector) const
//! \sa GetTargetVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetTargetVelocityVector(    double              *InputVector
                                        ,   const unsigned int  &SizeInBytes) const
    {
        memcpy(     (void*)InputVector
                ,   (void*)this->TargetVelocityVector->VecData
                ,   SizeInBytes );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetTargetVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//!
//! \brief
//! Copies one element of the target velocity vector
//! \f$ \vec{V}_{i}^{\,trgt} \f$ to the memory pointed to by \c InputValue
//!
//! \param InputValue
//! A pointer to one \c double value, to which the desired vector element
//! will be copied
//!
//! \param Index
//! Specifies the desired element of the vector. The element numbering
//! starts with \em 0 (zero). If this value is greater the number
//! of vector elements, a value of \em 0.0 will be written to the memory
//! pointed to by \c InputValue.
//!
//! \sa SetTargetVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetTargetVelocityVector(RMLDoubleVector *InputVector) const
//! \sa GetTargetVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//! \sa GetTargetVelocityVectorElement(const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetTargetVelocityVectorElement(     double              *InputValue
                                                ,   const unsigned int  &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->TargetVelocityVector->GetVecDim() ) )
        {
            *InputValue =   0.0;
        }
        else
        {
            *InputValue =   (*this->TargetVelocityVector)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetTargetVelocityVectorElement(const unsigned int &Index) const
//!
//! \brief
//! Returns one single element of the target velocity vector
//! \f$ \vec{V}_{i}^{\,trgt} \f$
//!
//! \param Index
//! Specifies the desired element of the vector. The index of the first
//! vector element is \em 0 (zero). If the value of \c Index value is
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c InputValue.
//!
//! \sa SetTargetVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetTargetVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline double GetTargetVelocityVectorElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->TargetVelocityVector->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {
            return( (*this->TargetVelocityVector)[Index] );
        }
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
//! \fn inline double GetMinimumSynchronizationTime(void) const
//!
//! \brief
//! Returns the \b optional parameter for the desired execution time.
//!
//! \return
//! Desired execution time in seconds.
//!
//! \sa MinimumSynchronizationTime
//  ----------------------------------------------------------
    inline double GetMinimumSynchronizationTime(void) const
    {
        return(this->MinimumSynchronizationTime);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetMinimumSynchronizationTime(double Time)
//!
//! \brief
//! Sets the \b optional parameter for the desired execution time.
//!
//! \sa MinimumSynchronizationTime
//  ----------------------------------------------------------
    inline void SetMinimumSynchronizationTime(double Time)
    {
        this->MinimumSynchronizationTime    =   Time;
        return;
    }


protected:


//  ---------------------- Doxygen info ----------------------
//! \fn void Echo(FILE* FileHandler = stdout) const
//!
//! \brief
//! Prints the complete set of input parameters to *FileHandler
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

        fprintf(FileHandler,   "Selection vector           : ");
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            fprintf(FileHandler, " %s ", (this->SelectionVector->VecData[i])?("true"):("false"));
        }
        fprintf(FileHandler, "\nCurrent position vector    : ");
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            fprintf(FileHandler, " %.20le ", this->CurrentPositionVector->VecData[i]);
        }
        fprintf(FileHandler, "\nCurrent velocity vector    : ");
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            fprintf(FileHandler, " %.20le ", this->CurrentVelocityVector->VecData[i]);
        }
        fprintf(FileHandler, "\nCurrent acceleration vector: ");
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            fprintf(FileHandler, " %.20le ", this->CurrentAccelerationVector->VecData[i]);
        }
        fprintf(FileHandler, "\nTarget velocity vector     : ");
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            fprintf(FileHandler, " %.20le ", this->TargetVelocityVector->VecData[i]);
        }
        fprintf(FileHandler, "\nMax. acceleration vector   : ");
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            fprintf(FileHandler, " %.20le ", this->MaxAccelerationVector->VecData[i]);
        }
        fprintf(FileHandler, "\nMax. jerk vector           : ");
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            fprintf(FileHandler, " %.20le ", this->MaxJerkVector->VecData[i]);
        }
        fprintf(FileHandler, "\n");
        return;
    }

public:

//  ---------------------- Doxygen info ----------------------
//! \var unsigned int NumberOfDOFs
//!
//! \brief
//! The number of degrees of freedom \f$ K \f$
//!
//! \sa RMLInputParameters::RMLInputParameters()
//  ----------------------------------------------------------
    unsigned int            NumberOfDOFs;


//  ---------------------- Doxygen info ----------------------
//! \var double MinimumSynchronizationTime
//!
//! \brief
//! Minimum trajectory execution time in seconds specified by the user (\b optional input parameter)
//!
//! \details
//! This input parameter is \b optional and allows users to specify a
//! desired minimum execution time for time- and phase-synchronized
//! trajectories that will be computed by calling
//! ReflexxesAPI::RMLPosition() or ReflexxesAPI::RMLVelocity().\n\n
//!
//! The following four cases may happen if this input parameter is used for
//! the <b>position-based algorithm</b> (TypeIIRMLPosition):
//!
//!  - If the flag RMLFlags::NO_SYNCHRONIZATION is used, the value of
//!    \c MinimumSynchronizationTime will be ignored.
//!  - If the value of \c DesiredExecutionTime is less then the minimum
//!    possible synchronization time, it will be ignored and the
//!    minimum possible synchronization time will be used.
//!  - If the value of \c DesiredExecutionTime is inside of an inoperative
//!    time interval of one or more degrees of freedom, the end of
//!    the time interval that will provide the time closest to the
//!    desired execution time will be used. This time is greater than
//!    the value of \c MinimumSynchronizationTime.
//!  - Otherwise, a trajectory will be computed whose execution time
//!    is \c MinimumSynchronizationTime.
//!
//! The following three cases may happen if this input parameter is used for
//! the <b>velocity-based algorithm</b> (TypeIIRMLVelocity):
//!
//!  - If the flag RMLFlags::NO_SYNCHRONIZATION is used, the value of
//!    \c MinimumSynchronizationTime will be ignored.
//!  - If the value of \c DesiredExecutionTime is less then the minimum
//!    possible synchronization time, it will be ignored and the
//!    minimum possible synchronization time will be used.
//!  - Otherwise, a trajectory will be computed whose execution time
//!    is \c MinimumSynchronizationTime.
//!
//! \sa RMLFlags::SyncBehaviorEnum
//! \sa TypeIIRMLPosition::SynchronizationTime
//! \sa TypeIIRMLVelocity::SynchronizationTime
//! \sa RMLOutputParameters::SynchronizationTime
//! \sa \ref page_Code_07_RMLPositionSampleApplication
//! \sa \ref page_Code_08_RMLVelocitySampleApplication
//  ----------------------------------------------------------
    double                  MinimumSynchronizationTime;


//  ---------------------- Doxygen info ----------------------
//! \var RMLBoolVector *SelectionVector
//!
//! \brief
//! A pointer to the selection vector \f$ \vec{S}_{i} \f$
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - SetSelectionVector(const RMLBoolVector &InputVector)\n\n
//!  - SetSelectionVector(const bool *InputVector)\n\n
//!  - SetSelectionVectorElement(const bool &InputValue, const unsigned int &Index)\n\n
//!  - GetSelectionVector(RMLBoolVector *InputVector) const\n\n
//!  - GetSelectionVector(bool *InputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetSelectionVectorElement(bool *InputValue, const unsigned int &Index) const\n\n
//!  - GetSelectionVectorElement(const unsigned int &Index) const\n\n
//  ----------------------------------------------------------
    RMLBoolVector           *SelectionVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *CurrentPositionVector
//!
//! \brief
//! A pointer to the current position vector \f$ \vec{P}_{i} \f$
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - SetCurrentPositionVector(const RMLDoubleVector &InputVector)\n\n
//!  - SetCurrentPositionVector(const double *InputVector)\n\n
//!  - SetCurrentPositionVectorElement(const double &InputValue, const unsigned int &Index)\n\n
//!  - GetCurrentPositionVector(RMLDoubleVector *InputVector) const\n\n
//!  - GetCurrentPositionVector(double *InputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetCurrentPositionVectorElement(double *InputValue, const unsigned int &Index) const\n\n
//!  - GetCurrentPositionVectorElement(const unsigned int &Index) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         *CurrentPositionVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *CurrentVelocityVector
//!
//! \brief
//! A pointer to the current velocity vector \f$ \vec{V}_{i} \f$
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - SetCurrentVelocityVector(const RMLDoubleVector &InputVector)\n\n
//!  - SetCurrentVelocityVector(const double *InputVector)\n\n
//!  - SetCurrentVelocityVectorElement(const double &InputValue, const unsigned int &Index)\n\n
//!  - GetCurrentVelocityVector(RMLDoubleVector *InputVector) const\n\n
//!  - GetCurrentVelocityVector(double *InputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetCurrentVelocityVectorElement(double *InputValue, const unsigned int &Index) const\n\n
//!  - GetCurrentVelocityVectorElement(const unsigned int &Index) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         *CurrentVelocityVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *CurrentAccelerationVector
//!
//! \brief
//! A pointer to the current acceleration vector \f$ \vec{A}_{i} \f$
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - SetCurrentAccelerationVector(const RMLDoubleVector &InputVector)\n\n
//!  - SetCurrentAccelerationVector(const double *InputVector)\n\n
//!  - SetCurrentAccelerationVectorElement(const double &InputValue, const unsigned int &Index)\n\n
//!  - GetCurrentAccelerationVector(RMLDoubleVector *InputVector) const\n\n
//!  - GetCurrentAccelerationVector(double *InputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetCurrentAccelerationVectorElement(double *InputValue, const unsigned int &Index) const\n\n
//!  - GetCurrentAccelerationVectorElement(const unsigned int &Index) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         *CurrentAccelerationVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MaxAccelerationVector
//!
//! \brief
//! A pointer to the maximum acceleration vector \f$ \vec{A}_{i}^{\,max} \f$
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - SetMaxAccelerationVector(const RMLDoubleVector &InputVector)\n\n
//!  - SetMaxAccelerationVector(const double *InputVector)\n\n
//!  - SetMaxAccelerationVectorElement(const double &InputValue, const unsigned int &Index)\n\n
//!  - GetMaxAccelerationVector(RMLDoubleVector *InputVector) const\n\n
//!  - GetMaxAccelerationVector(double *InputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetMaxAccelerationVectorElement(double *InputValue, const unsigned int &Index) const\n\n
//!  - GetMaxAccelerationVectorElement(const unsigned int &Index) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         *MaxAccelerationVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MaxJerkVector
//!
//! \brief
//! A pointer to the maximum jerk vector \f$ \vec{J}_{i}^{\,max} \f$
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - SetMaxJerkVector(const RMLDoubleVector &InputVector)\n\n
//!  - SetMaxJerkVector(const double *InputVector)\n\n
//!  - SetMaxJerkVectorElement(const double &InputValue, const unsigned int &Index)\n\n
//!  - GetMaxJerkVector(RMLDoubleVector *InputVector) const\n\n
//!  - GetMaxJerkVector(double *InputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetMaxJerkVectorElement(double *InputValue, const unsigned int &Index) const\n\n
//!  - GetMaxJerkVectorElement(const unsigned int &Index) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         *MaxJerkVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *TargetVelocityVector
//!
//! \brief
//! A pointer to the target velocity vector \f$ \vec{V}_{i}^{\,trgt} \f$
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - SetTargetVelocityVector(const RMLDoubleVector &InputVector)\n\n
//!  - SetTargetVelocityVector(const double *InputVector)\n\n
//!  - SetTargetVelocityVectorElement(const double &InputValue, const unsigned int &Index)\n\n
//!  - GetTargetVelocityVector(RMLDoubleVector *InputVector) const\n\n
//!  - GetTargetVelocityVector(double *InputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetTargetVelocityVectorElement(double *InputValue, const unsigned int &Index) const\n\n
//!  - GetTargetVelocityVectorElement(const unsigned int &Index) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         *TargetVelocityVector;


};// class RMLInputParameters



#endif
