//  ---------------------- Doxygen info ----------------------
//! \file RMLPositionInputParameters.h
//!
//! \brief
//! Header file for the class RMLPositionInputParameters
//!
//! \details
//! The class RMLPositionInputParameters is derived from the class
//! RMLInputParameters and constitutes a part of the interface for the
//! position-based On-Line Trajectory Generation algorithm.
//!
//! \sa RMLInputParameters
//! \sa RMLPositionOutputParameters
//! \sa RMLVelocityInputParameters
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


#ifndef __RMLPositionInputParameters__
#define __RMLPositionInputParameters__


#include <RMLInputParameters.h>
#include <RMLVector.h>
#include <string.h>
#include <math.h>


//  ---------------------- Doxygen info ----------------------
//! \class RMLPositionInputParameters
//!
//! \brief
//! Class for the input parameters of the position-based On-Line
//! Trajectory Generation algorithm
//!
//! \details
//! The class RMLPositionInputParameters is derived from the class
//! RMLInputParameters and constitutes a part of the interface for the
//! position-based On-Line Trajectory Generation algorithm.\n\n
//!
//! A detailed description can be found at \ref page_InputValues.
//!
//! \sa ReflexxesAPI
//! \sa RMLInputParameters
//! \sa RMLVelocityInputParameters
//! \sa RMLPositionOutputParameters
//! \sa \ref page_InputValues
//  ----------------------------------------------------------
class RMLPositionInputParameters : public RMLInputParameters
{
public:


//  ---------------------- Doxygen info ----------------------
//! \fn RMLPositionInputParameters(const unsigned int DegreesOfFreedom)
//!
//! \brief
//! Constructor of class RMLPositionInputParameters
//!
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//!
//! \param DegreesOfFreedom
//! Specifies the number of degrees of freedom
//  ----------------------------------------------------------
    RMLPositionInputParameters(const unsigned int DegreesOfFreedom) : RMLInputParameters(DegreesOfFreedom)
    {
        this->MaxVelocityVector                 =   new RMLDoubleVector (DegreesOfFreedom)  ;
        this->TargetPositionVector              =   new RMLDoubleVector (DegreesOfFreedom)  ;
        this->AlternativeTargetVelocityVector   =   new RMLDoubleVector (DegreesOfFreedom)  ;

        memset(this->MaxVelocityVector->VecData                 ,   0x0 ,       DegreesOfFreedom * sizeof(double));
        memset(this->TargetPositionVector->VecData              ,   0x0 ,       DegreesOfFreedom * sizeof(double));
        memset(this->AlternativeTargetVelocityVector->VecData   ,   0x0 ,       DegreesOfFreedom * sizeof(double));
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLPositionInputParameters(const RMLPositionInputParameters& IP)
//!
//! \brief
//! Copy constructor of class RMLPositionInputParameters
//!
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//!
//! \param IP
//! Object to be copied
//  ----------------------------------------------------------
    RMLPositionInputParameters(const RMLPositionInputParameters &IP) : RMLInputParameters(IP)
    {
        this->MaxVelocityVector                     =   new RMLDoubleVector ((IP.CurrentPositionVector)->GetVecDim())   ;
        this->TargetPositionVector                  =   new RMLDoubleVector ((IP.CurrentPositionVector)->GetVecDim())   ;
        this->AlternativeTargetVelocityVector       =   new RMLDoubleVector ((IP.CurrentPositionVector)->GetVecDim())   ;

        *(this->MaxVelocityVector)                  =   *(IP.MaxVelocityVector)                                         ;
        *(this->TargetPositionVector)               =   *(IP.TargetPositionVector)                                      ;
        *(this->AlternativeTargetVelocityVector)    =   *(IP.AlternativeTargetVelocityVector)                           ;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLPositionInputParameters(void)
//!
//! \brief
//! Destructor of class RMLPositionInputParameters
//  ----------------------------------------------------------
    ~RMLPositionInputParameters(void)
    {
        delete  this->MaxVelocityVector                     ;
        delete  this->TargetPositionVector                  ;
        delete  this->AlternativeTargetVelocityVector       ;

        this->MaxVelocityVector                 =   NULL    ;
        this->TargetPositionVector              =   NULL    ;
        this->AlternativeTargetVelocityVector   =   NULL    ;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLPositionInputParameters &operator = (const RMLPositionInputParameters &IP)
//!
//! \brief
//! Copy operator
//!
//! \param IP
//! RMLPositionInputParameters object to be copied
//  ----------------------------------------------------------
    RMLPositionInputParameters &operator = (const RMLPositionInputParameters &IP)
    {
        RMLInputParameters::operator=(IP);

        *(this->MaxVelocityVector               )   =   *(IP.MaxVelocityVector)                 ;
        *(this->TargetPositionVector            )   =   *(IP.TargetPositionVector)              ;
        *(this->AlternativeTargetVelocityVector )   =   *(IP.AlternativeTargetVelocityVector)   ;

        return(*this);
    }


// #############################################################################


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetMaxVelocityVector(const RMLDoubleVector &InputVector)
//!
//! \brief
//! Sets the maximum velocity vector \f$ \vec{V}_{i}^{\,max} \f$ by using the
//! an \c RMLDoubleVector object
//!
//! \param InputVector
//! The input vector, whose elements are copied to the attributes of this
//! class.
//!
//! \sa SetMaxVelocityVector(const double *InputVector)
//! \sa SetMaxVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetMaxVelocityVector(RMLDoubleVector *InputVector) const
//  ----------------------------------------------------------
    inline void SetMaxVelocityVector(const RMLDoubleVector &InputVector)
    {
        *(this->MaxVelocityVector) = InputVector;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetMaxVelocityVector(const double *InputVector)
//!
//! \brief
//! Sets the maximum velocity vector \f$ \vec{V}_{i}^{\,max} \f$ by using a
//! native C \c double array
//!
//! \param InputVector
//! The input vector to an array of \c double values, whose elements are
//! copied to the attributes of this class.
//!
//! \sa SetMaxVelocityVector(const RMLDoubleVector &InputVector)
//! \sa SetMaxVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetMaxVelocityVector(double *InputVector, const unsigned int &SizeInBytes) const
//  ----------------------------------------------------------
    inline void SetMaxVelocityVector(const double *InputVector)
    {
        memcpy(     (void*)this->MaxVelocityVector->VecData
                ,   (void*)InputVector
                ,   (this->MaxVelocityVector->GetVecDim() * sizeof(double)) );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetMaxVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//!
//! \brief
//! Sets one element of the maximum velocity vector \f$ \vec{V}_{i}^{\,max} \f$
//!
//! \param InputValue
//! The input value that is copied to the element \c Index of the
//! maximum velocity input vector attribute of this class.
//!
//! \param Index
//! The \em index of the element to be copied
//!
//! \sa SetMaxVelocityVector(const RMLDoubleVector &InputVector)
//! \sa SetMaxVelocityVector(const double *InputVector)
//! \sa GetMaxVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void SetMaxVelocityVectorElement(    const double        &InputValue
                                            ,   const unsigned int  &Index)
    {
        (*this->MaxVelocityVector)[Index]   =   InputValue;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetMaxVelocityVector(RMLDoubleVector *InputVector) const
//!
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! maximum velocity vector \f$ \vec{V}_{i}^{\,max} \f$ to the
//! \c RMLDoubleVector object referred to by \c InputVector
//!
//! \param InputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be
//! copied
//!
//! \sa SetMaxVelocityVector(const RMLDoubleVector &InputVector)
//! \sa GetMaxVelocityVector(double *InputVector, const unsigned int &SizeInBytes) const
//! \sa GetMaxVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetMaxVelocityVector(RMLDoubleVector *InputVector) const
    {
        *InputVector    =   *(this->MaxVelocityVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetMaxVelocityVector(double *InputVector, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the array of \c double values representing the maximum
//! velocity vector \f$ \vec{V}_{i}^{\,max} \f$ to the memory pointed to by
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
//! \sa SetMaxVelocityVector(const double *InputVector)
//! \sa GetMaxVelocityVector(RMLDoubleVector *InputVector) const
//! \sa GetMaxVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetMaxVelocityVector(       double              *InputVector
                                        ,   const unsigned int  &SizeInBytes) const
    {
        memcpy(     (void*)InputVector
                ,   (void*)this->MaxVelocityVector->VecData
                ,   SizeInBytes );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetMaxVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//!
//! \brief
//! Copies one element of the maximum velocity vector
//! \f$ \vec{V}_{i}^{\,max} \f$ to the memory pointed to by \c InputValue
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
//! \sa SetMaxVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetMaxVelocityVector(RMLDoubleVector *InputVector) const
//! \sa GetMaxVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//! \sa GetMaxVelocityVectorElement(const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetMaxVelocityVectorElement(    double              *InputValue
                                            ,   const unsigned int  &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->MaxVelocityVector->GetVecDim() ) )
        {
            *InputValue =   0.0;
        }
        else
        {
            *InputValue =   (*this->MaxVelocityVector)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetMaxVelocityVectorElement(const unsigned int &Index) const
//!
//! \brief
//! Returns one single element of the maximum velocity vector
//! \f$ \vec{V}_{i}^{\,max} \f$
//!
//! \param Index
//! Specifies the desired element of the vector. The index of the first
//! vector element is \em 0 (zero). If the value of \c Index value is
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c InputValue.
//!
//! \sa SetMaxVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetMaxVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline double GetMaxVelocityVectorElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->MaxVelocityVector->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {
            return( (*this->MaxVelocityVector)[Index] );
        }
    }


// #############################################################################


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetTargetPositionVector(const RMLDoubleVector &InputVector)
//!
//! \brief
//! Sets the target position vector \f$ \vec{P}_{i}^{\,trgt} \f$ by using the
//! an \c RMLDoubleVector object
//!
//! \param InputVector
//! The input vector, whose elements are copied to the attributes of this
//! class.
//!
//! \sa SetTargetPositionVector(const double *InputVector)
//! \sa SetTargetPositionVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetTargetPositionVector(RMLDoubleVector *InputVector) const
//  ----------------------------------------------------------
    inline void SetTargetPositionVector(const RMLDoubleVector &InputVector)
    {
        *(this->TargetPositionVector) = InputVector;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetTargetPositionVector(const double *InputVector)
//!
//! \brief
//! Sets the target position vector \f$ \vec{P}_{i}^{\,trgt} \f$ by using a
//! native C \c double array
//!
//! \param InputVector
//! The input vector to an array of \c double values, whose elements are
//! copied to the attributes of this class.
//!
//! \sa SetTargetPositionVector(const RMLDoubleVector &InputVector)
//! \sa SetTargetPositionVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetTargetPositionVector(double *InputVector, const unsigned int &SizeInBytes) const
//  ----------------------------------------------------------
    inline void SetTargetPositionVector(const double *InputVector)
    {
        memcpy(     (void*)this->TargetPositionVector->VecData
                ,   (void*)InputVector
                ,   (this->TargetPositionVector->GetVecDim() * sizeof(double))  );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetTargetPositionVectorElement(const double &InputValue, const unsigned int &Index)
//!
//! \brief
//! Sets one element of the target position vector \f$ \vec{P}_{i}^{\,trgt} \f$
//!
//! \param InputValue
//! The input value that is copied to the element \c Index of the target
//! position input vector attribute of this class.
//!
//! \param Index
//! The \em index of the element to be copied
//!
//! \sa SetTargetPositionVector(const RMLDoubleVector &InputVector)
//! \sa SetTargetPositionVector(const double *InputVector)
//! \sa GetTargetPositionVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void SetTargetPositionVectorElement(     const double        &InputValue
                                                ,   const unsigned int  &Index)
    {
        (*this->TargetPositionVector)[Index]    =   InputValue;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetTargetPositionVector(RMLDoubleVector *InputVector) const
//!
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! target position vector \f$ \vec{P}_{i}^{\,trgt} \f$ to the \c RMLDoubleVector
//! object referred to by \c InputVector
//!
//! \param InputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be
//! copied
//!
//! \sa SetTargetPositionVector(const RMLDoubleVector &InputVector)
//! \sa GetTargetPositionVector(double *InputVector, const unsigned int &SizeInBytes) const
//! \sa GetTargetPositionVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetTargetPositionVector(RMLDoubleVector *InputVector) const
    {
        *InputVector    =   *(this->TargetPositionVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetTargetPositionVector(double *InputVector, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the array of \c double values representing the target
//! position vector \f$ \vec{P}_{i}^{\,trgt} \f$ to the memory pointed to by
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
//! \sa SetTargetPositionVector(const double *InputVector)
//! \sa GetTargetPositionVector(RMLDoubleVector *InputVector) const
//! \sa GetTargetPositionVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetTargetPositionVector(    double              *InputVector
                                        ,   const unsigned int  &SizeInBytes) const
    {
        memcpy(     (void*)InputVector
                ,   (void*)this->TargetPositionVector->VecData
                ,   SizeInBytes );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetTargetPositionVectorElement(double *InputValue, const unsigned int &Index) const
//!
//! \brief
//! Copies one element of the target position vector
//! \f$ \vec{P}_{i}^{\,trgt} \f$ to the memory pointed to by \c InputValue
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
//! \sa SetTargetPositionVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetTargetPositionVector(RMLDoubleVector *InputVector) const
//! \sa GetTargetPositionVectorElement(double *InputValue, const unsigned int &Index) const
//! \sa GetTargetPositionVectorElement(const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetTargetPositionVectorElement(     double              *InputValue
                                                ,   const unsigned int  &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->TargetPositionVector->GetVecDim() ) )
        {
            *InputValue =   0.0;
        }
        else
        {
            *InputValue =   (*this->TargetPositionVector)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetTargetPositionVectorElement(const unsigned int &Index) const
//!
//! \brief
//! Returns one single element of the target position vector
//! \f$ \vec{P}_{i}^{\,trgt} \f$
//!
//! \param Index
//! Specifies the desired element of the vector. The index of the first
//! vector element is \em 0 (zero). If the value of \c Index value is
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c InputValue.
//!
//! \sa SetTargetPositionVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetTargetPositionVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline double GetTargetPositionVectorElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->TargetPositionVector->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {
            return( (*this->TargetPositionVector)[Index] );
        }
    }


// #############################################################################


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetAlternativeTargetVelocityVector(const RMLDoubleVector &InputVector)
//!
//! \brief
//! Sets the alternative target velocity vector \f$ \vec{V}_{i}^{\,\underline{trgt}} \f$
//! by using the an \c RMLDoubleVector object.
//!
//! \details
//! This vector only becomes applied for the fall back strategy when no
//! correct output values can be calculated (e.g., due to incorrect
//! input values.
//!
//! \param InputVector
//! The input vector, whose elements are copied to the attributes of this
//! class.
//!
//! \note
//! By default and and in most cases this vector equals the zero vector.
//!
//! \sa SetAlternativeTargetVelocityVector(const double *InputVector)
//! \sa SetAlternativeTargetVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetAlternativeTargetVelocityVector(RMLDoubleVector *InputVector) const
//  ----------------------------------------------------------
    inline void SetAlternativeTargetVelocityVector(const RMLDoubleVector &InputVector)
    {
        *(this->AlternativeTargetVelocityVector) = InputVector;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetAlternativeTargetVelocityVector(const double *InputVector)
//!
//! \brief
//! Sets the alternative target velocity vector \f$ \vec{V}_{i}^{\,max} \f$
//! by using the an \c RMLDoubleVector object.
//!
//! \details
//! This vector only becomes applied for the fall back strategy when no
//! correct output values can be calculated (e.g., due to incorrect
//! input values.
//!
//! \param InputVector
//! The input vector to an array of \c double values, whose elements are
//! copied to the attributes of this class.
//!
//! \note
//! By default and and in most cases this vector equals the zero vector.
//!
//! \sa SetAlternativeTargetVelocityVector(const RMLDoubleVector &InputVector)
//! \sa SetAlternativeTargetVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetAlternativeTargetVelocityVector(double *InputVector, const unsigned int &SizeInBytes) const
//  ----------------------------------------------------------
    inline void SetAlternativeTargetVelocityVector(const double *InputVector)
    {
        memcpy(     (void*)this->AlternativeTargetVelocityVector->VecData
                ,   (void*)InputVector
                ,   (this->AlternativeTargetVelocityVector->GetVecDim() * sizeof(double))   );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetAlternativeTargetVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//!
//! \brief
//! Sets one element of the alternative target velocity vector
//! \f$ \vec{V}_{i}^{\,\underline{trgt}} \f$
//!
//! \details
//! This vector only becomes applied for the fall back strategy when no
//! correct output values can be calculated (e.g., due to incorrect
//! input values.
//!
//! \param InputValue
//! The input value that is copied to the element \c Index of the
//! maximum velocity input vector attribute of this class.
//!
//! \param Index
//! The \em index of the element to be copied
//!
//! \note
//! By default and and in most cases this vector equals the zero vector.
//!
//! \sa SetAlternativeTargetVelocityVector(const RMLDoubleVector &InputVector)
//! \sa SetAlternativeTargetVelocityVector(const double *InputVector)
//! \sa GetAlternativeTargetVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void SetAlternativeTargetVelocityVectorElement(      const double        &InputValue
                                                            ,   const unsigned int  &Index)
    {
        (*this->AlternativeTargetVelocityVector)[Index] =   InputValue;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetAlternativeTargetVelocityVector(RMLDoubleVector *InputVector) const
//!
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! alternative target velocity vector
//! \f$ \vec{V}_{i}^{\,\underline{trgt}} \f$ to the
//! \c RMLDoubleVector object referred to by \c InputVector
//!
//! \param InputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be
//! copied
//!
//! \sa SetAlternativeTargetVelocityVector(const RMLDoubleVector &InputVector)
//! \sa GetAlternativeTargetVelocityVector(double *InputVector, const unsigned int &SizeInBytes) const
//! \sa GetAlternativeTargetVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetAlternativeTargetVelocityVector(RMLDoubleVector *InputVector) const
    {
        *InputVector    =   *(this->AlternativeTargetVelocityVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetAlternativeTargetVelocityVector(double *InputVector, const unsigned int &SizeInBytes) const
//!
//! \brief
//! Copies the array of \c double values representing the alternative
//! target velocity vector
//! \f$ \vec{V}_{i}^{\,\underline{trgt}} \f$ to the memory pointed to by
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
//! \sa SetAlternativeTargetVelocityVector(const double *InputVector)
//! \sa GetAlternativeTargetVelocityVector(RMLDoubleVector *InputVector) const
//! \sa GetAlternativeTargetVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetAlternativeTargetVelocityVector(     double              *InputVector
                                                    ,   const unsigned int  &SizeInBytes) const
    {
        memcpy(     (void*)InputVector
                ,   (void*)this->AlternativeTargetVelocityVector->VecData
                ,   SizeInBytes );
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetAlternativeTargetVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//!
//! \brief
//! Copies one element of the alternative target velocity vector
//! \f$ \vec{V}_{i}^{\,\underline{trgt}} \f$ to the memory pointed to by
//! \c InputValue
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
//! \sa SetAlternativeTargetVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetAlternativeTargetVelocityVector(RMLDoubleVector *InputVector) const
//! \sa GetAlternativeTargetVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//! \sa GetAlternativeTargetVelocityVectorElement(const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetAlternativeTargetVelocityVectorElement(      double              *InputValue
                                                            ,   const unsigned int  &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->AlternativeTargetVelocityVector->GetVecDim() ) )
        {
            *InputValue =   0.0;
        }
        else
        {
            *InputValue =   (*this->AlternativeTargetVelocityVector)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetAlternativeTargetVelocityVectorElement(const unsigned int &Index) const
//!
//! \brief
//! Returns one single element of the alternative target velocity vector
//! \f$ \vec{V}_{i}^{\,\underline{trgt}} \f$
//!
//! \param Index
//! Specifies the desired element of the vector. The index of the first
//! vector element is \em 0 (zero). If the value of \c Index value is
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c InputValue.
//!
//! \sa SetAlternativeTargetVelocityVectorElement(const double &InputValue, const unsigned int &Index)
//! \sa GetAlternativeTargetVelocityVectorElement(double *InputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline double GetAlternativeTargetVelocityVectorElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->AlternativeTargetVelocityVector->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {
            return( (*this->AlternativeTargetVelocityVector)[Index] );
        }
    }


// #############################################################################


//  ---------------------- Doxygen info ----------------------
//! \fn bool CheckForValidity(void) const
//!
//! \brief
//! Checks the input parameters for validity
//!
//! \details
//! This is an important method to ensure numerical robustness of the
//! On-Line Trajectory Generation algorithm. Only if the result of this
//! method is \c true, a correct computation of output values can be
//! obtained. If the result is false, the On-Line Trajectory Generation
//! Algorithm will try compute correct output values, and in many cases,
//! this is possible, but it is not guaranteed. All input values have to
//! be within a proper order of magnitude
//! (cf. RMLPositionInputParameters::MAXIMUM_MAGNITUDE_RANGE), the kinematic
//! motion constraints, that is, the maximum values for velocity, and
//! acceleration, have to be positive, and the target velocity
//! must not be greater than the maximum velocity.
//!
//! \return
//!  - \c true, if all requirements for input parameters are met
//!  - \c false, otherwise
//!
//! \note
//!  - In order to achieve an optimal performance of the On-Line Trajectory
//!    Generation algorithm, this method is only executed in case of an error
//!    with the purpose to specify the error code of the result value
//!    (cf. ReflexxesAPI::RMLResultValue). As a consequence, there is no loss
//!    CPU time inside of the Reflexxes Motion Library, such that
//!    also the worst-case execution time is not negatively influenced.
//!    Please consider this method just as an additional possibility for
//!    the user application to check whether the current input values are
//!    valid for the On-Line Trajectory Generation algorithm.\n\n
//!  - If the On-Line Trajectory Generation algorithm is fed with invalid
//!    input parameters, it will  try to compute correct output values.
//!    This works for many cases, and correct output values can be
//!    provided. If no correct output values can be calculated, it is
//!    guaranteed that valid  output are provided in any case
//!    (cf. \ref page_ErrorHandling).
//!
//! \sa ReflexxesAPI::RMLResultValue
//! \sa RMLPositionInputParameters::MAXIMUM_MAGNITUDE_RANGE
//! \sa RMLVelocityInputParameters::CheckForValidity()
//! \sa RMLVelocityInputParameters::MAXIMUM_MAGNITUDE_RANGE
//! \sa \ref page_InputValues
//  ----------------------------------------------------------
    bool CheckForValidity(void) const
    {
        unsigned int        i                           =   0;

        double              MinimumOrderOfMagnitude     =   0.0
                        ,   MaximumOrderOfMagnitude     =   0.0;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if ((this->SelectionVector->VecData)[i])
            {
                if (    (this->MaxVelocityVector->VecData               [i] <=  0.0 )
                    ||  (this->MaxAccelerationVector->VecData           [i] <=  0.0 )
                    ||  (   fabs(this->TargetVelocityVector->VecData    [i]         )
                        >   (this->MaxVelocityVector->VecData)          [i]         )   )
                {
                    return(false);
                }

                if (    ((this->MaxVelocityVector->VecData)[i]  >=  (this->MaxAccelerationVector->VecData)          [i] )
                    &&  ((this->MaxVelocityVector->VecData)[i]  >=  fabs((this->CurrentPositionVector->VecData)     [i]))
                    &&  ((this->MaxVelocityVector->VecData)[i]  >=  fabs((this->TargetPositionVector->VecData)      [i]))
                    &&  ((this->MaxVelocityVector->VecData)[i]  >=  fabs((this->CurrentVelocityVector->VecData)     [i]))
                    &&  ((this->MaxVelocityVector->VecData)[i]  >=  fabs((this->CurrentAccelerationVector->VecData) [i])))
                {
                    MaximumOrderOfMagnitude =   (this->MaxVelocityVector->VecData)[i];
                }
                else
                {
                    if (    ((this->MaxAccelerationVector->VecData)[i]  >=  fabs((this->CurrentPositionVector->VecData)     [i]))
                        &&  ((this->MaxAccelerationVector->VecData)[i]  >=  fabs((this->TargetPositionVector->VecData)      [i]))
                        &&  ((this->MaxAccelerationVector->VecData)[i]  >=  fabs((this->CurrentVelocityVector->VecData)     [i]))
                        &&  ((this->MaxAccelerationVector->VecData)[i]  >=  fabs((this->CurrentAccelerationVector->VecData) [i])))
                    {
                        MaximumOrderOfMagnitude =   (this->MaxAccelerationVector->VecData)[i];
                    }
                    else
                    {
                        if (    (fabs((this->CurrentPositionVector->VecData)[i])    >=  fabs((this->TargetPositionVector->VecData)      [i]))
                            &&  (fabs((this->CurrentPositionVector->VecData)[i])    >=  fabs((this->CurrentVelocityVector->VecData)     [i]))
                            &&  (fabs((this->CurrentPositionVector->VecData)[i])    >=  fabs((this->CurrentAccelerationVector->VecData) [i])))
                        {
                            MaximumOrderOfMagnitude =   fabs((this->CurrentPositionVector->VecData)[i]);
                        }
                        else
                        {
                            if (    (fabs((this->TargetPositionVector->VecData)[i]) >=  fabs((this->CurrentVelocityVector->VecData)     [i]))
                                &&  (fabs((this->TargetPositionVector->VecData)[i]) >=  fabs((this->CurrentAccelerationVector->VecData) [i])))
                            {
                                MaximumOrderOfMagnitude =   fabs((this->TargetPositionVector->VecData)[i]);
                            }
                            else
                            {
                                MaximumOrderOfMagnitude =   fabs((this->CurrentAccelerationVector->VecData)[i]);
                            }
                        }
                    }
                }

                if ((this->MaxVelocityVector->VecData)[i]   <=  (this->MaxAccelerationVector->VecData)  [i] )
                {
                    MinimumOrderOfMagnitude =   (this->MaxVelocityVector->VecData)[i];
                }
                else
                {
                    MinimumOrderOfMagnitude =   (this->MaxAccelerationVector->VecData)[i];
                }

                // The target velocity value does not have to be checked as we
                // already know that is lesser than the maximum velocity value.
                // The alternative target velocity vector does not have to be
                // checked.

                // The value of MinimumOrderOfMagnitude is greater than
                // zero:
                if (    (MaximumOrderOfMagnitude / MinimumOrderOfMagnitude)
                    >   (double)pow((float)10, (int)(RMLPositionInputParameters::MAXIMUM_MAGNITUDE_RANGE)))
                {
                    return(false);
                }
            }
        }

        if (this->MinimumSynchronizationTime > 1e10)
        {
            return(false);
        }

        return(true);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn void Echo(FILE* FileHandler = stdout) const
//!
//! \brief
//! \copybrief RMLInputParameters::Echo()
//!
//! \details
//! \copydetails RMLInputParameters::Echo()
//  ----------------------------------------------------------
    void Echo(FILE* FileHandler = stdout) const
    {
        unsigned int        i   =   0;

        if (FileHandler == NULL)
        {
            return;
        }

        RMLInputParameters::Echo(FileHandler);

        fprintf(FileHandler, "Max. velocity vector       : ");
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            fprintf(FileHandler, " %.20le ", this->MaxVelocityVector->VecData[i]);
        }
        fprintf(FileHandler,   "\nTarget position vector     : ");
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            fprintf(FileHandler, " %.20le ", this->TargetPositionVector->VecData[i]);
        }
        fprintf(FileHandler,   "\nAlternative trgt vel. vector: ");
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            fprintf(FileHandler, " %.20le ", this->AlternativeTargetVelocityVector->VecData[i]);
        }
        fprintf(FileHandler, "\n");


        return;
    }


protected:


    enum
    {
//  ---------------------- Doxygen info ----------------------
//! \brief
//! Specifies the maximum allowed range for the orders of magnitude of
//! the input values.
//  ----------------------------------------------------------
        MAXIMUM_MAGNITUDE_RANGE     =   8
    };


public:


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MaxVelocityVector
//!
//! \brief
//! A pointer to the maximum velocity vector \f$ \vec{V}_{i}^{\,max} \f$
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - SetMaxVelocityVector(const RMLDoubleVector &InputVector)\n\n
//!  - SetMaxVelocityVector(const double *InputVector)\n\n
//!  - SetMaxVelocityVectorElement(const double &InputValue, const unsigned int &Index)\n\n
//!  - GetMaxVelocityVector(RMLDoubleVector *InputVector) const\n\n
//!  - GetMaxVelocityVector(double *InputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetMaxVelocityVectorElement(double *InputValue, const unsigned int &Index) const\n\n
//!  - GetMaxVelocityVectorElement(const unsigned int &Index) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         *MaxVelocityVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *TargetPositionVector
//!
//! \brief
//! A pointer to the target position vector \f$ \vec{P}_{i}^{\,trgt} \f$
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - SetTargetPositionVector(const RMLDoubleVector &InputVector)\n\n
//!  - SetTargetPositionVector(const double *InputVector)\n\n
//!  - SetTargetPositionVectorElement(const double &InputValue, const unsigned int &Index)\n\n
//!  - GetTargetPositionVector(RMLDoubleVector *InputVector) const\n\n
//!  - GetTargetPositionVector(double *InputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetTargetPositionVectorElement(double *InputValue, const unsigned int &Index) const\n\n
//!  - GetTargetPositionVectorElement(const unsigned int &Index) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector         *TargetPositionVector;


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *AlternativeTargetVelocityVector
//!
//! \brief
//! A pointer to an alternative target velocity vector
//! \f$ \vec{V}_{i}^{\,\underline{trgt}} \f$
//!
//! \details
//! This vector becomes applied in the second safety layer, which becomes
//! activated by calling TypeIVRMLPosition::FallBackStrategy(). The second
//! layer applies the velocity-based On-Line Trajectory Generation
//! algorithm, and this vector is used as desired target velocity vector
//! \f$ \vec{V}_{i}^{\,trgt} \f$. In many use cases, a target velocity
//! vector of zero may be sufficient, but in dependence on the application
//! it may be desirable to specify an alternative target velocity vector
//! that becomes applied in the second safety layer
//! (cf. TypeIVRMLVelocity). As a further alternative, the flag
//! RMLPositionFlags::KeepCurrentVelocityInCaseOfFallbackStrategy may be
//! used to use the current velocity vector as set-point and input value
//! for the velocity-based On-Line Trajectory Generation algorithm.\n\n
//!
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - SetAlternativeTargetVelocityVector(const RMLDoubleVector &InputVector)\n\n
//!  - SetAlternativeTargetVelocityVector(const double *InputVector)\n\n
//!  - SetAlternativeTargetVelocityVectorElement(const double &InputValue, const unsigned int &Index)\n\n
//!  - GetAlternativeTargetVelocityVector(RMLDoubleVector *InputVector) const\n\n
//!  - GetAlternativeTargetVelocityVector(double *InputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetAlternativeTargetVelocityVectorElement(double *InputValue, const unsigned int &Index) const\n\n
//!  - GetAlternativeTargetVelocityVectorElement(const unsigned int &Index) const\n\n
//!
//! \note
//! By default, this vector is set to the zero vector, such that in case
//! of an internal error (e.g., erroneous input values for the
//! position-based On-Line Trajectory Generation algorithm) a
//! motion trajectory guiding all selected degrees of freedom to
//! zero-velocity will be generated.
//!
//! \sa RMLPositionFlags::KeepCurrentVelocityInCaseOfFallbackStrategy
//! \sa \ref page_ErrorHandling
//  ----------------------------------------------------------
    RMLDoubleVector         *AlternativeTargetVelocityVector;


};// class RMLPositionInputParameters



#endif


