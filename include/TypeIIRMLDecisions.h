//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLDecisions.h
//!
//! \brief
//! Header file for decisions of the two decision trees of the
//! Type II On-Line Trajectory Generation algorithm
//!
//! \details
//! This file contains all necessary decisions for the Type II On-Line
//! Trajectory Generation algorithm. All functions are part
//! of the namespace TypeIIRMLMath.
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


#ifndef __TypeIIRMLDecisions__
#define __TypeIIRMLDecisions__



namespace TypeIIRMLMath
{

//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1A__001(const double &CurrentVelocity)
//!
//! \brief
//! Is (vi >= 0)?
//  ----------------------------------------------------------
bool Decision_1A__001(      const double &CurrentVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1A__002(const double &CurrentVelocity, const double &MaxVelocity)
//!
//! \brief
//! Is (vi <= +vmax)?
//  ----------------------------------------------------------
bool Decision_1A__002(      const double &CurrentVelocity
                        ,   const double &MaxVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1A__003(const double &CurrentVelocity, const double &TargetVelocity)
//!
//! \brief
//! Is (vi <= vtrgt)?
//  ----------------------------------------------------------
bool Decision_1A__003(      const double &CurrentVelocity
                        ,   const double &TargetVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1A__004(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxAcceleration)
//!
//! \brief
//! If v->vtrgt, is p<=ptrgt?
//  ----------------------------------------------------------
bool Decision_1A__004(      const double &CurrentPosition
                        ,   const double &CurrentVelocity
                        ,   const double &TargetPosition
                        ,   const double &TargetVelocity
                        ,   const double &MaxAcceleration);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1A__005(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxVelocity, const double &MaxAcceleration)
//!
//! \brief
//! If v->vmax->vtrgt, is p<=ptrgt?
//  ----------------------------------------------------------
bool Decision_1A__005(      const double &CurrentPosition
                        ,   const double &CurrentVelocity
                        ,   const double &TargetPosition
                        ,   const double &TargetVelocity
                        ,   const double &MaxVelocity
                        ,   const double &MaxAcceleration);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1A__006(const double &TargetVelocity)
//!
//! \brief
//! Is (vtrgt >= 0)?
//  ----------------------------------------------------------
bool Decision_1A__006(      const double &TargetVelocity);



//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1A__007(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxAcceleration)
//!
//! \brief
//! If v->vtrgt, is p>=ptrgt?
//  ----------------------------------------------------------
bool Decision_1A__007(      const double &CurrentPosition
                        ,   const double &CurrentVelocity
                        ,   const double &TargetPosition
                        ,   const double &TargetVelocity
                        ,   const double &MaxAcceleration);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1A__008(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxAcceleration)
//!
//! \brief
//! If v->0->vtrgt, is p>=ptrgt?
//  ----------------------------------------------------------
bool Decision_1A__008(      const double &CurrentPosition
                        ,   const double &CurrentVelocity
                        ,   const double &TargetPosition
                        ,   const double &TargetVelocity
                        ,   const double &MaxAcceleration);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1A__009(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxVelocity, const double &MaxAcceleration)
//!
//! \brief
//! If v->vmax->0->vtrgt, is p>=ptrgt?
//  ----------------------------------------------------------
bool Decision_1A__009(      const double &CurrentPosition
                        ,   const double &CurrentVelocity
                        ,   const double &TargetPosition
                        ,   const double &TargetVelocity
                        ,   const double &MaxVelocity
                        ,   const double &MaxAcceleration);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1B__001(const double &CurrentVelocity)
//!
//! \brief
//! Is (vi >= 0)?
//  ----------------------------------------------------------
bool Decision_1B__001(      const double &CurrentVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1B__002(const double &CurrentVelocity, const double &MaxVelocity)
//!
//! \brief
//! Is (vi <= +vmax)?
//  ----------------------------------------------------------
bool Decision_1B__002(      const double &CurrentVelocity
                        ,   const double &MaxVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1B__003(const double &TargetVelocity)
//!
//! \brief
//! Is (vtrgt >= 0)?
//  ----------------------------------------------------------
bool Decision_1B__003(      const double &TargetVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1B__004(const double &CurrentVelocity, const double &TargetVelocity)
//!
//! \brief
//! Is (vi <= vtrgt)?
//  ----------------------------------------------------------
bool Decision_1B__004(      const double &CurrentVelocity
                        ,   const double &TargetVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1B__005(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxAcceleration)
//!
//! \brief
//! If v->vtrgt, is p<=ptrgt?
//  ----------------------------------------------------------
bool Decision_1B__005(      const double &CurrentPosition
                        ,   const double &CurrentVelocity
                        ,   const double &TargetPosition
                        ,   const double &TargetVelocity
                        ,   const double &MaxAcceleration);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1B__006(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxAcceleration)
//!
//! \brief
//! If v->0->vtrgt, is p<=ptrgt?
//  ----------------------------------------------------------
bool Decision_1B__006(      const double &CurrentPosition
                        ,   const double &CurrentVelocity
                        ,   const double &TargetPosition
                        ,   const double &TargetVelocity
                        ,   const double &MaxAcceleration);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1B__007(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxAcceleration)
//!
//! \brief
//! If v->vtrgt, is p>=ptrgt?
//  ----------------------------------------------------------
bool Decision_1B__007(      const double &CurrentPosition
                        ,   const double &CurrentVelocity
                        ,   const double &TargetPosition
                        ,   const double &TargetVelocity
                        ,   const double &MaxAcceleration);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1C__001(const double &CurrentVelocity)
//!
//! \brief
//! Is (vi >= 0)?
//  ----------------------------------------------------------
bool Decision_1C__001(      const double &CurrentVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1C__002(const double &CurrentVelocity, const double &MaxVelocity)
//!
//! \brief
//! Is (vi <= +vmax)?
//  ----------------------------------------------------------
bool Decision_1C__002(      const double &CurrentVelocity
                        ,   const double &MaxVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1C__003(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxVelocity, const double &MaxAcceleration)
//!
//! \brief
//! If v->vmax->0->vtrgt, is p>=ptrgt?
//  ----------------------------------------------------------
bool Decision_1C__003(      const double &CurrentPosition
                        ,   const double &CurrentVelocity
                        ,   const double &TargetPosition
                        ,   const double &TargetVelocity
                        ,   const double &MaxVelocity
                        ,   const double &MaxAcceleration);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2___001(const double &CurrentVelocity)
//!
//! \brief
//! Is (vi >= 0)?
//  ----------------------------------------------------------
bool Decision_2___001(      const double &CurrentVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2___002(const double &CurrentVelocity, const double &MaxVelocity)
//!
//! \brief
//! Is (vi <= +vmax)?
//  ----------------------------------------------------------
bool Decision_2___002(      const double &CurrentVelocity
                        ,   const double &MaxVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2___003(const double &CurrentVelocity, const double &TargetVelocity)
//!
//! \brief
//! Is (vi <= vtrgt)?
//  ----------------------------------------------------------
bool Decision_2___003(      const double &CurrentVelocity
                        ,   const double &TargetVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2___004(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxAcceleration, const double &CurrentTime, const double &SynchronizationTime)
//!
//! \brief
//! If v->vtrgt->hold, so that t=tsync, is p<=ptrgt?
//  ----------------------------------------------------------
bool Decision_2___004(      const double &CurrentPosition
                        ,   const double &CurrentVelocity
                        ,   const double &TargetPosition
                        ,   const double &TargetVelocity
                        ,   const double &MaxAcceleration
                        ,   const double &CurrentTime
                        ,   const double &SynchronizationTime);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2___005(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxAcceleration, const double &CurrentTime, const double &SynchronizationTime)
//!
//! \brief
//! If v->hold->vtrgt, so that t=tsync, is p<=ptrgt?
//  ----------------------------------------------------------
bool Decision_2___005(      const double &CurrentPosition
                        ,   const double &CurrentVelocity
                        ,   const double &TargetPosition
                        ,   const double &TargetVelocity
                        ,   const double &MaxAcceleration
                        ,   const double &CurrentTime
                        ,   const double &SynchronizationTime);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2___006(const double &CurrentTime, const double &SynchronizationTime, const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxAcceleration)
//!
//! \brief
//! If (v->0->vtrgt, is p<=ptrgt || t > tsync)?
//  ----------------------------------------------------------
bool Decision_2___006(      const double &CurrentTime
                        ,   const double &SynchronizationTime
                        ,   const double &CurrentPosition
                        ,   const double &CurrentVelocity
                        ,   const double &TargetPosition
                        ,   const double &TargetVelocity
                        ,   const double &MaxAcceleration);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2___007(const double &TargetVelocity)
//!
//! \brief
//! Is (vtrgt >= 0)?
//  ----------------------------------------------------------
bool Decision_2___007(      const double &TargetVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2___008(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxAcceleration, const double &CurrentTime, const double &SynchronizationTime)
//!
//! \brief
//! If v->hold->vtrgt, so that t=tsync, is p<=ptrgt?
//  ----------------------------------------------------------
bool Decision_2___008(      const double &CurrentPosition
                        ,   const double &CurrentVelocity
                        ,   const double &TargetPosition
                        ,   const double &TargetVelocity
                        ,   const double &MaxAcceleration
                        ,   const double &CurrentTime
                        ,   const double &SynchronizationTime);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2___009(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxAcceleration, const double &CurrentTime, const double &SynchronizationTime)
//!
//! \brief
//! If v->vtrgt->hold, so that t=tsync, is p<=ptrgt?
//  ----------------------------------------------------------
bool Decision_2___009(      const double &CurrentPosition
                        ,   const double &CurrentVelocity
                        ,   const double &TargetPosition
                        ,   const double &TargetVelocity
                        ,   const double &MaxAcceleration
                        ,   const double &CurrentTime
                        ,   const double &SynchronizationTime);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2___010(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxAcceleration)
//!
//! \brief
//! If v->0->vtrgt, is p>=ptrgt?
//  ----------------------------------------------------------
bool Decision_2___010(      const double &CurrentPosition
                        ,   const double &CurrentVelocity
                        ,   const double &TargetPosition
                        ,   const double &TargetVelocity
                        ,   const double &MaxAcceleration);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2___011(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxAcceleration, const double &CurrentTime, const double &SynchronizationTime)
//!
//! \brief
//! If v->vtrgt->hold, so that t=tsync, is p<=ptrgt?
//  ----------------------------------------------------------
bool Decision_2___011(      const double &CurrentPosition
                        ,   const double &CurrentVelocity
                        ,   const double &TargetPosition
                        ,   const double &TargetVelocity
                        ,   const double &MaxAcceleration
                        ,   const double &CurrentTime
                        ,   const double &SynchronizationTime);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2___012(const double &CurrentPosition, const double &CurrentVelocity, const double &TargetPosition, const double &TargetVelocity, const double &MaxAcceleration, const double &CurrentTime, const double &SynchronizationTime)
//!
//! \brief
//! If v->hold->0->vtrgt, so that t=tsync, is p<=ptrgt?
//  ----------------------------------------------------------
bool Decision_2___012(      const double &CurrentPosition
                        ,   const double &CurrentVelocity
                        ,   const double &TargetPosition
                        ,   const double &TargetVelocity
                        ,   const double &MaxAcceleration
                        ,   const double &CurrentTime
                        ,   const double &SynchronizationTime);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_V___001(const double &CurrentVelocity, const double &TargetVelocity)
//!
//! \brief
//! Is (vi <= vtrgt)?
//  ----------------------------------------------------------
bool Decision_V___001(      const double &CurrentVelocity
                        ,   const double &TargetVelocity);


}   // namespace TypeIIRMLMath

#endif
