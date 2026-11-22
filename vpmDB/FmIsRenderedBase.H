// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmIsRenderedBase.H
  \brief Base class for all objects which can be visualized in the 3D view.
*/

#ifndef FM_IS_RENDERED_BASE_H
#define FM_IS_RENDERED_BASE_H

#include "vpmDB/FmIsMeasuredBase.H"
#ifdef USE_INVENTOR
#include "vpmDisplay/FaDOF.H"
#else
//! \brief Empty dummy class used when built without visualization interface.
class FaDOF {};
#endif

class FdObject;
class FmTriad;


/*!
  \brief Base class for all objects which can be visualized in the 3D view.
*/

class FmIsRenderedBase : public FmIsMeasuredBase
{
  //! \cond DO_NOT_DOCUMENT
  Fmd_DB_HEADER_INIT();
  //! \endcond

public:
#ifdef USE_INVENTOR
  //! \brief Returns the accompanying visualization object for this object.
  FdObject* getFdPointer() { return itsDisplayPt; }
#endif

  //! \brief Updates the rendering of this object in the 3D view.
  bool draw(bool updateChildren = true) final;

  //! \brief Highlights this object in the 3D view.
  virtual bool highlight(bool trueOrFalse);
  //! \brief Detaches this object from its connections.
  virtual bool detach() { return false; }

  // From FmSimulationModelBase
  bool isListable() const override { return true; }

  //! \brief Updates the topology rendering for this object.
  void updateTopologyInViewer(bool updateChildren = true);
  //! \brief Updates the topology rendering for this object.
  void updateDisplayTopology();
  //! \brief Updates the detailed rendering for this object.
  void updateDisplayDetails();
  //! \brief Updates the coordinate system rendering for this object.
  void updateDisplayCS();

  //! \brief Returns the movement freedom of this object.
  virtual FaDOF getObjDegOfFreedom() const { return FaDOF(); }

protected:
  //! \brief Default constructor.
  FmIsRenderedBase();
  //! \brief The destructor deletes the visualization model.
  virtual ~FmIsRenderedBase();

  //! \brief Deleted copy constructor.
  FmIsRenderedBase(const FmIsRenderedBase&) = delete;
  //! \brief Deleted assignment operator.
  FmIsRenderedBase& operator=(const FmIsRenderedBase&) = delete;

  // From FmBase
  bool cloneLocal(FmBase* obj, int depth) override;

#ifdef USE_INVENTOR
  FdObject* itsDisplayPt; //!< Visualization model associated with this object
#endif

public:
  //! Used when detaching axial springs and dampers,
  //! see FmAxialSpring::detach() and FmAxialDamper::detach()
  static FmTriad* detachTriad;
};

#endif
