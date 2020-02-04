// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#ifndef FRYDOM_FRIRRAPP_H
#define FRYDOM_FRIRRAPP_H

#include "chrono_irrlicht/ChIrrApp.h"
#include "frydom/core/common/FrTreeNode.h"


namespace frydom {

  // Forward declaration
  class FrOffshoreSystem;

  class FrIrrCamera;


  /**
   * \class FrIrrApp
   * \brief Class for Irrlicht applications (visualization).
   */
  class FrIrrApp : public chrono::irrlicht::ChIrrApp {

   public:

    explicit FrIrrApp(FrOffshoreSystem *frSystem, chrono::ChSystem *system, double dist = 100);

    ~FrIrrApp() final;

    /// Create a skybox that has Z pointing up.
    /// Note that the default ChIrrApp::AddTypicalSky() uses Y up.
    void SetSkyBox();

    FrIrrCamera *AddCustomCamera(irr::core::vector3df mpos = irr::core::vector3df(0, 0, -8),
                                 irr::core::vector3df mtarg = irr::core::vector3df(0, 0, 0));

    void AddCustomLights(irr::core::vector3df pos1 = irr::core::vector3df(-100.f, -30.f, 30.f),
                         irr::core::vector3df pos2 = irr::core::vector3df(80.f, 30.f, -30.f),
                         double rad1 = 290,
                         double rad2 = 190,
                         irr::video::SColorf col1 = irr::video::SColorf(0.7f, 0.7f, 0.7f, 1.0f),
                         irr::video::SColorf col2 = irr::video::SColorf(0.7f, 0.8f, 0.8f, 1.0f));

    void Run(double endTime);

    void Visualize();

    void VisualizeStaticAnalysis();

   private:

    FrOffshoreSystem *GetSystem() const;

    FrOffshoreSystem *m_system;

  };

} // end namespace frydom

#endif //FRYDOM_FRIRRAPP_H
