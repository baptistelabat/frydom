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


#ifndef FRYDOM_FRHYDROMESH_H
#define FRYDOM_FRHYDROMESH_H

#include "frydom/core/common/FrPhysicsItem.h"
#include "frydom/mesh/FrMesh.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/mesh/FrMeshClipper.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"

namespace frydom {

    class FrOffshoreSystem;
    class FrBody;

    /**
     * \class FrHydroMesh
     * \brief Class for managing the meshes used for computing the nonlinear hydrostatic and Froude-Krylov loads.
     */
    class FrHydroMesh : public FrPrePhysicsItem {

    private:

        /// Offshore system.
        FrOffshoreSystem* m_system;

        /// Input mesh file.
        std::string m_meshfilename; // Input mesh file.

        /// Clipped mesh.
        mesh::FrMesh m_clipped_mesh;

        /// Input mesh file.
        mesh::FrMesh m_mesh_init;

        /// Mesh frame offset in the body frame.
        Position m_MeshOffset;

        /// Rotation of the mesh frame compared to the body frame.
        mathutils::Matrix33<double> m_Rotation;

        /// Boolean to know if the mesh is clipped by a wave (True) or a plane (False).
        bool m_WNL_or_NL;

        /// Body.
        std::shared_ptr<FrBody> m_body;

        /// Clipper.
        std::unique_ptr<mesh::MeshClipper> m_clipper;

    public:

        /// Constructor.
        FrHydroMesh(FrOffshoreSystem* system, std::string meshfile, std::shared_ptr<FrBody> body, bool WNL_or_NL){
            m_system = system;
            m_meshfilename = meshfile;

            m_body = body;
            m_WNL_or_NL = WNL_or_NL;

            // Initilization by default.
            m_Rotation.SetIdentity();
            m_MeshOffset = Position(0,0,0);

            // m_clipper.
            m_clipper = std::make_unique<mesh::MeshClipper>();

            // Loading the input mesh file.
            m_mesh_init = mesh::FrMesh(m_meshfilename);

            // Tidal height.
            double TidalHeight = m_system->GetEnvironment()->GetOcean()->GetFreeSurface()->GetTidal()->GetHeight(NWU);

            // Clipping surface.
            if(m_WNL_or_NL == true) { // Incident wave field.

                // Incident free surface.
                FrFreeSurface *FreeSurface = m_system->GetEnvironment()->GetOcean()->GetFreeSurface();

                // Setting the free surface.
                m_clipper->SetWaveClippingSurface(TidalHeight, FreeSurface);
            }
            else{ // Plane.

                // Setting the free surface.
                m_clipper->SetPlaneClippingSurface(TidalHeight);
            }

            // Body.
            m_clipper->SetBody(m_body.get());

            // Position and orientation of the mesh frame compared to the body frame.
            m_clipper->SetMeshOffsetRotation(m_MeshOffset, m_Rotation);
        }

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "HydroMesh"; }

        /// Update nonlinear hydrostatic force.
        /// \param time Current time of the simulation from beginning.
        void Update(double time) override;

        /// Intialize the nonlinear hydrostatic force model.
        void Initialize() override;


        /// This function sets the offset of the mesh frame in the body frame.
        void SetMeshOffsetRotation(const Position Offset, const mathutils::Matrix33<double> Rotation){
            m_MeshOffset = Offset;
            m_Rotation = Rotation;
        };

        /// Initialize the log
        void InitializeLog() override;

        /// This function returns the clipped mesh.
        mesh::FrMesh GetClippedMesh(){
            return m_clipped_mesh;
        }

        mesh::FrMesh GetInitialMesh(){
            return m_mesh_init;
        }

        /// This function returns the center of buoyancy of the clipped mesh in the world frame.
        Position GetCenterOfBuoyancyInBody(FRAME_CONVENTION fc);

    };

    /// This function creates a hydrodynamic mesh for using in the computation of the nonlinear hydrostatic and/or Froude-Krylov loads.
    std::shared_ptr<FrHydroMesh> make_hydro_mesh_nonlinear(FrOffshoreSystem* system, std::shared_ptr<FrBody> body, std::string meshfile);

    /// This function creates a hydrodynamic mesh for using in the computation of the weakly nonlinear hydrostatic and/or Froude-Krylov loads.
    std::shared_ptr<FrHydroMesh> make_hydro_mesh_weakly_nonlinear(FrOffshoreSystem* system, std::shared_ptr<FrBody> body, std::string meshfile);

    }  // end namespace frydom

#endif //FRYDOM_FRHYDROMESH_H