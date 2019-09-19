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

#ifndef FRYDOM_FRASSETOWNER_H
#define FRYDOM_FRASSETOWNER_H

#include <vector>
#include "frydom/core/misc/FrColors.h"
#include <vector>
#include <memory>

namespace chrono{
    class ChPhysicsItem;
}

namespace chrono {
    class ChPhysicsItem;
}  // end namespace chrono

namespace frydom {

    // Forward declarations:
    class FrAsset;
    class FrTriangleMeshConnected;
    class FrBoxShape;
    class FrCylinderShape;
    class FrSphereShape;
    class FrTriangleMeshShape;

    class FrAssetOwner {

    protected:

        using AssetContainer = std::vector<std::shared_ptr<FrAsset>>;
        AssetContainer m_assets;                    ///< Container of the assets added to the body

        using BoxShapeContainer = std::vector<std::shared_ptr<FrBoxShape>>;
        using BoxShapeConstContainer = std::vector<std::shared_ptr<const FrBoxShape>>;
        using CylinderShapeContainer = std::vector<std::shared_ptr<FrCylinderShape>>;
        using CylinderShapeConstContainer = std::vector<std::shared_ptr<const FrCylinderShape>>;
        using SphereShapeContainer = std::vector<std::shared_ptr<FrSphereShape>>;
        using SphereShapeConstContainer = std::vector<std::shared_ptr<const FrSphereShape>>;
        using TriangleMeshShapeContainer = std::vector<std::shared_ptr<FrTriangleMeshShape>>;
        using TriangleMeshShapeConstContainer = std::vector<std::shared_ptr<const FrTriangleMeshShape>>;
        BoxShapeContainer m_boxShapes;
        CylinderShapeContainer m_cylinderShapes;
        SphereShapeContainer m_sphereShapes;
        TriangleMeshShapeContainer m_meshShapes;

        /// Get the internal item, related to chrono::ChPhysicsItem
        /// \return internal item, related to chrono::ChPhysicsItem
        virtual chrono::ChPhysicsItem* GetChronoItem_ptr() const = 0;

    public:

//        /// Gets the simulation time of this object
//        /// \return simulation time of this object
//        double GetTime(); //FIXME : bug in release?

        /// Update the assets
        void UpdateAsset();


        /// Add a box shape to the body with its dimensions defined in absolute coordinates. Dimensions in meters
        /// \param xSize size of the box along the x absolute coordinates
        /// \param ySize size of the box along the y absolute coordinates
        /// \param zSize size of the box along the z absolute coordinates
        void AddBoxShape(double xSize, double ySize, double zSize);  // TODO : definir plutot les dimensions dans le repere local du corps...

        /// Add a cylinder shape to the body with its dimensions defined in ???? Dimensions in meters
        /// \param radius radius of the cylinder shape.
        /// \param height height of the cylinder shape.
        void AddCylinderShape(double radius, double height);  // FIXME : travailler la possibilite de definir un axe... dans le repere local du corps

        /// Add a sphere shape to the body. Dimensions in meters.
        /// \param radius radius of the sphere shape.
        void AddSphereShape(double radius);  // TODO : permettre de definir un centre en coords locales du corps

        /// Add a mesh as an asset for visualization given a WaveFront .obj file name
        /// \param obj_filename filename of the asset to be added
        void AddMeshAsset(std::string obj_filename);

        /// Add a mesh as an asset for visualization given a FrTriangleMeshConnected mesh object
        /// \param mesh mesh of the asset to be added
        void AddMeshAsset(std::shared_ptr<FrTriangleMeshConnected> mesh);

        BoxShapeConstContainer GetBoxShapes() const;
        CylinderShapeConstContainer GetCylinderShapes() const;
        SphereShapeConstContainer GetSphereShapes() const;
        TriangleMeshShapeConstContainer GetMeshAssets() const;

        /// Add an asset for visualization, based on FrAsset, to the asset owner.
        /// Check FrAsset for a list of all its subclasses.
        /// \param asset asset to be added
        void AddAsset(std::shared_ptr<FrAsset> asset);

        void RemoveAssets();

        /// Set the asset color in visualization given a color id
        /// \param colorName color of the asset
        void SetColor(NAMED_COLOR colorName);

        /// Set the asset color in visualization given a FrColor object
        /// \param color color of the asset
        void SetColor(const FrColor& color);

        // Linear iterators on assets
        using AssetIter = AssetContainer::iterator;
        using ConstAssetIter = AssetContainer::const_iterator;

        AssetIter       asset_begin();
        ConstAssetIter  asset_begin() const;

        AssetIter       asset_end();
        ConstAssetIter  asset_end() const;

    };

}// end namespace frydom
#endif //FRYDOM_FRASSETOWNER_H
