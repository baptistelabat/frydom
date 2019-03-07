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

#ifndef FRYDOM_FRNODE_H
#define FRYDOM_FRNODE_H


#include "chrono/physics/ChMarker.h"
#include "FrRotation.h"
#include "FrFrame.h"
#include "FrObject.h"

#include "frydom/core/link/links_lib/FrLink.h"


namespace frydom {

    // Forward declarations
    class FrNode;

    namespace internal {

        struct FrMarker : public chrono::ChMarker {

            FrNode * m_frydomNode;

            explicit FrMarker(FrNode* node);

        };

    } // end namespace frydom::internal


    // Forward declarations
    class FrBody;
    class FrNodeAsset;

    /**
     * \class FrNode
     * \brief Class for defining nodes (in order to add links).
     */
    class FrNode : public FrObject {

    private:

        FrBody* m_body;                                    ///< Pointer to the body containing this node
        std::shared_ptr<internal::FrMarker> m_chronoMarker;   ///< Chrono class for nodes/marker.

        // Asset for a node
        bool m_showAsset;
        std::shared_ptr<FrNodeAsset> m_asset;

    public:

        /// Default Constructor
        /// \param body body to which the node belongs
        explicit FrNode(FrBody* body);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "Node"; }

        /// Set if an asset is to be displayed
        /// \param showAsset true if a ForceAsset is to be displayed
        void ShowAsset(bool showAsset);;

        FrNodeAsset* GetAsset();

        /// Set node position and direction axis, with respect to body reference frame
        /// \param pos relative position of the frame node with respect to body reference frame
        /// \param e1 direction of the x-axis with respect to reference frame
        /// \param e2 direction of the y-axis with respect to reference frame
        /// \param e3 direction of the z-axis with respect to reference frame
        /// \param fc frame convention (NED/NWU)
        void Set(const Position& pos, const Direction& e1, const Direction& e2, const Direction& e3, FRAME_CONVENTION fc);


        /*
         * Setters
         */

        /// Set the node position with respect to body reference frame
        void SetPositionInBody(const Position& bodyPosition, FRAME_CONVENTION fc);
        void SetPositionInWorld(const Position& worldPosition, FRAME_CONVENTION fc);

        void TranslateInBody(const Translation &translationInBody, FRAME_CONVENTION fc);
        void TranslateInBody(const Direction& directionBody, double distance, FRAME_CONVENTION fc);
        void TranslateInBody(double x, double y, double z, FRAME_CONVENTION fc);

        void TranslateInWorld(const Translation &translationInWorld, FRAME_CONVENTION fc);
        void TranslateInWorld(const Direction& directionWorld, double distance, FRAME_CONVENTION fc);
        void TranslateInWorld(double x, double y, double z, FRAME_CONVENTION fc);


        void SetOrientationInBody(const FrRotation& rotation);
        void SetOrientationInBody(const FrUnitQuaternion& quaternion);

        void RotateInBody(const FrRotation& rotation);
        void RotateInBody(const FrUnitQuaternion& quaternion);

        void RotateInWorld(const FrRotation& rotation);
        void RotateInWorld(const FrUnitQuaternion& quaternion);

        void RotateAroundXInBody(double angleRad, FRAME_CONVENTION fc);
        void RotateAroundYInBody(double angleRad, FRAME_CONVENTION fc);
        void RotateAroundZInBody(double angleRad, FRAME_CONVENTION fc);

        void RotateAroundXInWorld(double angleRad, FRAME_CONVENTION fc);
        void RotateAroundYInWorld(double angleRad, FRAME_CONVENTION fc);
        void RotateAroundZInWorld(double angleRad, FRAME_CONVENTION fc);



        /// Destructor
        ~FrNode() = default;

        /// Get the body pointer
        /// \return the body to which the node belongs
        FrBody* GetBody();

        /// Get the node frame, given in the world reference frame
        /// \return the node frame in the world reference frame
        FrFrame GetFrameInWorld() const;

        /// Get the node frame, given in the body reference frame
        /// \return the node frame in the body reference frame
        FrFrame GetFrameInBody() const;

        /// Get the node frame with respect to the COG in body reference coordinates
        /// \return the node frame with respect to COG
        FrFrame GetFrameWRT_COG_InBody() const;

        void SetFrameInBody(const FrFrame& frameInBody);

        void SetFrameInWorld(const FrFrame& frameInWorld);


        /// Get the node position in world reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the node position in world reference frame
        Position GetPositionInWorld(FRAME_CONVENTION fc) const;

        /// Get the node position in world reference frame
        /// \param position reference to the node position in world reference frame
        /// \param fc Frame convention (NED/NWU)
        void GetPositionInWorld(Position &position, FRAME_CONVENTION fc);

        /// Get the node position in body reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the node position in body reference frame
        Position GetNodePositionInBody(FRAME_CONVENTION fc) const;

        /// Get the node velocity in world reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the node velocity in world reference frame
        Velocity GetVelocityInWorld(FRAME_CONVENTION fc) const;

        /// Get the node velocity in the node reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the node velocity in the node reference frame
        Velocity GetVelocityInNode(FRAME_CONVENTION fc) const;

        /// Get the node acceleration in world reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the node acceleration in world reference frame
        Acceleration GetAccelerationInWorld(FRAME_CONVENTION fc) const;

        /// Get the node acceleration in the node reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the node acceleration in the node reference frame
        Acceleration GetAccelerationInNode(FRAME_CONVENTION fc) const;

        /// Initialize method not implemented yet
        void Initialize() override;

        /// StepFinalize method not implemented yet
        void StepFinalize() override;

        // Logging

        void InitializeLog();


        // =============================================================================================================
        // PROJECTIONS
        // =============================================================================================================

        // Projection of 3D vectors defined in FrVector.h

        /// Project a vector from node reference frame to world reference frame
        /// \tparam Vector vector template
        /// \param nodeVector vector to be projected, given in the node reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the vector projected in the world reference frame
        template <class Vector>
        Vector ProjectVectorInWorld(const Vector& nodeVector, FRAME_CONVENTION fc) const {
            return GetFrameInWorld().GetQuaternion().Rotate<Vector>(nodeVector, fc);
        }

        /// Project a vector from node reference frame to world reference frame
        /// \tparam Vector vector template
        /// \param nodeVector vector to be projected, given in the node reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the vector projected in the world reference frame
        template <class Vector>
        Vector& ProjectVectorInWorld(Vector& nodeVector, FRAME_CONVENTION fc) const {
            nodeVector = GetFrameInWorld().GetQuaternion().Rotate<Vector>(nodeVector, fc);
            return nodeVector;
        }

        /// Project a vector from world reference frame to node reference frame
        /// \tparam Vector vector template
        /// \param worldVector vector to be projected, given in the world reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the vector projected in the node reference frame
        template <class Vector>
        Vector ProjectVectorInNode(const Vector &worldVector, FRAME_CONVENTION fc) const {
            return GetFrameInWorld().GetQuaternion().GetInverse().Rotate<Vector>(worldVector, fc);
        }

        /// Project a vector from world reference frame to node reference frame
        /// \tparam Vector vector template
        /// \param worldVector vector to be projected, given in the world reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the vector projected in the node reference frame
        template <class Vector>
        Vector& ProjectVectorInNode(Vector& worldVector, FRAME_CONVENTION fc) const {
            worldVector = GetFrameInWorld().GetQuaternion().GetInverse().Rotate<Vector>(worldVector, fc);
            return worldVector;
        }

    private:

        friend void FrLink::SetMarkers(FrNode*, FrNode*);

    };




}  // end namespace frydom


#endif //FRYDOM_FRNODE_H
