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

#ifndef FRYDOM_DICE_MESHCLIPPER_H
#define FRYDOM_DICE_MESHCLIPPER_H

#include <memory>
#include "FrMesh.h"
#include "MathUtils/MathUtils.h"

#include "frydom/core/math/FrVector.h"

namespace frydom {

  //Forward declarations
  class FrFreeSurface;

  namespace geom {
    class FrPlane;
  }

  namespace mesh {

    enum FacePositionType {
      // Position code is composed of 3 digit AOU where A is the number of vertices above the clipping surface,
      // O the number of vertices on and U the number of vertices under.
          FPT_003 = 3,  // TODO : simplifier la representation de cas en enum, on peut certainemet nommer de maniere unique les cas entierement mouille, sec ou a couper
      //  -----------  totally wet
      //       *
      //      / \
            //     /   \
            //    *-----*
          FPT_012 = 12,
      //   ------*------ totally wet
      //        / \
            //       /   \
            //      *-----*
          FPT_021 = 21,
      //  ----*-----*---- totally wet
      //       \   /
      //        \ /
      //         *
          FPT_030 = 30,
      //  ----*----*----*  Lying on the clipping surface, should be removed

      FPT_102 = 102,
      //         *    Face to clip
      //        / \
            //    ---o---o---
      //      /     \
            //     *-------*
          FPT_111 = 111,
      //          *               *    Face to clip
      //         /|               |\
            //        / |               | \
            //    ---*--o---    or   ---o--*---
      //        \ |               | /
      //         \|               |/
      //          *               *
          FPT_120 = 120,
      //          *  totally dry
      //         / \
            //        /   \
            //   ----*-----*----

      FPT_201 = 201,
      //       *-------*  Face to clip
      //        \     /
      //      ---o---o---
      //          \ /
      //           *
          FPT_210 = 210,
      //        *-----*  totally dry
      //         \   /
      //          \ /
      //       ----*----
          FPT_300 = 300,
      //           *  totally dry
      //          / \
            //         /   \
            //        *-----*
      //
      //   ----------------

      FPT_UNDEFINED = -1
    };

    /**
    * \class FrClippingSurface
    * \brief Class for dealing with the clipping incident wave field.
    */
    class FrClippingSurface {

     protected:

      double m_ThresholdDichotomy = 1e-4;     ///< threshold for the dichotomy in the intersection computation

      Position m_bodyPosition = {0., 0., 0.};   ///< horizontal position of the body, related to the mesh to be clipped

     public:

      /// Set the body position in world reference frame, for correction in ClippingWaveSurface::GetDistance
      /// \param bodyPos
      void SetBodyPosition(Position bodyPos);

      /// This function gives the distance to the incident wave field.
      virtual double GetDistance(const FrMesh::Point &point) const = 0;

      /// This function gives the intersection node position between an edge and an incident wave field.
      virtual FrMesh::Point GetIntersection(const FrMesh::Point &p0, const FrMesh::Point &p1) = 0;

    };

    /**
    * \class FrClippingPlane
    * \brief Class used when the clipping incident wave field is a HORIZONTAL plane.
    */
    class FrClippingPlane : public FrClippingSurface {

     private:

      std::shared_ptr<geom::FrPlane> m_plane;     ///< plane used for clipping

     public:

      explicit FrClippingPlane(const std::shared_ptr<geom::FrPlane> &plane);;

      /// This function gives the distance to the plane.
      double GetDistance(const FrMesh::Point &point) const override;

      /// This function gives the intersection node position between an edge and the plane.
      FrMesh::Point GetIntersection(const FrMesh::Point &p0, const FrMesh::Point &p1) override;

      geom::FrPlane *GetPlane() const;

    };

    /**
    * \class FrClippingWavesSurface
    * \brief Class for clipping a mesh by an arbitrary incident wave field.
    */
    class FrClippingWaveSurface : public FrClippingSurface {

     private:

      FrFreeSurface *m_freeSurface;   ///< free surface used for clipping

     public:

      explicit FrClippingWaveSurface(FrFreeSurface *freeSurface) : m_freeSurface(freeSurface) {};

      /// This function gives the distance to the incident wave.
      double GetDistance(const FrMesh::Point &point) const override;

      /// This function performs a bisection method to track the intersection node.
      FrMesh::Point GetIntersection(const FrMesh::Point &p0, const FrMesh::Point &p1) override;
    };

    class FrMeshClipper {

     private:

      /// Initial mesh.
      FrMesh *m_mesh;

      /// Clipping surface, by default the plane z = 0.
      std::shared_ptr<FrClippingSurface> m_clippingSurface = nullptr;

      double m_Threshold = 1e-4;
      double m_ProjectionThresholdRatio = 1 / 4.;

      /// Vector to store the faces which are on and/or above the incident free surface and have to be deleted.
      std::vector<FrMesh::FaceHandle> c_FacesToDelete;

      /// Vector to store the faces which need to be clipped.
      std::vector<FrMesh::FaceHandle *> c_FacesToUpdate;


     public:

      /// This function gives the clipping surface.
      FrClippingSurface *GetClippingSurface();

      /// This function initializes the MeshClipper object from an input mesh and performs the clipping.
      void Apply(FrMesh *mesh);

      /// Set the threshold used for crossing and classifying computations
      /// \param eps threshold
      void SetThreshold(double eps);

      /// Set the threshold used for projection computations
      /// \param projectionThresholdRatio threshold
      void SetProjectionThresholdRatio(double projectionThresholdRatio);

      /// Set the clipping surface to be used
      /// \param clippingSurface clipping surface
      void SetClippingSurface(std::shared_ptr<FrClippingSurface> clippingSurface);

     private:

      /// Initialize the mesh clipper
      void Initialize();

      /// Clear the mesh
      void Clear();

      /// This function classify the vertices wrt the clipping surface.
      void ClassifyVertices();

      /// This function computes the distance wrt the clipping surface and classifies the nodes.
      /// \param vh vertex to be classified
      /// \return vertex position with respect to the clipping surface
      VertexPosition ClassifyVertex(const FrMesh::VertexHandle &vh) const;

      /// This function classfies faces wrt the incident clipping surface.
      /// \param fh face to be classified
      /// \return face position with respect to the clipping surface
      FacePositionType ClassifyFace(const FrMesh::FaceHandle &fh);

      /// Clip the mesh with the given clipping surface
      void Clip();

      void UpdateModifiedFaceProperties(FaceHandle fh);

      void ProcessFace(const FrMesh::FaceHandle &fh);

      bool HasProjection(const FrMesh::FaceHandle &fh);

      void ProcessHalfEdge(FrMesh::HalfedgeHandle heh);

      void FlagFaceToBeDeleted(const FrMesh::FaceHandle &fh);

      void FlagVertexAdjacentFacesToBeDeleted(const FrMesh::VertexHandle &vh);

      bool IsEdgeCrossing(const FrMesh::EdgeHandle &eh);

      bool IsHalfEdgeCrossing(const FrMesh::HalfedgeHandle &heh);

      bool IsHalfEdgeDownCrossing(const FrMesh::HalfedgeHandle &heh);

      bool IsHalfEdgeUpCrossing(const FrMesh::HalfedgeHandle &heh);

      FrMesh::HalfedgeHandle FindUpcrossingHalfEdge(const FrMesh::FaceHandle &fh);

      FrMesh::HalfedgeHandle FindDowncrossingHalfEdge(const FrMesh::FaceHandle &fh);

      FrMesh::VertexHandle InsertIntersectionVertex(const FrMesh::HalfedgeHandle &heh);

      double GetVertexDistanceToSurface(const FrMesh::VertexHandle &vh) const;

      void ApplyFaceDeletion();

      void Finalize();

//            /// This function gives the position in the body frame of a node in the mesh frame.
//            FrMesh::Point GetMeshPointPositionInBody(FrMesh::Point point) const;


    };

  }  // end namespace mesh
}  // end namespace frydom


#endif //FRYDOM_DICE_MESHCLIPPER_H
