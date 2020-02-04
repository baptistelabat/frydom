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


#ifndef FRYDOM_FRTRIANGLEMESHCONNECTED_H
#define FRYDOM_FRTRIANGLEMESHCONNECTED_H

#include <iostream>

#include "chrono/geometry/ChTriangleMeshConnected.h"


namespace frydom {

  // Forward declaration
  class FrRotation;

  class Direction;

  // FIXME : ne plus reposer sur l'heritage chrono. Proposer une fonction de conversion depuis openmesh vers
  // le format de maillage chrono

  /**
   * \class FrTriangleMeshConnected
   * \brief Class for dealing with connected triangle meshes.
   */
  class FrTriangleMeshConnected : public chrono::geometry::ChTriangleMeshConnected {

   public:

    /// Add a vertex to the mesh
    void addVertex(chrono::ChVector<double> vertex);

    /// Add a list of vertex to the mesh
    void addVertex(std::vector<chrono::ChVector<>> vertices);

    /// Add a face to the mesh
    void addTriangle(chrono::ChVector<int> triangle);

    /// Add a list of faces to the mesh
    void addTriangle(std::vector<chrono::ChVector<int>> faces);

    void Scale(double scalingFactor);

    void Rotate(const FrRotation &rotation);

    void Translate(const Direction &direction);

    unsigned long GetNbVertices();

    using VertexIterator = std::vector<chrono::ChVector<double>>::iterator;
    using VertexConstIterator = std::vector<chrono::ChVector<double>>::const_iterator;

    VertexIterator vertex_begin();

    VertexIterator vertex_end();

    VertexConstIterator vertex_begin() const;

    VertexConstIterator vertex_end() const;

  };

}  // end namespace frydom

#endif //FRYDOM_FRTRIANGLEMESHCONNECTED_H
