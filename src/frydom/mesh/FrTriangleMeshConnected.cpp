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



#include "FrTriangleMeshConnected.h"


namespace frydom{

    void FrTriangleMeshConnected::addVertex(chrono::ChVector<double> vertex) {
        m_vertices.push_back(vertex);
    }

    void FrTriangleMeshConnected::addVertex(std::vector<chrono::ChVector<>> vertices) {
        for (int i = 0; i < vertices.size(); i++) {
            m_vertices.push_back(vertices[i]);
        }
    }

    // FIXME: calculer aussi les normales, les uv_coords... voir DeformableTerrain.cpp ligne 222
    void FrTriangleMeshConnected::addTriangle(chrono::ChVector<int> triangle) {
        m_face_v_indices.push_back(triangle);
    }

    void FrTriangleMeshConnected::addTriangle(std::vector<chrono::ChVector<int>> triangles) {
        for (int i = 0; i < triangles.size(); i++) {
            m_face_v_indices.push_back(triangles[i]);
        }
    }

    unsigned long FrTriangleMeshConnected::GetNbVertices() {
        return m_vertices.size();
    }

    FrTriangleMeshConnected::VertexIterator FrTriangleMeshConnected::vertex_begin() {
        return m_vertices.begin();
    }

    FrTriangleMeshConnected::VertexIterator FrTriangleMeshConnected::vertex_end() {
        return m_vertices.end();
    }

    FrTriangleMeshConnected::VertexConstIterator FrTriangleMeshConnected::vertex_begin() const {
        return m_vertices.cbegin();
    }

    FrTriangleMeshConnected::VertexConstIterator FrTriangleMeshConnected::vertex_end() const {
        return m_vertices.cend();
    }

}  // end namespace frydom