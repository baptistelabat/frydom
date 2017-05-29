//
// Created by frongere on 29/05/17.
//

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

    void FrTriangleMeshConnected::addTriangle(chrono::ChVector<int> triangle) {
        m_face_v_indices.push_back(triangle);
    }

    void FrTriangleMeshConnected::addTriangle(std::vector<chrono::ChVector<int>> triangles) {
        for (int i = 0; i < triangles.size(); i++) {
            m_face_v_indices.push_back(triangles[i]);
        }
    }
}