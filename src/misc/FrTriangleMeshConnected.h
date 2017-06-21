//
// Created by frongere on 29/05/17.
//

#ifndef FRYDOM_FRTRIANGLEMESHCONNECTED_H
#define FRYDOM_FRTRIANGLEMESHCONNECTED_H

#include "chrono/core/ChVector.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace frydom {

    class FrTriangleMeshConnected : public chrono::geometry::ChTriangleMeshConnected {

      public:

        ~FrTriangleMeshConnected() {
            std::cout << "Mesh has been destroyed" << std::endl;
        }

        /// Add a vertex to the mesh
        void addVertex(chrono::ChVector<double> vertex);

        /// Add a list of vertex to the mesh
        void addVertex(std::vector<chrono::ChVector<>> vertices);

        /// Add a face to the mesh
        void addTriangle(chrono::ChVector<int> triangle);

        /// Add a list of faces to the mesh
        void addTriangle(std::vector<chrono::ChVector<int>> faces);
    };

} // end namespace frydom

#endif //FRYDOM_FRTRIANGLEMESHCONNECTED_H
