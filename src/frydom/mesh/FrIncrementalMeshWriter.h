//
// Created by lletourn on 08/07/19.
//

#ifndef FRYDOM_FRINCREMENTALMESHWRITER_H
#define FRYDOM_FRINCREMENTALMESHWRITER_H

#include <iostream>

namespace frydom {

    namespace mesh {

        // Forward declaration
        class FrMesh;

        namespace meshutils {

            class FrIncrementalMeshWriter {

            private:
                std::string m_meshFileBase = "output";
                std::string m_extension = ".obj";
                int m_counter = 0;

            public:
                FrIncrementalMeshWriter() = default;

                void SetFileBase(std::string base);

                void SetFileType(std::string fileType);

                void Reinit(int i);

                void Reinit();

                void operator()(const FrMesh &mesh);

                void Write(const FrMesh &mesh);

            private:
                std::string GetFilename() const;

            };

        } // end namespace frydom::mesh::meshutils

    } // end namespace frydom::mesh

} // end namespace frydom


#endif //FRYDOM_FRINCREMENTALMESHWRITER_H
