//
// Created by lletourn on 08/07/19.
//

#include "FrIncrementalMeshWriter.h"
#include "FrMesh.h"

namespace frydom {
    namespace mesh {
        namespace meshutils {


            void FrIncrementalMeshWriter::operator()(const FrMesh &mesh) {
                Write(mesh);
            }

            void FrIncrementalMeshWriter::Write(const FrMesh &mesh) {
                mesh.Write(GetFilename());
                m_counter++;
            }

            void FrIncrementalMeshWriter::SetFileBase(std::string base) {
                m_meshFileBase = base;
            }

            void FrIncrementalMeshWriter::SetFileType(std::string fileType) {
                m_extension = fileType;
            }

            void FrIncrementalMeshWriter::Reinit(int i) {
                m_counter = i;
            }

            void FrIncrementalMeshWriter::Reinit() {
                m_counter = 0;
            }

            std::string FrIncrementalMeshWriter::GetFilename() const {
                return m_meshFileBase + std::to_string(m_counter) + m_extension;
            }

        } // end namespace frydom::mesh::meshutils

    } // end namespace frydom::mesh

} // end namespace frydom