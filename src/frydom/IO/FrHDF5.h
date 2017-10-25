//
// Created by frongere on 20/10/17.
//

#ifndef FRYDOM_FRHDF5_H
#define FRYDOM_FRHDF5_H


#include <memory>
#include <iostream>
#include "H5Cpp.h"

#include "frydom/misc/FrEigen.h"


#define R H5F_ACC_RDONLY
#define RW H5F_ACC_RDWR

using namespace H5;
using namespace Eigen;

namespace frydom {
namespace IO {

    class FrHDF5Reader {

        typedef unsigned int MODE;
//        enum MODE {
//            R = H5F_ACC_RDONLY,
//            RW = H5F_ACC_RDWR
//        };

    private:

        MODE m_mode = R;

        std::string m_filename;
        std::unique_ptr<H5File> m_file;

    public:

        FrHDF5Reader() = default;

        explicit FrHDF5Reader(const std::string &filename, MODE mode=R)
                : m_filename(filename), m_file(std::make_unique<H5File>(filename, mode)) {}

        ~FrHDF5Reader() {
            m_file->close();
            std::cout << std::endl << "HDF5 file " << m_filename << " has been properly closed" << std::endl;
        }

        void SetFilename(const std::string& filename, MODE mode=R) {
            m_filename = filename;
            m_file.release();
            m_file = std::make_unique<H5File>(filename, mode);
        }

        void Close() { m_file->close(); }

        Matrix<double, Dynamic, Dynamic> ReadArray(std::string h5Path) const {

            DataSet dset = m_file->openDataSet(h5Path); // TODO: try
            DataSpace dspace = dset.getSpace();
            const int ndims = dspace.getSimpleExtentNdims();

            if (ndims > 2) {
                throw("Too much dimensions"); // TODO: better error msg
            }

            // Essai
            auto dtype = dset.getDataType();

            hsize_t dims[ndims];
            dspace.getSimpleExtentDims(dims);

            hsize_t nb_rows = 0;
            hsize_t nb_cols = 0;
            hsize_t nb_elt  = 0;

            switch (ndims) {
                case 1:
                    nb_elt = dims[0];
                    nb_rows = nb_elt;
                    nb_cols = 1;
                    break;
                case 2:
                    nb_rows = dims[0];
                    nb_cols = dims[1];
                    nb_elt = nb_rows * nb_cols;
                    break;
                default:
                    std::cout << "Cannot read multidimensional array yet..." << std::endl;

            }

            Matrix<double, Dynamic, Dynamic> out(nb_rows, nb_cols);

            auto* buffer = new double[nb_elt];

            dset.read(buffer, dtype);

            for (long irow=0; irow<nb_rows; ++irow) {
                for (long icol=0; icol<nb_cols; ++icol) {
                    out(irow, icol) = buffer[irow*nb_cols + icol];
                }
            }

            delete [] buffer;

            return out;
        }

        double ReadDouble(std::string h5Path) {
            DataSet dset = m_file->openDataSet(h5Path);

            double d[1];
            dset.read(d, PredType::NATIVE_DOUBLE);

            return d[0];
        }

        int ReadInt(std::string h5Path) {
            DataSet dset = m_file->openDataSet(h5Path);

            int i[1];
            dset.read(i, PredType::NATIVE_INT);

            return i[0];
        }

        std::string ReadString(std::string h5Path) {

            StrType dtype(0, H5T_VARIABLE);
            DataSpace dspace(H5S_SCALAR);
            DataSet dset = m_file->openDataSet(h5Path);

            std::string str;

            return str;

        }

        void CreateGroup(std::string h5Path) {}

        void CreateDataset(std::string h5Path) {} // A rendre polymorphe pour les differents types de donnees

//        void SearchGroup(std::string& h5Path) {}



    };

}  // end namesapce IO
}  // end namespace frydom

#endif //FRYDOM_FRHDF5_H
