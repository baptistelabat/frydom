//
// Created by frongere on 20/10/17.
//

#ifndef FRYDOM_FRHDF5_H
#define FRYDOM_FRHDF5_H


#include <memory>
#include <iostream>
#include "H5Cpp.h"

#include "frydom/utils/FrEigen.h"


#define R H5F_ACC_RDONLY
#define RW H5F_ACC_RDWR

using namespace H5;

namespace frydom {

    class FrHDF5Reader {

        typedef unsigned int MODE;

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
//            std::cout << std::endl << "HDF5 file " << m_filename << " has been properly closed" << std::endl;
        }

        void SetFilename(const std::string& filename, MODE mode=R) {
            m_filename = filename;
            m_file.release();
            try {
            m_file = std::make_unique<H5File>(filename, mode);
            } catch (const H5::FileIException& e) {
                std::cout << "   --- ERROR : HDF5 file '" << filename << "' not found.";
                throw (e);
            }
        }

        void Close() { m_file->close(); }

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> ReadDoubleArray(std::string h5Path) const {

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

            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> out(nb_rows, nb_cols);

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

        std::vector<std::vector<double>> ReadDoubleArraySTD(std::string h5path) const {

            auto mat = ReadDoubleArray(h5path);

            std::vector<double> vec;
            std::vector<std::vector<double>> res;

            for (auto i=0; i<mat.cols(); i++) {
                for (auto j=0; j<mat.rows(); j++) {
                    vec.push_back(mat(j,i));
                }
                res.push_back(vec);
            }

            return res;

        }

        Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> ReadIntArray(std::string h5Path) const {

            DataSet dset = m_file->openDataSet(h5Path);
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

            Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> out(nb_rows, nb_cols);

            auto* buffer = new int[nb_elt];

            dset.read(buffer, PredType::NATIVE_INT);

            hsize_t ielt;
            int val;
            for (long irow=0; irow<nb_rows; ++irow) {
                for (long icol=0; icol<nb_cols; ++icol) {
                    out(irow, icol) = buffer[irow*nb_cols + icol];
                }
            }

            delete [] buffer;

            return out;
        }

        std::vector<std::vector<int>> ReadIntArraySTD(std::string h5path) const {

            auto mat = ReadDoubleArray(h5path);
            std::vector<std::vector<int>> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
            return vec;

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

        hbool_t ReadBool(std::string h5Path) {
            DataSet dset = m_file->openDataSet(h5Path);

            hbool_t b[1];
            dset.read(b, PredType::NATIVE_HBOOL);

            return b[0];
        }

        std::string ReadString(std::string h5Path) {

            StrType dtype(0, H5T_VARIABLE);
            DataSpace dspace(H5S_SCALAR);
            DataSet dset = m_file->openDataSet(h5Path);

            std::string str;
            dset.read(str,dtype, dspace);

            return str;
        }

        void CreateGroup(std::string h5Path) {}

        void CreateDataset(std::string h5Path) {} // A rendre polymorphe pour les differents types de donnees

//        void SearchGroup(std::string& h5Path) {}



    };

}  // end namespace frydom

#endif //FRYDOM_FRHDF5_H
