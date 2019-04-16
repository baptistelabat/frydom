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


#include "FrHDF5.h"


namespace frydom {

    FrHDF5Reader::FrHDF5Reader() = default;

    FrHDF5Reader::FrHDF5Reader(const std::string &filename, HDF5_READ_MODE mode)
            : m_filename(filename), m_file(std::make_unique<H5File>(filename, mode)) {}

    FrHDF5Reader::~FrHDF5Reader() {
        m_file->close();
    }

    void FrHDF5Reader::SetFilename(const std::string &filename, HDF5_READ_MODE mode) {
        m_filename = filename;
        m_file.release();
        try {
            m_file = std::make_unique<H5File>(filename, mode);
        } catch (const H5::FileIException& e) {
            // TODO : throw a frydom exception
            std::cout << "   --- ERROR : HDF5 file '" << filename << "' not found.";
            throw (e);
        }
    }

    void FrHDF5Reader::Close() { m_file->close(); }

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> FrHDF5Reader::ReadDoubleArray(std::string h5Path) const {

        DataSet dset = m_file->openDataSet(h5Path);
        DataSpace dspace = dset.getSpace();
        const int ndims = dspace.getSimpleExtentNdims();

        if (ndims > 2) {
            // TODO : throw a frydom exception
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
                // TODO : throw a frydom exception
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

    std::vector<std::vector<double>> FrHDF5Reader::ReadDoubleArraySTD(std::string h5path) const {

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

    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> FrHDF5Reader::ReadIntArray(std::string h5Path) const {

        DataSet dset = m_file->openDataSet(h5Path);
        DataSpace dspace = dset.getSpace();
        const int ndims = dspace.getSimpleExtentNdims();

        if (ndims > 2) {
            // TODO : throw a frydom exception
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
                // TODO : throw a frydom exception
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

    std::vector<std::vector<int>> FrHDF5Reader::ReadIntArraySTD(std::string h5path) const {

        auto mat = ReadDoubleArray(h5path);
        std::vector<std::vector<int>> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
        return vec;

    }

    double FrHDF5Reader::ReadDouble(std::string h5Path) {
        DataSet dset = m_file->openDataSet(h5Path);

        double d[1];
        dset.read(d, PredType::NATIVE_DOUBLE);

        return d[0];
    }

    int FrHDF5Reader::ReadInt(std::string h5Path) {
        DataSet dset = m_file->openDataSet(h5Path);

        int i[1];
        dset.read(i, PredType::NATIVE_INT);

        return i[0];
    }

    hbool_t FrHDF5Reader::ReadBool(std::string h5Path) {
        DataSet dset = m_file->openDataSet(h5Path);

        hbool_t b[1];
        dset.read(b, PredType::NATIVE_HBOOL);

        return b[0];
    }

    std::string FrHDF5Reader::ReadString(std::string h5Path) {

        StrType dtype(0, H5T_VARIABLE);
        DataSpace dspace(H5S_SCALAR);
        DataSet dset = m_file->openDataSet(h5Path);

        std::string str;
        dset.read(str,dtype, dspace);

        return str;
    }

    void FrHDF5Reader::CreateGroup(std::string h5Path) {
        // TODO
    }

    void FrHDF5Reader::CreateDataset(std::string h5Path) {
        // TODO
    }

    bool FrHDF5Reader::GroupExist(const std::string& h5Path) const {
        return H5Lexists(m_file->getId(), h5Path.c_str(), H5P_DEFAULT);
    }

}  // end namespace frydom
