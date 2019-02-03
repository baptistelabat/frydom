//
// Created by frongere on 20/10/17.
//

#ifndef FRYDOM_FRHDF5_H
#define FRYDOM_FRHDF5_H


#include <memory>
#include <iostream>
#include "H5Cpp.h"

#include "frydom/utils/FrEigen.h"


//#define R H5F_ACC_RDONLY
//#define RW H5F_ACC_RDWR

using namespace H5;

namespace frydom {

    enum HDF5_READ_MODE {
        READ,
        READWRITE
    };

    class FrHDF5Reader {

    private:

        HDF5_READ_MODE m_mode = READ;

        std::string m_filename;
        std::unique_ptr<H5File> m_file;

    public:

        FrHDF5Reader();

        explicit FrHDF5Reader(const std::string &filename, HDF5_READ_MODE mode=READ);

        /// Destructor that closes properly the HDF5 file
        ~FrHDF5Reader();

        /// Set the file to read/write
        void SetFilename(const std::string& filename, HDF5_READ_MODE mode=READ);

        /// Close the HDF5 file properly
        void Close();

        /// Read a double 2x2 array as an eigen matrix from HDF5 path
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> ReadDoubleArray(std::string h5Path) const;

        /// Read a double 2x2 array as a std::vector<std::vector<double>> from HDF5 path
        std::vector<std::vector<double>> ReadDoubleArraySTD(std::string h5path) const;

        /// Read a int 2x2 array as an eigen matrix from HDF5 path
        Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> ReadIntArray(std::string h5Path) const;

        /// Read an int 2x2 array as a std::vector<std::vector<double>> from HDF5 path
        std::vector<std::vector<int>> ReadIntArraySTD(std::string h5path) const;

        /// Read a double value from HDF5 path
        double ReadDouble(std::string h5Path);

        /// Read an int value from HDF5 path
        int ReadInt(std::string h5Path);

        /// Read a boolean value from HDF5 path
        hbool_t ReadBool(std::string h5Path);

        /// Read a string value from HDF5 path
        std::string ReadString(std::string h5Path);

        /// Create a group in the HDF5 file given a HDF5 path
        void CreateGroup(std::string h5Path);

        /// Create a dataset in the HDF5 file given a HDF5 path
        void CreateDataset(std::string h5Path); // A rendre polymorphe pour les differents types de donnees

        /// Checks if a group exists in the HDF5 file given a HDF5 path
        bool GroupExist(const std::string& h5Path) const;

    };

}  // end namespace frydom

#endif //FRYDOM_FRHDF5_H
