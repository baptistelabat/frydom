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

        FrHDF5Reader();

        explicit FrHDF5Reader(const std::string &filename, MODE mode=R);

        ~FrHDF5Reader();

        void SetFilename(const std::string& filename, MODE mode=R);

        void Close();

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> ReadDoubleArray(std::string h5Path) const;

        std::vector<std::vector<double>> ReadDoubleArraySTD(std::string h5path) const;

        Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> ReadIntArray(std::string h5Path) const;

        std::vector<std::vector<int>> ReadIntArraySTD(std::string h5path) const;

        double ReadDouble(std::string h5Path);

        int ReadInt(std::string h5Path);

        hbool_t ReadBool(std::string h5Path);

        std::string ReadString(std::string h5Path);

        void CreateGroup(std::string h5Path);

        void CreateDataset(std::string h5Path); // A rendre polymorphe pour les differents types de donnees

        bool GroupExist(std::string h5Path) const;

//        void SearchGroup(std::string& h5Path) {}



    };

}  // end namespace frydom

#endif //FRYDOM_FRHDF5_H
