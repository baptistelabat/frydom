//
// Created by frongere on 20/10/17.
//

#include <iostream>
//#include "H5Cpp.h"

#include "frydom/IO/FrHDF5.h"

//using namespace H5;

using namespace frydom::IO;

int main(int argc, char* argv[]) {

    std::string filename = "../src/frydom/tests/data/sphere.h5";

//    FrHDF5Reader reader(filename);

    FrHDF5Reader reader;
    reader.SetFilename(filename);

    std::string inf_freq_path("/body1/hydro_coeffs/added_mass/inf_freq");
    std::cout << reader.ReadArray(inf_freq_path);

//    auto reade = frydom::IO::FrHDF5Reader()

//    H5File file("../src/frydom/tests/data/sphere.h5", H5F_ACC_RDONLY);
//
//    auto inf_freq_dset = file.openDataSet("/body1/hydro_coeffs/added_mass/inf_freq");
//    DataSpace dspace = inf_freq_dset.getSpace();
//    int ndims = dspace.getSimpleExtentNdims();
//
//    hsize_t dims[ndims];
//    dspace.getSimpleExtentDims(dims);
//    hsize_t nb_rows = dims[0];
//    hsize_t nb_cols = dims[1];
//
//    auto nb_elt = nb_rows * nb_cols;
//
//    double* buffer = new double [nb_elt];
//    inf_freq_dset.read(buffer, PredType::NATIVE_DOUBLE);
//
//    for (int i=0; i<nb_elt; ++i) {
//        std::cout << buffer[i] << std::endl;
//    }
//
//    delete[] buffer;
//
//    file.close();


    return 0;
}