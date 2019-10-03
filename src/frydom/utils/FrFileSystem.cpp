//
// Created by frongere on 02/10/19.
//

#include <iostream>

#include "FrFileSystem.h"


namespace frydom {


  bool FrFileSystem::mkdir(const std::string &path) {
    cppfs::FileHandle fh = cppfs::fs::open(path);
    return mkdir(fh);
  }

  bool FrFileSystem::mkdir(cppfs::FileHandle &fh) {

    if (fh.exists()) return true;

    cppfs::FileHandle parent_dir = fh.parentDirectory();
    if (mkdir(parent_dir)) {
      if (!fh.createDirectory()) {
        std::cerr << "Cannot create directory \"" << fh.path() << "\". Permission denied." << std::endl;
        exit(EXIT_FAILURE);
      }
      return true;
    }
    return false;
  }
}  // end namespace frydom
