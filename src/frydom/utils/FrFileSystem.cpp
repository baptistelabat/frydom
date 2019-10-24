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
