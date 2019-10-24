//
// Created by frongere on 02/10/19.
//

#ifndef FRYDOM_FRFILESYSTEM_H
#define FRYDOM_FRFILESYSTEM_H

#include <string>

#include "cppfs/fs.h"
#include "cppfs/FileHandle.h"

// TODO : cette classe merite d'etre placee dans un depot a part !!

namespace frydom {


  class FrFileSystem {

   public:

    /// Create directory in a recursive way (similar to mkdir -p /path/to/dir under Unix but multiplatform)
    static bool mkdir(const std::string &path);

   private:

    static bool mkdir(cppfs::FileHandle &fh);

  };

}  // end namespace frydom

#endif //FRYDOM_FRFILESYSTEM_H
