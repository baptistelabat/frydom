//
// Created by frongere on 02/10/19.
//

#ifndef FRYDOM_FRFILESYSTEM_H
#define FRYDOM_FRFILESYSTEM_H

#include <string>

#include "cppfs/FileHandle.h"


// TODO : cette classe merite d'etre placee dans un depot a part !!

namespace frydom {


  class FrFileSystem {

   public:

    /// Return a normalized absolutized version of the pathname path.
    static std::string cwd();

    /// Return the home directory
    static std::string get_home();

//    /// Return a normalized absolutized version of the pathname path
//    static std::string abspath(const std::string &path);
//
//    static std::string normpath(const std::string& path);
//
//    /// Return the base name of pathname path. This is the second element of the pair returned by passing
//    /// path to the function split()
//    static std::string basename(const std::string &path);
//
//    /// Return the directory name of pathname path. This is the first element of the pair returned by
//    /// passing path to the function split()
//    static std::string dirname(const std::string &path);

    /// Return True if path refers to an existing path.
    static bool exists(const std::string &path);

    /// Return True if path is an absolute pathname. On Unix, that means it begins with a slash, on Windows
    /// that it begins with a (back)slash after chopping off a potential drive letter.
    static bool isabs(const std::string &path);

    /// Return True if path is an existing regular file
    static bool isfile(const std::string &path);

    /// Return True if path is an existing directory
    static bool isdir(const std::string &path);

    /// Join one or more path components intelligently
    static std::string join(std::vector<std::string> paths);

//    /// Split the pathname path into a pair, (head, tail) where tail is the last pathname component and head is
//    /// everything leading up to that. The tail part will never contain a slash; if path ends in a slash, tail
//    /// will be empty. If there is no slash in path, head will be empty. If path is empty, both head and tail
//    /// are empty. Trailing slashes are stripped from head unless it is the root (one or more slashes only).
//    /// In all cases, join(head, tail) returns a path to the same location as path (but the strings may differ
//    static std::pair<std::string, std::string> split(const std::string &path);
//
//    /// Split the pathname path into a pair (drive, tail) where drive is either a mount point or the empty string.
//    /// On systems which do not use drive specifications, drive will always be the empty string. In all cases,
//    /// drive + tail will be the same as path.
//    static std::pair<std::string, std::string> splitdrive(const std::string &path);
//
//    /// Split the pathname path into a pair (root, ext) such that root + ext == path, and ext is empty or begins
//    /// with a period and contains at most one period. Leading periods on the basename are ignored;
//    /// splitext('.cshrc') returns ('.cshrc', '').
//    static std::pair<std::string, std::string> splitext(const std::string &path);
//
//    /// Copy the file src to the file or directory dst. If dst is a directory, a file with the same basename
//    /// as src is created (or overwritten) in the directory specified. Permission bits are copied. src and dst
//    /// are path names given as strings.
//    static void copy(const std::string &src, const std::string &dst);
//
//    /// Recursively copy an entire directory tree rooted at src. The destination directory, named by dst, must
//    /// not already exist; it will be created as well as missing parent directories.
//    static void copytree(const std::string &src, const std::string &dst);
//
//    /// Delete an entire directory tree; path must point to a directory (but not a symbolic link to a directory).
//    static void rmtree(const std::string &path);
//
//    /// Recursively move a file or directory (src) to another location (dst).
//    ///
//    /// If the destination is an existing directory, then src is moved inside that directory. If the destination
//    ///  already exists but is not a directory, it may be overwritten
//    static void move(const std::string &src, const std::string &dst);

//    /
//    static void rename(const std::string &src, const std::string &dst);

//    static void make_archive_tgz(const std::string &archive_name, const std::string &root_dir);
//
//    static void make_archive_zip(const std::string &archive_name, const std::string &root_dir);

    /// Return the name of the user logged in on the controlling terminal of the process
    static std::string get_login();

    /// Return the current hostname
    static std::string get_hostname();




    /// Create directory in a recursive way (similar to mkdir -p /path/to/dir under Unix but multiplatform)
    static bool mkdir(const std::string &path);


   private:

    static bool mkdir(cppfs::FileHandle &fh);

  };

}  // end namespace frydom

#endif //FRYDOM_FRFILESYSTEM_H
