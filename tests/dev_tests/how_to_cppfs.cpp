//
// Created by lletourn on 21/02/19.
//

#include <cppfs/fs.h>
#include <cppfs/FileHandle.h>
#include <cppfs/FilePath.h>
#include <cppfs/FileIterator.h>

#include <iostream>
#include <sstream>

#include <fmt/format.h>

using namespace cppfs;
using namespace fmt;

int main(int argc, char* argv[]) {

    // File path manipulation
    print("\n---------------------------\n");
    print("File path manipulation\n");
    FilePath workspacePath = "/home/lletourn/workspace/";

    print("File path: {}\n", workspacePath.toNative());

    // Check if the path is empty ("")
    bool empty = workspacePath.isEmpty();
    print("Empty path : {}\n", empty);

    // Check if the path is an absolute path (e.g., "/test.txt", or "C:/test.txt")
    bool abs = workspacePath.isAbsolute();
    print("Absolute path : {}\n", abs);

    // Check if the path is a relative path (e.g., "data/test.txt")
    bool rel = workspacePath.isRelative();
    print("Relative path : {}\n", rel);

    // Check if path points to a file/directory (e.g., "/path/to/dir")
    // or its content (e.g., "/path/to/dir/").
    bool listContent = workspacePath.pointsToContent();
    print("point to a directory : {}\n", listContent);


    // Read and write in files
    print("\n---------------------------\n");
    print("Read and write in files\n");

    FileHandle currentDir = fs::open(".");
    FileHandle workspaceDir  = fs::open(workspacePath.path());
    FileHandle readmeFile = fs::open("readme.txt");


    //ReadFile
    print("ReadFile gives :{}\n",readmeFile.readFile());

    //WriteFile
    readmeFile.writeFile("Je suis toujours là !");
    print("After writing in it :{}\n",readmeFile.readFile());


    // Write
    auto out = readmeFile.createOutputStream();
    (*out)<<fmt::format("On test des {}\n", "trucs");
    (*out).flush();
//    (*out)<<"On teste d'autres trucs"<<std::endl;

    // Read
    auto in = readmeFile.createInputStream();
    std::stringstream buffer;
    buffer << in->rdbuf();
    print("Qu'est-ce qu'il y a là dedans? {}\n", buffer.str());


    // File handle manipulation
    print("\n---------------------------\n");
    print("File handle manipulation\n");

    // Create directory if it does not yet exist
    if (!workspaceDir.isDirectory()) workspaceDir.createDirectory();

//    print("readmeFile exists : {}\n",readmeFile.exists());

    // Copy file into directory
    readmeFile.copy(workspaceDir);

    // Copy file to another file
    FileHandle dest;
    dest = workspaceDir.open("readme2.txt");
    readmeFile.copy(dest);

    // Rename file
    readmeFile.rename("readme3.txt");

    // Move file into directory
    readmeFile.move(workspaceDir);

//    // Create hard link
//    dest = workspaceDir.open("readme4.txt");
//    readmeFile.createLink(dest);
//
//    // Create symbolic link
//    dest = workspaceDir.open("readme5.txt");
//    readmeFile.createSymbolicLink(dest);


    FilePath path_to_movedReadMe = workspacePath.resolve(FilePath("readme.txt"));

    FileHandle FH_to_movedReadMe = fs::open(path_to_movedReadMe.path());

    FH_to_movedReadMe.move(currentDir);

//    // Delete all files in workspace directory
//    print("List all files in workspace directory :\n");
//    if (workspaceDir.isDirectory())
//    {
//        for (FileIterator it = workspaceDir.begin(); it != workspaceDir.end(); ++it)
//        {
////            std::string path = *it;
//            print("    {}\n", *it);
//            FilePath filePath = workspacePath.resolve(FilePath(*it));
//            FileHandle fileI = fs::open(filePath.path());
//            fileI.remove();
////            print("file exists : {}\n", fileI.exists());
//        }
//    }

    // Delete file
    readmeFile.remove();

    dest.remove();

    // Delete directory (only if empty)
    workspaceDir.removeDirectory();

}