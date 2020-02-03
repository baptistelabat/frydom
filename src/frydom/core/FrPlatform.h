//
// Created by frongere on 25/09/19.
//

#ifndef FRYDOM_FRPLATFORM_H
#define FRYDOM_FRPLATFORM_H


#if defined(_WIN32) || defined(_WIN64)

#define WINDOWS_OS

#elif defined(__linux__)

#define LINUX_OS

#endif

std::string GetPlatformName()
{
  #ifdef _WIN32
  return "Windows 32-bit";
  #elif _WIN64
  return "Windows 64-bit";
//  #elif __APPLE__ || __MACH__
//  return "Mac OSX";
  #elif __linux__
  return "Linux";
//  #elif __FreeBSD__
//  return "FreeBSD";
//    #elif __unix || __unix__
//    return "Unix";
//    #else
//    return "Other";
  #endif
}


#endif //FRYDOM_FRPLATFORM_H
