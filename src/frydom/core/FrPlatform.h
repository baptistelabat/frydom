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


#endif //FRYDOM_FRPLATFORM_H
