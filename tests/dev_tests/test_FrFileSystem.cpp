//
// Created by frongere on 25/10/19.
//

#include <iostream>

#include "frydom/utils/FrFileSystem.h"


using namespace frydom;

int main() {

  std::cout << "Current working dir: " << FrFileSystem::cwd() << std::endl;

  std::cout << "User login: " << FrFileSystem::get_login() << std::endl;

  std::cout << "Hostname: " << FrFileSystem::get_hostname() << std::endl;


}
