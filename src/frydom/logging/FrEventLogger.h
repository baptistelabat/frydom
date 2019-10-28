//
// Created by frongere on 28/10/19.
//

#ifndef FRYDOM_FREVENTLOGGER_H
#define FRYDOM_FREVENTLOGGER_H


#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>


#include <frydom/utils/FrFileSystem.h>


namespace frydom {


  class FrEventLogger {

   public:


    static void init(const std::string &name, const std::string &log_file) {
      // Initializing event logger
      auto file_logger = spdlog::basic_logger_mt(name, log_file);
      spdlog::set_default_logger(file_logger);
      spdlog::flush_on(spdlog::level::info);


      spdlog::info("Welcome to FRyDoM!");
      spdlog::error("ey, une erreur !!!");

      spdlog::info("coucou", FrFileSystem::get_login());



    }


  };


}  // end namespace frydom



#endif //FRYDOM_FREVENTLOGGER_H
