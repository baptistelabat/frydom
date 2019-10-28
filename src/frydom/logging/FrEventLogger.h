//
// Created by frongere on 28/10/19.
//

#ifndef FRYDOM_FREVENTLOGGER_H
#define FRYDOM_FREVENTLOGGER_H


#include <spdlog/spdlog.h>
#include <spdlog/common.h>
#include <spdlog/fmt/fmt.h>
#include <spdlog/details/fmt_helper.h>
#include <spdlog/sinks/basic_file_sink.h>


#include <frydom/utils/FrFileSystem.h>
#include <iostream>


#include "frydom/core/FrOffshoreSystem.h"

namespace frydom {


  namespace internal {

    class SimulationFormatter : public spdlog::formatter {

     public:

      explicit SimulationFormatter(FrOffshoreSystem* system) : m_system(system) {}

      void format(const spdlog::details::log_msg& msg, spdlog::memory_buf_t &dest) override {

        /*
         * [<simulation_time>] [<log_level>] [<ObjType>] [<obj_name>] <Message>
         */

        // Simulation time
        fmt::format_to(dest, "[{}] ", m_system->GetTime());

        // TODO: see how we can add color formatting for std output...

        // Log level
        fmt::format_to(dest, "[{}] ", spdlog::level::to_string_view(msg.level));

        // Object type
        // TODO

        // Object name
        // TODO

        // Message
        spdlog::details::fmt_helper::append_string_view(msg.payload, dest);

        // End of line
        spdlog::details::fmt_helper::append_string_view(spdlog::details::os::default_eol, dest);

      }

      std::unique_ptr<spdlog::formatter> clone() const override {
        return spdlog::details::make_unique<SimulationFormatter>(m_system);
      }

     private:
      FrOffshoreSystem* m_system;

    };

  }


  class FrEventLogger {

   public:

    using string_view = spdlog::string_view_t;

    static void init(FrOffshoreSystem* system, const std::string &name, const std::string &log_file) {
      // Initializing event logger
      auto file_logger = spdlog::basic_logger_mt(name, log_file);
      spdlog::set_default_logger(file_logger);

      // TODO: permettre de faire ce set...
      file_logger->set_level(spdlog::level::debug);

      spdlog::flush_on(spdlog::level::info); // TODO : faire un flush manuel...
      spdlog::flush_every(std::chrono::seconds(1));


      file_logger->set_formatter(std::make_unique<internal::SimulationFormatter>(system));


//      spdlog::info("Welcome to FRyDoM!");
//      spdlog::error("ey, une erreur !!!");


// trace = SPDLOG_LEVEL_TRACE,
//    debug = SPDLOG_LEVEL_DEBUG,
//    info = SPDLOG_LEVEL_INFO,
//    warn = SPDLOG_LEVEL_WARN,
//    err = SPDLOG_LEVEL_ERROR,
//    critical = SPDLOG_LEVEL_CRITICAL,
//    off = SPDLOG_LEVEL_OFF,


      info("coucou {}", FrFileSystem::get_login());
      info("coucou");
      debug(10.2);
      error("Mother fucker {}", FrFileSystem::get_home());

    }


    template <typename T>
    static inline void debug(const T &msg) { spdlog::debug(msg); }

    template <typename T>
    static inline void info(const T &msg) { spdlog::info(msg); }

    template <typename T>
    static inline void warn(const T &msg) { spdlog::warn(msg); }

    template <typename T>
    static inline void error(const T &msg) { spdlog::error(msg); }

    template <typename T>
    static inline void critical(const T &msg) { spdlog::critical(msg); }



    template<typename... Args>
    static inline void debug(string_view fmt, const Args &... args) { spdlog::debug(fmt, args...); }

    template<typename... Args>
    static inline void info(string_view fmt, const Args &... args) { spdlog::info(fmt, args...); }

    template<typename... Args>
    static inline void warn(string_view fmt, const Args &... args) { spdlog::warn(fmt, args...); }

    template<typename... Args>
    static inline void error(string_view fmt, const Args &... args) { spdlog::error(fmt, args...); }

    template<typename... Args>
    static inline void critical(string_view fmt, const Args &... args) { spdlog::critical(fmt, args...); }





  };


}  // end namespace frydom



#endif //FRYDOM_FREVENTLOGGER_H
