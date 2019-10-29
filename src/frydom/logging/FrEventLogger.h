//
// Created by frongere on 28/10/19.
//

#ifndef FRYDOM_FREVENTLOGGER_H
#define FRYDOM_FREVENTLOGGER_H


#include <spdlog/spdlog.h>
#include <spdlog/details/fmt_helper.h>
#include <spdlog/sinks/basic_file_sink.h>


#include <frydom/utils/FrFileSystem.h>
#include <iostream>


#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/logging/FrLoggable.h"


namespace frydom {

  namespace event_logger {


    enum LOG_LEVEL {
      TRACE,
      DEBUG,
      INFO,
      WARN,
      ERROR,
      CRITICAL
    };


    using string_view = spdlog::string_view_t;
    using memory_buffer = spdlog::memory_buf_t;

    namespace internal {

      class SimulationFormatter : public spdlog::formatter {

       public:

        explicit SimulationFormatter(FrOffshoreSystem *system);

        void format(const spdlog::details::log_msg &msg, spdlog::memory_buf_t &dest) override;

        std::unique_ptr<spdlog::formatter> clone() const override;

       private:
        FrOffshoreSystem *m_system;

      };

      template<typename T>
      inline string_view PreFormat(memory_buffer &buffer,
                                   const std::string &objType, const std::string &objName,
                                   const T &msg) {

        fmt::format_to(buffer, "[{}] [{}] {}", objType, objName, msg);
        return spdlog::details::fmt_helper::to_string_view(buffer);
      }

//      template<typename... Args>
//      inline string_view PreFormat(memory_buffer &buffer,
//                            const std::string &objType, const std::string &objName,
//                            string_view fmt, const Args &... args) {
//
//        fmt::format_to(buffer, "[{}] [{}] {}", objType, objName, fmt);
//        return spdlog::details::fmt_helper::to_string_view(buffer);
//      }


    }  // end namespace frydom::event_logger::internal




    // TODO : voir comment ajouter les infos de type d'objet et de nom d'objet avant de deployer partout !!!

    template<typename T>
    inline void debug(const std::string &objType, const std::string &objName, const T &msg) {
      memory_buffer buffer;
      spdlog::debug(internal::PreFormat(buffer, objType, objName, msg));
    }

    template<typename T>
    inline void info(const std::string &objType, const std::string &objName, const T &msg) {
      memory_buffer buffer;
      spdlog::info(internal::PreFormat(buffer, objType, objName, msg));
    }

    template<typename T>
    inline void warn(const std::string &objType, const std::string &objName, const T &msg) {
      memory_buffer buffer;
      spdlog::warn(internal::PreFormat(buffer, objType, objName, msg));
    }

    template<typename T>
    inline void error(const std::string &objType, const std::string &objName, const T &msg) {
      memory_buffer buffer;
      spdlog::error(internal::PreFormat(buffer, objType, objName, msg));
    }

    template<typename T>
    inline void critical(const std::string &objType, const std::string &objName, const T &msg) {
      memory_buffer buffer;
      spdlog::critical(internal::PreFormat(buffer, objType, objName, msg));
    }


    template<typename... Args>
    inline void debug(const std::string &objType, const std::string &objName, string_view fmt,
                      const Args &... args) {
      memory_buffer buffer;
      spdlog::debug(internal::PreFormat(buffer, objType, objName, fmt), args...);
    }

    template<typename... Args>
    inline void info(const std::string &objType, const std::string &objName, string_view fmt,
                     const Args &... args) {
      memory_buffer buffer;
      spdlog::info(internal::PreFormat(buffer, objType, objName, fmt), args...);
    }

    template<typename... Args>
    inline void warn(const std::string &objType, const std::string &objName, string_view fmt,
                     const Args &... args) {
      memory_buffer buffer;
      spdlog::warn(internal::PreFormat(buffer, objType, objName, fmt), args...);
    }

    template<typename... Args>
    inline void error(const std::string &objType, const std::string &objName, string_view fmt,
                      const Args &... args) {
      memory_buffer buffer;
      spdlog::error(internal::PreFormat(buffer, objType, objName, fmt), args...);
    }

    template<typename... Args>
    inline void critical(const std::string &objType, const std::string &objName, string_view fmt,
                         const Args &... args) {
      memory_buffer buffer;
      spdlog::critical(internal::PreFormat(buffer, objType, objName, fmt), args...);
    }

    static LOG_LEVEL get_default_log_level() { return INFO; }

    static void set_log_level(LOG_LEVEL log_level) {
      switch (log_level) {
        case TRACE:
          spdlog::set_level(spdlog::level::trace);
          break;
        case DEBUG:
          spdlog::set_level(spdlog::level::debug);
          break;
        case INFO:
          spdlog::set_level(spdlog::level::info);
          break;
        case WARN:
          spdlog::set_level(spdlog::level::warn);
          break;
        case ERROR:
          spdlog::set_level(spdlog::level::err);
          break;
        case CRITICAL:
          spdlog::set_level(spdlog::level::critical);
          break;
        default:
          error("FrEventLogger", "event logger", "Unknown log level {}. Set default log level at {}",
                log_level, get_default_log_level());
          set_log_level(get_default_log_level());
      }
    }

    static void set_default_log_level() {
      set_log_level(get_default_log_level());
    }

    static void init(FrOffshoreSystem *system, const std::string &name, const std::string &event_log_file) {
      // Initializing event logger
      auto file_logger = spdlog::basic_logger_mt(name, event_log_file);
      spdlog::set_default_logger(file_logger);

      set_default_log_level();


      // TODO : gerer manuellement le flush a chaque pas de temps... ou permettre de relaxer avec une temporisation
      // de flush interne basee sur le temps de simulation (et pas chrono).
      spdlog::flush_every(std::chrono::seconds(1));

      file_logger->set_formatter(std::make_unique<internal::SimulationFormatter>(system));

    }

  }  // end namespace frydom::event_logger

}  // end namespace frydom



#endif //FRYDOM_FREVENTLOGGER_H
