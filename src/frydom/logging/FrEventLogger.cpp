//
// Created by frongere on 28/10/19.
//

#include "FrEventLogger.h"

namespace frydom {

  namespace event_logger {

    namespace internal {

      SimulationFormatter::SimulationFormatter(FrOffshoreSystem *system) : m_system(system) {}

      void SimulationFormatter::format(const spdlog::details::log_msg &msg, spdlog::memory_buf_t &dest) {
        // [<simulation_time>] [<log_level>] [<ObjType>] [<obj_name>] <Message>

        // TODO: see how we can add color formatting for std output...

        fmt::format_to(dest, "[{}] [{}] ", m_system->GetTime(),
                       spdlog::level::to_string_view(msg.level)); // TODO : compiler
        spdlog::details::fmt_helper::append_string_view(msg.payload, dest);
        spdlog::details::fmt_helper::append_string_view(spdlog::details::os::default_eol, dest);

      }

      std::unique_ptr<spdlog::formatter> SimulationFormatter::clone() const {
        return spdlog::details::make_unique<SimulationFormatter>(m_system);
      }
    }  // end namespace frydom::event_logger::internal


    LOG_LEVEL get_default_log_level() { return INFO; }

    void set_log_level(LOG_LEVEL log_level) {
      switch (log_level) {
        case TRACE:
          spdlog::set_level(spdlog::level::trace);
          spdlog::info("Log level set to {}", "TRACE");
          break;
        case DEBUG:
          spdlog::set_level(spdlog::level::debug);
          spdlog::info("Log level set to {}", "DEBUG");
          break;
        case INFO:
          spdlog::set_level(spdlog::level::info);
          spdlog::info("Log level set to {}", "INFO");
          break;
        case WARN:
          spdlog::set_level(spdlog::level::warn);
          spdlog::info("Log level set to {}", "WARN");
          break;
        case ERROR:
          spdlog::set_level(spdlog::level::err);
          spdlog::info("Log level set to {}", "ERROR");
          break;
        case CRITICAL:
          spdlog::set_level(spdlog::level::critical);
          spdlog::info("Log level set to {}", "CRITICAL");
          break;
        default:
          error("FrEventLogger", "event logger",
                "Unknown log level {}. Set default log level at {}",
                log_level, get_default_log_level());
          set_log_level(get_default_log_level());
      }
    }

    void set_default_log_level() {
      set_log_level(get_default_log_level());
    }

    void init(FrOffshoreSystem *system, const std::string &name, const std::string &event_log_file) {
      // Initializing event logger
      auto file_logger = spdlog::basic_logger_mt(name, event_log_file);
      spdlog::set_default_logger(file_logger);

      spdlog::info("***************** {} *****************", GetFrydomFlavor());
      spdlog::info("Copyright D-ICE Engineering & Ecole Centrale de Nantes", GetFrydomFlavor());

      set_default_log_level();


      // TODO : gerer manuellement le flush a chaque pas de temps... ou permettre de relaxer avec une temporisation
      // de flush interne basee sur le temps de simulation (et pas chrono).
      spdlog::flush_on(spdlog::level::warn);

      file_logger->set_formatter(std::make_unique<internal::SimulationFormatter>(system));

    }

    void flush() {
      spdlog::default_logger()->flush();
    }

    void reset_to_default_logger() {
      spdlog::drop_all();
      auto color_sink = std::make_shared<spdlog::sinks::ansicolor_stdout_sink_mt>();
      auto default_logger_ = std::make_shared<spdlog::logger>("", std::move(color_sink));
      spdlog::set_default_logger(default_logger_);
    }
  } // end namespace frydom::event_logger

}  // end namespace frydom
