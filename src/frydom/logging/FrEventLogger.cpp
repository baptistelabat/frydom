//
// Created by frongere on 28/10/19.
//

#include <regex>

#include "FrEventLogger.h"


namespace frydom {

  namespace event_logger {

    namespace internal {


      SimulationFormatter::SimulationFormatter(FrOffshoreSystem *system) : m_system(system) {}

      void SimulationFormatter::format(const spdlog::details::log_msg &msg, spdlog::memory_buf_t &dest) {

        /*
         * [<simulation_time>] [<log_level>] [<ObjType>] [<obj_name>] <Message>
         */

        // Here we extract object type name and object name from the message


        // Simulation time & log level
        fmt::format_to(dest, "[{}] [{}] ", m_system->GetTime(), spdlog::level::to_string_view(msg.level)); // TODO : compiler

        // TODO: see how we can add color formatting for std output...

        // Object type
        // TODO

        // Object name
        // TODO

        // Message
        spdlog::details::fmt_helper::append_string_view(msg.payload, dest);

        // End of line
        spdlog::details::fmt_helper::append_string_view(spdlog::details::os::default_eol, dest);

      }

      std::unique_ptr<spdlog::formatter> SimulationFormatter::clone() const {
        return spdlog::details::make_unique<SimulationFormatter>(m_system);
      }
    }  // end namespace frydom::event_logger::internal






  } // end namespace frydom::event_logger



}  // end namespace frydom
