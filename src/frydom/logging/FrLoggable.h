//
// Created by frongere on 24/09/19.
//

#ifndef FRYDOM_FRLOGGABLE_H
#define FRYDOM_FRLOGGABLE_H


#include <string>
#include <memory>

#include "hermes/hermes.h"
#include "frydom/core/common/FrTreeNode.h"

namespace frydom {


    class FrPathManager;

    template<class ParentType>
    class FrLoggable : public FrTreeNode<ParentType> {

     public:

      explicit FrLoggable(const std::string &name);

      const std::string &GetName() const;

//    /// Check if the object is logged
//    /// \return true if the object is logged
//    bool IsLogged();

//    /// Set the object to be logged or not
//    /// \param isLogged true if the object is to be logged
//    void SetLogged(bool isLogged);

      /// Initialize the logging of the object : build the path, create the directory, add the fields to be logged, etc.
      /// \param path path of the parent object, to build the path of the present object
      void InitializeLog(const std::string &path);

      /// Initialize the logging of the dependencies (attributes of the present object)
      /// \param path path of the present object, to give to the InitializeLog of the dependencies to build their log path
      virtual void InitializeLog_Dependencies(const std::string &path) {};

//    /// Set the pointer to the path manager service, in charge of building the path of every object to be logged
//    /// \param manager shared pointer to the path manager service
//    void SetPathManager(const std::shared_ptr <FrPathManager> &manager);
//
//    /// Get the shared pointer to the path manager service
//    /// \return shared pointer to the path manager service
//    std::shared_ptr <FrPathManager> GetPathManager() const;
//
//    /// Get the frame convention used in the logging
//    /// \return Frame convention used in logging (NED/NWU)
//    FRAME_CONVENTION GetLogFrameConvention() const;

      /// Clear the Hermes message, from all fields and serializer
      void ClearMessage();

     protected:

      /// Serialize and send the message
      void SendLog();

      /// Build the path to the log file, create the directory and add a csv serializer
      /// \param rootPath path of the parent directory
      /// \return path of the present log directory
      virtual std::string BuildPath(const std::string &rootPath);

      /// Add the fields to the Hermes message
      virtual void AddFields() {};


     protected:
//    bool m_is_logged;

      std::unique_ptr<hermes::Message> m_message;     ///< Hermes message, containing the fields to be logged

//    std::shared_ptr<FrPathManager> m_pathManager;   ///< pointer to the path manager, in charge of building the path
//                                                    ///< to the log file of this object
     protected:
      const std::string &m_name;

    };

}  // end namespace frydom


#include "FrLoggable.cpp"

#endif //FRYDOM_FRLOGGABLE_H
