// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================

#ifndef FRYDOM_FRPATHMANAGER_H
#define FRYDOM_FRPATHMANAGER_H

#include <unordered_set>
#include <typeinfo>
#include <iostream>


namespace frydom {

  template<class ParentType>
  class FrTreeNode;

  class FrOffshoreSystem;

  class FrBody;

  class FrLink;

  class FrForce;

  class FrNode;


  // TODO : renommer en TreeManager ??
  class FrPathManager {

   public:
    FrPathManager() = default;

    ~FrPathManager() = default;

    template<class ParentType>
    static std::string GetPath(const FrTreeNode<ParentType> *node) {
      if (!node) return "";

      return GetPath(node->GetParent()) + GetNormalizedPathName(node);
    }

    template<class ParentType>
    static std::string GetPath(const FrTreeNode<ParentType> &node) {
      return GetPath(&node);
    }


    template<class ParentType>
    bool RegisterTreeNode(FrTreeNode<ParentType> *node) {
      auto path = GetPath(node);

      if (RegisterPath(path)) {
        node->SetTreePath(path);
        return true;
      }
      return false;
    }

   private:

    bool RegisterPath(const std::string &path) {
      if (HasPath(path)) return false;

      m_used_paths.insert(path);
      return true;
    }


    bool HasPath(const std::string &path) {
      return (m_used_paths.find(path) != m_used_paths.end());
    }

    /// Gives the normalized path of the node given a hard coded policy concerning the naming scheme.
    template<class ParentType>
    static std::string GetNormalizedPathName(const FrTreeNode<ParentType> *node) {

      std::string path_name_prefix;

      if (dynamic_cast<const FrOffshoreSystem *>(node)) {
        path_name_prefix = "FRYDOM_";

      } else if (dynamic_cast<const FrBody *>(node)) {
        path_name_prefix = "BODY/BODY_";

      } else if (dynamic_cast<const FrForce *>(node)) {
        path_name_prefix = "FORCE/FORCE_";

      } else if (dynamic_cast<const FrNode *>(node)) {
        path_name_prefix = "NODE/NODE_";

      } else if (dynamic_cast<const FrLink *>(node)) {
        path_name_prefix = "LINK/LINK_";

      } else {
        std::cerr << "No known policy for building normalized path name of " << typeid(node).name() << std::endl;
        exit(EXIT_FAILURE);
      }


      return path_name_prefix + node->GetName() + "/";
    }


   private:
    std::unordered_set<std::string> m_used_paths;

  };








//  //Forward declaration  // FIXME : il n'y a aucune raison pour laquelle FrPathManager devrait connaitre quoique ce soit des classes de FRyDoM, seulement une interface (FrLoggable)
////    class FrObject;
////    class FrBody;
//  class FrOffshoreSystem;
////    class FrForce;
////    class FrNode;
////    class FrPhysicsItem;
////    class FrLinkBase;
////    class FrStaticAnalysis;
////    class FrFEAMesh;
//
//  class FrPathManager_ {
//   private:
//
//    FRAME_CONVENTION m_logFrameConvention;
//
//    cppfs::FilePath m_outputPath; // TODO : tous ces path doivent etre geres par le FrLogManager
//    cppfs::FilePath m_projectPath;
//    cppfs::FilePath m_runPath;
//    cppfs::FilePath m_resourcesPath;
//
//   public:
//
//    /// Constructor for a log manager service
//    explicit FrPathManager_();
//
//
//    /// Set the frame convention for the logs
//    /// \param fc frame convention (NED/NWU)
//    void SetLogFrameConvention(FRAME_CONVENTION fc);
//
//    /// Get the frame convention for the logs
//    /// \return frame convention (NED/NWU)
//    FRAME_CONVENTION GetLogFrameConvention() const;
//
//    /// Set the path for the output directory, containing all log files
//    /// \param path path for the output directory
//    void SetLogOutputPath(std::string path);
//
//    /// Get the path for the output directory, containing all log files
//    /// \return path for the output directory
//    std::string GetLogOutputPath() const;
//
//    void SetRunPath(std::string relPath);
//
//    std::string GetRunPath() const;
//
//
//    ///Initialize the log manager serice
//    void Initialize(FrOffshoreSystem *system);
//
//    std::string BuildPath(const std::string &rootPath, const std::string &relPath) const;
//
//    std::string BuildPath(const std::string &absPath) const;
//
//    void SetResourcesPath(std::string absPath);
//
//    std::string GetDataPath(const std::string &relPath) const;
//
//   private:
//    /// Read the config file
//    void ReadConfig();
//  };

} // end namespace frydom

#endif //FRYDOM_FRPATHMANAGER_H
