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

#ifndef FRYDOM_FRLOGMANAGER_H
#define FRYDOM_FRLOGMANAGER_H

#include <cppfs/fs.h>
#include <cppfs/FileHandle.h>
#include <cppfs/FilePath.h>

#include "frydom/core/common/FrConvention.h"

namespace frydom {

    //Forward declaration
    class FrObject;
    class FrBody;
    class FrOffshoreSystem;
    class FrForce;
    class FrNode;
    class FrPhysicsItem;
    class FrLinkBase;
    class FrStaticAnalysis;

    class FrPathManager {
    private:

        FRAME_CONVENTION m_logFrameConvention;

        cppfs::FilePath m_outputPath;
        cppfs::FilePath m_runPath;

    public:

        /// Constructor for a log manager service
        explicit FrPathManager();


        /// Set the frame convention for the logs
        /// \param fc frame convention (NED/NWU)
        void SetLogFrameConvention(FRAME_CONVENTION fc);

        /// Get the frame convention for the logs
        /// \return frame convention (NED/NWU)
        FRAME_CONVENTION GetLogFrameConvention() const;

        /// Set the path for the output directory, containing all log files
        /// \param path path for the output directory
        void SetLogOutputPath(std::string path);

        /// Get the path for the output directory, containing all log files
        /// \return path for the output directory
        std::string GetLogOutputPath() const;


        ///Initialize the log manager serice
        void Initialize(FrOffshoreSystem* system);

        /// Build the path and directories needed for the system log
        /// \param system system for which a log is declared
        /// \param relPath relative path, added to the path
        /// \return path to the system log file
        std::string BuildPath(FrOffshoreSystem *system, std::string relPath);

        /// Build the path and directories needed for a physics item log
        /// \param pi physics item for which a log is declared
        /// \param relPath relative path, added to the path
        /// \return path to the physics item log file
        std::string BuildPath(FrPhysicsItem *pi, std::string relPath);

        /// Build the path and directories needed for a link log
        /// \param link link for which a log is declared
        /// \param relPath relative path, added to the path
        /// \return path to the link log file
        std::string BuildPath(FrLinkBase *link, std::string relPath);

        /// Build the path and directories needed for a body log
        /// \param body body for which a log is declared
        /// \param relPath relative path, added to the path
        /// \return path to the body log file
        std::string BuildPath(FrBody *body, std::string relPath);

        /// Build the path and directories needed for a force log
        /// \param force force for which a log is declared
        /// \param relPath relative path, added to the path
        /// \return path to the force log file
        std::string BuildPath(FrForce *force, std::string relPath);

        /// Build the path and directories needed for a node log
        /// \param node node for which a log is declared
        /// \param relPath relative path, added to the path
        /// \return path to the node log file
        std::string BuildPath(FrNode *node, std::string relPath);

        /// Build the path and directories needed for a static analysis log
        /// \param staticAnalysis static analysis for which a log is declared
        /// \param relPath relative path, added to the path
        /// \return path to the static analysis log file
        std::string BuildPath(FrStaticAnalysis* staticAnalysis, std::string relPath);


    private:
        /// Read the config file
        void ReadConfig();
    };

} // end namespace frydom

#endif //FRYDOM_FRLOGMANAGER_H
