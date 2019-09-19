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

#ifndef RESOURCES_PATH
#define RESOURCES_PATH
#endif

namespace frydom {

    //Forward declaration
    template <typename OffshoreSystemType>
    class FrOffshoreSystem;


    template <typename OffshoreSystemType>
    class FrPathManager {
    private:

        FRAME_CONVENTION m_logFrameConvention; // FIXME : not used at that moment !!

        cppfs::FilePath m_outputPath;
        cppfs::FilePath m_projectPath;
        cppfs::FilePath m_runPath;
        cppfs::FilePath m_resourcesPath;

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

        void SetRunPath(std::string relPath);

        std::string GetRunPath() const;


        ///Initialize the log manager serice
        void Initialize(FrOffshoreSystem<OffshoreSystemType>* system);

        std::string BuildPath(const std::string& rootPath, const std::string& relPath) const;

        std::string BuildPath(const std::string& absPath) const;

        void SetResourcesPath(std::string absPath);

        std::string GetDataPath(const std::string& relPath) const;

    private:
        /// Read the config file
        void ReadConfig();
    };

} // end namespace frydom

#endif //FRYDOM_FRLOGMANAGER_H
