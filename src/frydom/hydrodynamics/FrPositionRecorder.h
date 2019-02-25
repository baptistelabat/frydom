//// ==========================================================================
//// FRyDoM - frydom-ce.org
////
//// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
//// All rights reserved.
////
//// Use of this source code is governed by a GPLv3 license that can be found
//// in the LICENSE file of FRyDoM.
////
//// ==========================================================================
//
//
//#ifndef FRYDOM_FRPOSITIONRECORDER_H
//#define FRYDOM_FRPOSITIONRECORDER_H
//
//#include "boost/circular_buffer.hpp"
//#include "chrono/physics/ChProbe.h"
//#include "frydom/core/common/FrObject.h"
//#include "frydom/core/body/FrBody.h"
//#include "frydom/core/FrOffshoreSystem.h"
//
//namespace frydom {
//
//    /**
//     * \class FrPositionRecorder
//     * \brief Class not used.
//     */
//    class FrPositionRecorder : public chrono::ChProbe, public FrObject {
//
//    private:
//        FrBody* m_body = nullptr;           ///< Body to which the position is recrorded
//        unsigned int m_size = 0;            ///< size of the recorder
//        std::vector<boost::circular_buffer<double>> m_positions;    ///< Recorded position
//        double m_lastTime = -1;             ///< Time of the last element store in the recorder
//
//    public:
//        /// Default constructor
//        FrPositionRecorder() = default;
//
//        /// Set size of the recorder
//        void SetSize(unsigned int size);
//
//        /// Return the size of the recorder
//        unsigned int GetSize() const;
//
//        /// Set the body object linked to the recorder
//        void SetBody(FrBody* body);
//
//        /// Return the body object linked to the recorder
//        FrBody* GetBody() const;
//
//        /// Initialization step
//        void Initialize() override;
//
//        /// Return last time saved in the recorder
//        double GetTime() const;
//
//        /// Save current velocity into recorder
//        void RecordPosition();
//
//        /// Return the velocity recorded for specific dof
//        boost::circular_buffer<double> GetRecordOnDOF(unsigned int iDOF) const;
//
//        /// Method executed at the end of each time step
//        void StepFinalize() override;
//    };
//
//}   // end namespace frydom
//
//
//
//#endif //FRYDOM_FRPOSITIONRECORDER_H
