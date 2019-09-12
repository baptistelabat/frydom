//
// Created by lletourn on 11/09/19.
//


#include <chrono/physics/ChBodyEasy.h>
#include "frydom/frydom.h"
//#include "gtest/gtest.h"

using namespace frydom;


class TestBody : public FrBody {

public:

    void ForceCOG(const Position& cogPos) {
        SetCOG(cogPos, NWU);
    }

private:

    void StepFinalize() override {

        SetCOG(Position(4.,5.,6.+GetSystem()->GetTime()), NWU);

        FrBody::StepFinalize();
    }

    void AddFields() override {

        FrBody::AddFields();

        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                                                      ("COGPositionInBody","m", fmt::format("COG body position in the body reference frame in {}", GetLogFrameConvention()),
                                                              [this]() {return GetCOG(GetLogFrameConvention());});

    }

};



int main() {

// -- System

    FrOffshoreSystem system;
//    system.SetLogged(false);
    system.SetName("testCOG");
    system.GetPathManager()->SetLogFrameConvention(NWU);

    auto body = std::make_shared<TestBody>();
    system.AddBody(body);
    makeItBox(body, 1.,1.,1., 1000);
    body->SetPosition(Position(9.,6.,3.), NWU);
    body->SetFixedInWorld(true);

    auto node = body->NewNode();
    node->SetPositionInBody(Position(1.,2.,3.), NWU);
    node->ShowAsset(true);
    node->SetLogged(true);

    std::cout<<"body : \n"<<body->GetCOG(NWU)<<std::endl<<"node :\n" << node->GetNodePositionInBody(NWU)<<std::endl;
    std::cout<<"node WRT COG\n"<<node->GetFrameWRT_COG_InBody().GetPosition(NWU)<<std::endl;

    body->ForceCOG(Position(4.,5.,6.));

    std::cout<<"body : \n"<<body->GetCOG(NWU)<<std::endl<<"node :\n" << node->GetNodePositionInBody(NWU)<<std::endl;
    std::cout<<"node WRT COG\n"<<node->GetFrameWRT_COG_InBody().GetPosition(NWU)<<std::endl;

    system.Initialize();

    std::cout<<"body : \n"<<body->GetCOG(NWU)<<std::endl<<"node :\n" << node->GetNodePositionInBody(NWU)<<std::endl;
    std::cout<<"node WRT COG\n"<<node->GetFrameWRT_COG_InBody().GetPosition(NWU)<<std::endl;

//    system.RunInViewer(0,10.);

    auto time = 0.;
    double dt = 0.01;

//    clock_t begin = clock();

    while (time < 1.) {
        time += dt;

        system.AdvanceTo(time);

        std::cout<<"time : "<<time<<std::endl<<"body : \n"<<body->GetCOG(NWU)<<std::endl<<"node :\n" << node->GetNodePositionInBody(NWU)<<std::endl;
        std::cout<<"node WRT COG\n"<<node->GetFrameWRT_COG_InBody().GetPosition(NWU)<<std::endl;
    }

//    clock_t end = clock();
//    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
//    std::cout << "Elapsed cpu time in seconds : " << elapsed_secs << std::endl;
//    std::cout << "============================== End ===================================== " << std::endl;

}