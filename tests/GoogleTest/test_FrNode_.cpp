//
// Created by Lucas Letournel on 15/11/18.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;

TEST(FrNode,Position) {
    FRAME_CONVENTION fc = NED;

    /// Create a body
    auto body = std::make_shared<FrBody_>();

    ///         Set Position
    body->SetPosition(Position(1.,2.,3.), fc);

    ///         Set Orientation
    FrRotation_ BodyRotationInWorld; BodyRotationInWorld.SetCardanAngles_DEGREES(1.,2.,3.,fc);
    body->SetRotation(BodyRotationInWorld);

    ///         Set Velocity
    body->SetGeneralizedVelocityInWorld(Velocity(6.,5.,9.),AngularVelocity(8.,5.,1.),fc);

    ///         Set Acceleration
    body->SetAccelerationInWorldNoRotation(Acceleration(0.,4.,3.),fc);
    body->SetAngularAccelerationInWorld(AngularAcceleration(3.,0.,5.),fc);

    /// Create a node from a position given in body reference frame;
    Position NodePositionInBody(6.,2.,4.);

    auto node = body->NewNode(NodePositionInBody, fc);

    /// test GetBody
    EXPECT_TRUE(body.get()==node->GetBody());

    // test GetPositionInWorld
    Position testPosition  = node->GetPositionInWorld(fc) - body->GetPointPositionInWorld(NodePositionInBody,fc);
    EXPECT_TRUE(testPosition.isZero());
    if (not(testPosition.isZero())){
        std::cout<<body->GetPointPositionInWorld(NodePositionInBody,fc)<<std::endl;
        std::cout<<node->GetPositionInWorld(fc)<<std::endl;
    }

    /// test GetNodePositionInBody
    testPosition = node->GetNodePositionInBody(fc) - NodePositionInBody;
    EXPECT_TRUE(testPosition.isZero());
    if (not(testPosition.isZero())){
        std::cout<<NodePositionInBody<<std::endl;
        std::cout<<node->GetNodePositionInBody(fc)<<std::endl;
    }

    /// test GetVelocityInWorld
    Velocity testVelocity = node->GetVelocityInWorld(fc) - body->GetVelocityInWorldAtPointInBody(NodePositionInBody,fc);
    EXPECT_TRUE(testVelocity.isZero());
    if (not(testVelocity.isZero())){
        std::cout<<body->GetVelocityInWorldAtPointInBody(NodePositionInBody,fc)<<std::endl;
        std::cout<<node->GetVelocityInWorld(fc)<<std::endl;
    }

    /// test GetAccelerationInWorld
    Acceleration testAcceleration = node->GetAccelerationInWorld(fc) - body->GetAccelerationInWorldAtPointInBody(NodePositionInBody,fc);
    EXPECT_TRUE(testAcceleration.isZero());
    if (not(testAcceleration.isZero())){
        std::cout<<body->GetAccelerationInWorldAtPointInBody(NodePositionInBody,fc)<<std::endl;
        std::cout<<node->GetAccelerationInWorld(fc)<<std::endl;
    }

    /// test GetVelocityInNode
    testVelocity = node->GetFrameInWorld().GetRotation().Rotate(node->GetVelocityInNode(fc),fc) - body->GetVelocityInWorldAtPointInBody(NodePositionInBody,fc);
    EXPECT_TRUE(testVelocity.isZero());
    if (not(testVelocity.isZero())){
        std::cout<<body->GetVelocityInWorldAtPointInBody(NodePositionInBody,fc)<<std::endl;
        std::cout<< node->GetFrameInWorld().GetRotation().Rotate(node->GetVelocityInNode(fc),fc)<<std::endl;
    }

    /// test for fun
    auto Node2BodyRotation = body->GetFrame().GetThisFrameRelativeTransform_WRT_OtherFrame(node->GetFrameInWorld(),fc).GetRotation();
    testVelocity = Node2BodyRotation.Rotate(node->GetVelocityInNode(fc),fc) - body->GetVelocityInBodyAtPointInBody(NodePositionInBody,fc);
    EXPECT_TRUE(testVelocity.isZero());
    if (not(testVelocity.isZero())){
        std::cout<<body->GetVelocityInBodyAtPointInBody(NodePositionInBody,fc)<<std::endl;
        std::cout<<Node2BodyRotation.Rotate(node->GetVelocityInNode(fc),fc)<<std::endl;
    }

    /// test GetAccelerationInNode
    testAcceleration = node->GetFrameInWorld().GetRotation().Rotate(node->GetAccelerationInNode(fc),fc) - body->GetAccelerationInWorldAtPointInBody(NodePositionInBody,fc);
    EXPECT_TRUE(testAcceleration.isZero());
    if (not(testAcceleration.isZero())){
        std::cout<<body->GetAccelerationInWorldAtPointInBody(NodePositionInBody,fc)<<std::endl;
        std::cout<< node->GetFrameInWorld().GetRotation().Rotate(node->GetAccelerationInNode(fc),fc)<<std::endl;
    }


    /// Create a node from a frame;
    FrRotation_ NodeRotation; NodeRotation.SetCardanAngles_DEGREES(5.,9.,1.,fc);
    FrFrame_ NodeFrame;
    NodeFrame.SetPosition(NodePositionInBody,fc);
    NodeFrame.SetRotation(NodeRotation);

    auto node2 = body->NewNode(NodeFrame);

    /// test GetFrame
    testPosition  = node->GetFrameInWorld().GetPosition(fc) - body->GetPointPositionInWorld(NodePositionInBody,fc);
    EXPECT_TRUE(testPosition.isZero());
    if (not(testPosition.isZero())){
        std::cout<<body->GetPointPositionInWorld(NodePositionInBody,fc)<<std::endl;
        std::cout<<node->GetPositionInWorld(fc)<<std::endl;
    }

    /// test GetFrame
    bool testRotation = BodyRotationInWorld*NodeRotation == node2->GetFrameInWorld().GetRotation();
    EXPECT_TRUE(testRotation);
    if (not(testRotation)){
        std::cout<<BodyRotationInWorld*NodeRotation<<std::endl;
        std::cout<< node2->GetFrameInWorld().GetRotation()<<std::endl;
    }






}
