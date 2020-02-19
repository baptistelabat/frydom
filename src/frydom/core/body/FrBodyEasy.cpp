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


#include "FrBodyEasy.h"

#include "FrBody.h"


namespace frydom {

  void makeItBox(std::shared_ptr<FrBody> body, double xSize, double ySize, double zSize, double mass) {

    // Properties of the box
    double xSize2 = xSize * xSize;
    double ySize2 = ySize * ySize;
    double zSize2 = zSize * zSize;

    // inertia
    double Ixx = (1. / 12.) * mass * (ySize2 + zSize2);
    double Iyy = (1. / 12.) * mass * (xSize2 + zSize2);
    double Izz = (1. / 12.) * mass * (xSize2 + ySize2);

    // Building the chrono body
    body->SetInertiaTensor(FrInertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., Position(), NWU));


    // Collision
    auto collisionModel = body->GetChronoBody()->GetCollisionModel();
    collisionModel->ClearModel();
    collisionModel->AddBox(xSize * 0.5, ySize * 0.5, zSize * 0.5,
                           chrono::ChVector<double>()); // TODO: permettre de specifier une position relative (et orientation ?)
    collisionModel->BuildModel();
    body->AllowCollision(true);  // A retirer ??
    body->SetSmoothContact();

    // Asset
    body->AddBoxShape(xSize, ySize, zSize, {0., 0., 0.}, NWU);

  }

  void makeItCylinder(std::shared_ptr<FrBody> body, double radius, double height, double mass) {

    // Properties of the cylinder
    double r2 = radius * radius;
    double h2 = height * height;
    double Ixx = (1. / 12.) * mass * (3. * r2 + h2);  // FIXME : attention, on a pas les bons ordres !!
    double Iyy = 0.5 * mass * r2;
    double Izz = Ixx;

    // Building the chrono body
    body->SetInertiaTensor(FrInertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., Position(), NWU));

    // Collision
    auto collisionModel = body->m_chronoBody->GetCollisionModel();
    collisionModel->ClearModel();
    collisionModel->AddCylinder(radius, radius, height * 0.5,
                                chrono::ChVector<double>());  // TODO: permettre de specifier les coords relatives dans le modele !!
    collisionModel->BuildModel();
    body->AllowCollision(true);  // A retirer ?
    body->SetSmoothContact();  // Smooth contact by default

    // Asset
    body->AddCylinderShape(radius, height, {0., 0., 0.}, NWU);

  }

  void makeItSphere(std::shared_ptr<FrBody> body, double radius, double mass) {

    // Properties of the sphere
    double inertia = (2.0 / 5.0) * mass * radius * radius;

    // Building the Chrono body
    body->SetInertiaTensor(FrInertiaTensor(mass, inertia, inertia, inertia, 0., 0., 0., Position(), NWU));

    // Collision
    auto collisionModel = body->m_chronoBody->GetCollisionModel();
    collisionModel->ClearModel();
    collisionModel->AddSphere(radius,
                              chrono::ChVector<double>());  // TODO: permettre de specifier les coords relatives dans le modele !!
    collisionModel->BuildModel();
    body->AllowCollision(true);  // A retirer ?
    body->SetSmoothContact();  // Smooth contact by default

    // Asset
    body->AddSphereShape(radius, {0., 0., 0.}, NWU);

  }

  std::shared_ptr<FrBody> make_BoxBody(const std::string &name,
                                       FrOffshoreSystem *system,
                                       double xSize,
                                       double ySize,
                                       double zSize,
                                       double mass) {

    auto box = std::make_shared<FrBody>(name, system);
    makeItBox(box, xSize, ySize, zSize, mass);
    return box;
  }

  std::shared_ptr<FrBody> make_CylinderBody(const std::string &name,
                                            FrOffshoreSystem *system,
                                            double radius,
                                            double height,
                                            double mass) {

    auto cylinder = std::make_shared<FrBody>(name, system);
    makeItCylinder(cylinder, radius, height, mass);
    return cylinder;
  }

  std::shared_ptr<FrBody> make_SphereBody(const std::string &name,
                                          FrOffshoreSystem *system,
                                          double radius,
                                          double mass) {

    auto sphere = std::make_shared<FrBody>(name, system);
    makeItSphere(sphere, radius, mass);
    return sphere;
  }

}  // end namespace frydom
