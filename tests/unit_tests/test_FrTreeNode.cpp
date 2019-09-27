//
// Created by frongere on 27/09/19.
//

#include "frydom/frydom.h"

using namespace frydom;

int main() {

  FrOffshoreSystem system;


  FrTreeNode<FrOffshoreSystem> tree_node(&system);

//  tree_node.SetParent(&system);
  auto parent = tree_node.GetParent();


  return 0;
}
