Tree architecture {#tree}
=================

The different class objects in FRyDoM are organised into a tree architecture as represented in the next figure.

![Figure : representation of the organisation of the object in FRyDoM as a tree.](TreeNode_002_w600.png "Figure : representation of the organization of the object in FRyDoM as a tree.")

Each node, or class, derived from a frydom::FrTreeNode class templated by the parent of the node. One node can have multiple child nodes but only one parent node. The root node, corresponding to the system (frydom::FrOffshoreSystem), has no parent or an empty node (frydom::FrRootNode). The tree architecture is entirely managed by the frydom::FrPathManager, from which it is possible to evaluate the position of each node in the tree and its parent dependencies. 

The nodes are associated to a path, defining the position of the node in the tree. This path is composed of a normalized part (same of element of the same kind) and a user defined part (name) as follows:

    ParentPath/NormalizedPathPrefix_Name/

where :

- *ParentPath* is the path of the parent of the node
- *NormalizedPathPrefix* is the normalized part of the path defined by the type of the node. It is defined in the path policies file (frydom/logging/FrPathPolicies.cpp) with the TYPE_TO_NORMALIZED_PATH_PREFIX macro for each class object.
- *Name* is the name of the node given by the user.


The tree architecture of FRyDoM-CE can be seen in @ref frydom::FrTreeNodeBase graph.

**Note** : *Environment* objects are not included in this tree for now.



