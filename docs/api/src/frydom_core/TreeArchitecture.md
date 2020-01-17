Tree architecture {#tree}
=================

The different class objects in FRyDoM are organised into a tree architecture as represented in the next figure.

![Figure : representation of the organisation of the object in FRyDoM as a tree.](TreeNode_002_w600.png "Figure : representation of the organization of the object in FRyDoM as a tree.")

Each node, or class, derived from a FrTreeNode class templated by the parent of the node (FrTreeNode<ParentType>). One node can have multiple child nodes but only one parent node. The root node, corresponding to the system ([FrOffshoreSystem](#FrOffshoreSystem)), has no parent or an empty node (RootNode). The tree architecture is entirely managed by the [path manager](#FrPathManager) from which it is possible to evaluate the position of each node in the tree and its parent dependencies. 

The nodes are associated to a path, defining the position of the node in the tree. This path is composed of a normalized part (same of of element of the same kind) and a user defined part (name) as follows:

    ParentPath/NormalizedPathPrefix_Name/

where :

- *ParentPath* is the path of the parent of the node
- *NormalizedPathPrefix* is the normalized part of the path defined by the type the the node. It is defined in the path policies file (FrPathPolicies.cpp) for each class object and is fixed for each kind of object.
- *Name* is the name of the node given by the user.


The tree architecture of FRyDoM-CE is represented in [FrTreeNode](#FrTreeNode) graph.

**Note** : *Environment* objects are not included in this tree for now.



