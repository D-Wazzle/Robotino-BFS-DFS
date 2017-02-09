//
//  searchTreeNode.h
//  searchTree
//
//  Created by Kristopher Krasnosky on 11/20/13.
//  Copyright (c) 2013 DASL. All rights reserved.
//

#ifndef __searchTree__searchTreeNode__
#define __searchTree__searchTreeNode__

#include <iostream>
#include <vector>
#include "map.h"


class searchTreeNode{
public:
    searchTreeNode();
    std::vector<searchTreeNode *> children;
    std::vector<searchTreeNode *> parent;
    map::gridPoint *point;
private:
    
};




#endif /* defined(__searchTree__searchTreeNode__) */
