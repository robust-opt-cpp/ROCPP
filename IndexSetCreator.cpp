//
//  IndexSetCreator.cpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#include "IndexSetCreator.hpp"
#include <fstream>


void createAllVectorsElements1toI(vector<vector<uint> > &elements, size_t vec_length, size_t valI )
{
    elements.clear();
    
    bool addedMore(true);
    
    for (uint candidate=1; candidate <= valI; candidate++)
    {
        vector<uint> tmp;
        tmp.push_back(candidate);
        elements.push_back(tmp);
    }
    
    while (addedMore) {
        
        addedMore=appendVectorsElements1toI(elements, vec_length, valI);
        
    }
}


bool appendVectorsElements1toI(vector<vector<uint> > &elements, size_t vec_length, size_t valI)
{
    bool addedMore(false);
    
    vector<vector<uint> > elementsTmp;
    
    for (vector<vector<uint> >::iterator vit = elements.begin(); vit != elements.end(); vit++)
    {
        for (uint candidate=1; candidate <= valI; candidate++)
        {
            if ( (*vit).size() < vec_length)
            {
                vector<uint> cvec = *vit;
                cvec.push_back(candidate);
                elementsTmp.push_back(cvec);
                addedMore = true;
            }
        }
    }
    
    if (addedMore)
        elements = elementsTmp;
    
    return addedMore;
}
