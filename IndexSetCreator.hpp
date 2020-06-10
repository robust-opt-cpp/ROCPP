//
//  IndexSetCreator.hpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#ifndef IndexSetCreator_hpp
#define IndexSetCreator_hpp

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

void createAllVectorsElements1toI(vector<vector<uint> > &elements, size_t vec_length, size_t valI );
bool appendVectorsElements1toI(vector<vector<uint> > &elements, size_t vec_length, size_t valI);

#endif /* IndexSetCreator_hpp */
