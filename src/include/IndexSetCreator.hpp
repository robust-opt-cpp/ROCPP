/*
* ROCPP/IndexSetCreator.hpp
*
* This software is Copyright © 2020 The University of Southern California. All Rights Reserved.
* Authors: Phebe Vayanos, Qing Jin, George Elissaios
*
* This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
* If a copy of the MPL was not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.
*
* Paper: "ROC++: Robust Optimization in C++"
* Homepage: https://sites.google.com/usc.edu/robust-opt-cpp/home
*/


#ifndef IndexSetCreator_hpp
#define IndexSetCreator_hpp

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include "HeaderIncludeFiles.hpp"

using namespace std;

void createAllVectorsElements1toI(vector<vector<uint> > &elements, size_t vec_length, size_t valI );
bool appendVectorsElements1toI(vector<vector<uint> > &elements, size_t vec_length, size_t valI);

#endif /* IndexSetCreator_hpp */
