/*
 * ROCPP/FileRelations.hpp
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

#ifndef FileRelations_hpp
#define FileRelations_hpp

#include <iostream>
#include <map>
#include <vector>
using namespace std;

template<class T1,class T2>
int writeCSV(string folderName, string fileName, const map<T1,T2> &out, bool clear = true, string delimitter = ", ", string extension = ".csv");

template<class T1, class T2,class T3>
int writeCSV(string folderName, string fileName, const map<T1,pair<T2,T3> > &out, string delimitter = ", ", string extension = ".csv");

template<class T1,class T2>
int readCSV(string folderName, string fileName, map<T1,vector<T2> > &out, char delimiter, string extension, bool skipHash);

bool FileExists(string strFilename);

// load solution from Gurobi .sol file
void loadResult(string folderName, string slnName, map<string, double> &results);
#endif /* FileRelations_hpp */
