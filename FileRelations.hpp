//
//  FileRelations.hpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#ifndef FileRelations_hpp
#define FileRelations_hpp

#include <iostream>
#include <map>
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
