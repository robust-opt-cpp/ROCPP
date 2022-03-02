/*
 * ROCPP/FileRelations.cpp
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

#include <stdio.h>
#include "FileRelations.hpp"
#include <fstream>
#include <sys/stat.h>
#include <sstream>
#include <vector>
#include "IncludeFiles.hpp"

template int writeCSV(string folderName, string fileName, const map<string,pair<double,double> > &out, string delimitter, string extension);
template int writeCSV(string folderName, string fileName, const map<string,double> &out, bool clear, string delimitter, string extension);
template int writeCSV(string folderName, string fileName, const map<uint,double> &out, bool clear, string delimitter, string extension);

template int readCSV(string folderName, string fileName, map<string,vector<double> > &out, char delimiter, string extension, bool skipHash);

template<class T1,class T2>
int writeCSV(string folderName, string fileName, const map<T1,T2> &out, bool clear, string delimitter, string extension)
{
    //string filePath = ".\\"+folderName+"\\";
    
    string tstr(fileName+extension);
    if (folderName!="")
        tstr = folderName+"/"+tstr;
    
    if (clear){
        ofstream ofsummary(tstr.c_str());
        ofsummary.clear();
        
        //ofsummary << scientific;
        ofsummary.precision(4);
        //string comma(", ");
        
        typename map<T1,T2>::const_iterator p_it = out.begin();
        for (; p_it!=out.end(); ++p_it)
        {
            ofsummary << p_it->first << delimitter;
            ofsummary << p_it->second << endl;
        }
        ofsummary.close();
    }
    else{
        ofstream ofsummary(tstr.c_str(), ios::app);
        
        //ofsummary << scientific;
        ofsummary.precision(4);
        //string comma(", ");
        
        typename map<T1,T2>::const_iterator p_it = out.begin();
        for (; p_it!=out.end(); ++p_it)
        {
            ofsummary << p_it->first << delimitter;
            ofsummary << p_it->second << endl;
        }
        ofsummary.close();
    }
    return 0;
}


template<class T1, class T2,class T3>
int writeCSV(string folderName, string fileName, const map<T1,pair<T2,T3> > &out, string delimitter, string extension)
{
    //string filePath = ".\\"+folderName+"\\";
    
    string tstr(fileName+extension);
    if (folderName!="")
        tstr = folderName+"/"+tstr;
    
    ofstream ofsummary(tstr.c_str());
    ofsummary.clear();
    ofsummary << scientific;
    ofsummary.precision(30);
    //string comma(", ");
    
    typename map<T1,pair<T2,T3> >::const_iterator p_it = out.begin();
    for (; p_it!=out.end(); ++p_it)
    {
        ofsummary << p_it->first << delimitter;
        ofsummary << p_it->second.first << delimitter;
        ofsummary << p_it->second.second << endl;
    }
    ofsummary.close();
    return 0;
}


template<class T1,class T2>
int readCSV(string folderName, string fileName, map<T1,vector<T2> > &out, char delimiter, string extension, bool skipHash)
{
    out.clear();
    //string filePath = ".\\"+folderName+"\\";
    
    string tstr(fileName+extension);
    if (folderName!="")
        tstr = folderName+"/"+tstr;
    
    
    ifstream ifs(tstr.c_str());
    //ifstream ifs((filePath+fileName+".csv").c_str());
    
    //cout << "blu" << endl;
    
    if(!ifs.is_open())
    {
        std::cout << "File " << tstr << " not found!\n";
        throw MyException( ("File " + tstr + " not found!").c_str() );
        return 1;
    }
    
    
    
    string csvLine;
    
    // if it's a solution file, skip the first line which has a different format
    if (extension==".sol")
        getline(ifs, csvLine);
    
    
    // read every line from the stream
    istringstream strT;
    while(getline(ifs, csvLine))
    {
        istringstream csvStream(csvLine);
        string tmp(csvLine.substr(0,1));
        
        if ( (!skipHash) || (tmp!="#") )
        {
            string rowIdxStr;
            getline(csvStream, rowIdxStr, delimiter);
            
            vector<T2> csvRow;
            string csvElement;
            
            // read every element from the line that is seperated by commas
            // and put it into the vector or doubles
            while( (getline(csvStream, csvElement, delimiter)) ) //|| (getline(csvStream, csvElement, '\n')) || (getline(csvStream, csvElement, '\r')) )
            {
                //csvElement.erase(0,1);
                std::size_t loc( csvElement.find("\r") );
                if (loc!=std::string::npos)
                    csvElement = csvElement.erase(loc);
                loc = csvElement.find("\n");
                if (loc!=std::string::npos)
                    csvElement = csvElement.erase(loc);
                
                //MARK:convert string to type name
                T2 csvT2element;
                strT.str(csvElement);
                strT >> csvT2element;
                strT.clear();
                //= boost::lexical_cast<T2>(csvElement);
                csvRow.push_back(csvT2element);
            }
            //double tmp = boost::lexical_cast<double>(rowIdxStr);
            T1 rowIdx;
            strT.str(rowIdxStr);
            strT >> rowIdx;
            strT.clear();
            //= boost::lexical_cast<T1>();
            //T1 rowIdx = boost::lexical_cast<T1>(rowIdxStr);
            out.insert(out.end(),make_pair(rowIdx,csvRow));//.push_back(csvRow);
        }
    }
    
    return 0;
}



bool FileExists(string strFilename) {
    struct stat stFileInfo;
    bool blnReturn;
    int intStat;
    
    // Attempt to get the file attributes
    intStat = stat(strFilename.c_str(),&stFileInfo);
    if (intStat == 0)
    {
        blnReturn = true;
    }
    else
    {
        blnReturn = false;
        //throw MyException( ("File " + strFilename + " does not exist").c_str() );
    }
    
    return(blnReturn);
}

void loadResult(string folderName, string slnName, map<string, double> &results)
{
    map<string, vector<double> > valsMap;
    readCSV(folderName, slnName+"All", valsMap, ' ',".sol", true);
    
    map<string, vector<double> >::const_iterator val = valsMap.begin();
    
    for(; val != valsMap.end(); val++)
        results.insert(make_pair(val->first, val->second[0]));
}
