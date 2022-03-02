/*
 * ROCPP/helpersOpt.hpp
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

#ifndef helpersOpt_hpp
#define helpersOpt_hpp

#include "HeaderIncludeFiles.hpp"


void findMarginalSupportUncertaintySet(ROCPPOptModelIF_Ptr pModelIn, map<string,pair<double,double> > &margSupp, ROCPPMItoMB_Ptr pMIMBConverter, const map<string,uint> &numPartitionsMap, string solver = "cplex",bool onlyObsUnc = true, bool onlyUncInMap = false, const map<string,pair<double,double> > &outerApproxMargSupp = map<string,pair<double,double> >());

void findWholeMarginalSupport(ROCPPOptModelIF_Ptr pModelIn, const map<string,uint> &numPartitionsMap, map<string,pair<double,double> > &margSupp);

double calculateArea(const map<string,pair<double,double> > &allMap);
#endif /* helpersOpt_hpp */
