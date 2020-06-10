//
//  helpersOpt.hpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#ifndef helpersOpt_hpp
#define helpersOpt_hpp

#include "HeaderIncludeFiles.hpp"

class OptimizationModelIF;
class UncertainOptimizationModel;
class UncertaintyIF;
class Bilinear_MItoMB_Converter;
class BinaryConverter;

void findMarginalSupportUncertaintySet(boost::shared_ptr<OptimizationModelIF> pModelIn, map<string,pair<double,double> > &margSupp, boost::shared_ptr<Bilinear_MItoMB_Converter> pMIMBConverter, const map<string,uint> &numPartitionsMap, string solver = "cplex",bool onlyObsUnc = true, bool onlyUncInMap = false, const map<string,pair<double,double> > &outerApproxMargSupp = map<string,pair<double,double> >());

void findWholeMarginalSupport(boost::shared_ptr<OptimizationModelIF> pModelIn, const map<string,uint> &numPartitionsMap, map<string,pair<double,double> > &margSupp);

double calculateArea(const map<string,pair<double,double> > &allMap);
#endif /* helpersOpt_hpp */
