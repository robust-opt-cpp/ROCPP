//
//  OptModelConverters.hpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#ifndef OptModelConverters_hpp
#define OptModelConverters_hpp

#include "HeaderIncludeFiles.hpp"


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%% BILINEAR TERM REFORMULATOR INTERFACE %%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class BilinearTermReformulatorIF  // creates MILP if original problem containts only bilinear terms involving at least one binary variable; creates convex relaxation otherwise
{
public:
    
    BilinearTermReformulatorIF(string aux_var_nme = "bl", string aux_var_sffx = "", uint auxVarCnt=0) :  m_aux_var_nme(aux_var_nme), m_aux_var_sffx(aux_var_sffx), m_auxVarCnt(auxVarCnt) {} //, m_pRVBPR ( new RealVarBilinearPosReformulator() ) {}
    ~BilinearTermReformulatorIF(){}
    
    void Reset() {m_auxVarCnt=0;}
    
    void setAuxVarNme(string aux_var_nme) {m_aux_var_nme=aux_var_nme;}
    
    void setAuxVarSffx(string aux_var_sffx) {m_aux_var_sffx=aux_var_sffx;}
    
    ROCPPOptModelIF_Ptr linearize(ROCPPOptModelIF_Ptr pIn, const map<string,pair<double,double> >& variableBounds = (map<string,pair<double,double> >()));
    
    virtual void getlinearCstr(ROCPPVarIF_Ptr bindv, ROCPPVarIF_Ptr otherdv, ROCPPVarIF_Ptr newdv,  vector<ROCPPConstraint_Ptr > &cstrvec, map<string,pair<double,double> > &variableBounds) = 0;
    
    uint getAuxVarCnt() const {return m_auxVarCnt;}
    
private:
    
    string m_aux_var_nme;
    string m_aux_var_sffx;
    uint m_auxVarCnt;
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%$$%%% BILINEAR TERM REFORMULATOR BIG_M %%%%%%%%%%%%$$%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class BTR_bigM : public BilinearTermReformulatorIF
{
public:
    
    BTR_bigM(string aux_var_nme = "bl", string aux_var_sffx = "", uint auxVarCnt=0, double M = 1.e2) : BilinearTermReformulatorIF(aux_var_nme,aux_var_sffx,auxVarCnt), m_M(M) {}
    ~BTR_bigM(){}
    
    void getlinearCstr(ROCPPVarIF_Ptr bindv, ROCPPVarIF_Ptr otherdv, ROCPPVarIF_Ptr newdv, vector<ROCPPConstraint_Ptr > &cstrvec, map<string,pair<double,double> > &variableBounds);
    
private:
    
    double m_M;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% MODEL CONVERT FUNCTION %%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPUncSSOptModel_Ptr convertToUSSOM(ROCPPOptModelIF_Ptr pIn);
ROCPPMISOCP_Ptr convertToMISOCP(ROCPPOptModelIF_Ptr pIn);

#endif /* OptModelConverters_hpp */
