//
//  Interface.hpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 The University of Southern California. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#ifndef Interface_hpp
#define Interface_hpp
#include "HeaderIncludeFiles.hpp"


// REAL + ...
// real + var
ROCPPExpr_Ptr operator+(double real, ROCPPVarIF_Ptr var);
// real - var
ROCPPExpr_Ptr operator-(double real, ROCPPVarIF_Ptr var);

// real + unc
ROCPPExpr_Ptr operator+(double real, ROCPPUnc_Ptr unc);
// real - unc
ROCPPExpr_Ptr operator-(double real, ROCPPUnc_Ptr unc);

// real + expr
ROCPPExpr_Ptr operator+(double real, ROCPPconstExpr_Ptr expr);
// real - expr
ROCPPExpr_Ptr operator-(double real, ROCPPconstExpr_Ptr expr);

// real + term
ROCPPExpr_Ptr operator+(double real, ROCPPconstCstrTermIF_Ptr term);
// real - term
ROCPPExpr_Ptr operator-(double real, ROCPPconstCstrTermIF_Ptr term);


// VAR + ...
// var1 + var2
ROCPPExpr_Ptr operator+(ROCPPVarIF_Ptr var1, ROCPPVarIF_Ptr var2);
// var1 - var2
ROCPPExpr_Ptr operator-(ROCPPVarIF_Ptr var1, ROCPPVarIF_Ptr var2);

// var + real
ROCPPExpr_Ptr operator+(ROCPPVarIF_Ptr var, double real);
// var - real
ROCPPExpr_Ptr operator-(ROCPPVarIF_Ptr var, double real);

// var + unc
ROCPPExpr_Ptr operator+(ROCPPVarIF_Ptr var, ROCPPUnc_Ptr unc);
// var - unc
ROCPPExpr_Ptr operator-(ROCPPVarIF_Ptr var, ROCPPUnc_Ptr unc);

// var + expr
ROCPPExpr_Ptr operator+(ROCPPVarIF_Ptr var, ROCPPconstExpr_Ptr expr);
// var - expr
ROCPPExpr_Ptr operator-(ROCPPVarIF_Ptr var, ROCPPconstExpr_Ptr expr);

// var + term
ROCPPExpr_Ptr operator+(ROCPPVarIF_Ptr var, ROCPPconstCstrTermIF_Ptr term);
// var - term
ROCPPExpr_Ptr operator-(ROCPPVarIF_Ptr var, ROCPPconstCstrTermIF_Ptr term);


// UNC + ...
// unc + var
ROCPPExpr_Ptr operator+(ROCPPUnc_Ptr unc, ROCPPVarIF_Ptr var);
// unc - var
ROCPPExpr_Ptr operator-(ROCPPUnc_Ptr unc, ROCPPVarIF_Ptr var);

// unc + real
ROCPPExpr_Ptr operator+(ROCPPUnc_Ptr unc, double real);
// unc - real
ROCPPExpr_Ptr operator-(ROCPPUnc_Ptr unc, double real);

// unc + unc
ROCPPExpr_Ptr operator+(ROCPPUnc_Ptr unc, ROCPPUnc_Ptr unc1);
// unc - unc
ROCPPExpr_Ptr operator-(ROCPPUnc_Ptr unc, ROCPPUnc_Ptr unc1);

// unc + expr
ROCPPExpr_Ptr operator+(ROCPPUnc_Ptr unc, ROCPPconstExpr_Ptr expr);
// unc - expr
ROCPPExpr_Ptr operator-(ROCPPUnc_Ptr unc, ROCPPconstExpr_Ptr expr);

// unc + term
ROCPPExpr_Ptr operator+(ROCPPUnc_Ptr unc, ROCPPconstCstrTermIF_Ptr term);
// unc - term
ROCPPExpr_Ptr operator-(ROCPPUnc_Ptr unc, ROCPPconstCstrTermIF_Ptr term);


// EXPR + ...
// expr + var
ROCPPExpr_Ptr operator+(ROCPPconstExpr_Ptr expr, ROCPPVarIF_Ptr var);
// expr - var
ROCPPExpr_Ptr operator-(ROCPPconstExpr_Ptr expr, ROCPPVarIF_Ptr var);

// expr + real
ROCPPExpr_Ptr operator+(ROCPPconstExpr_Ptr expr, double real);
// expr - real
ROCPPExpr_Ptr operator-(ROCPPconstExpr_Ptr expr, double real);

// expr + unc
ROCPPExpr_Ptr operator+(ROCPPconstExpr_Ptr expr, ROCPPUnc_Ptr unc);
// expr - unc
ROCPPExpr_Ptr operator-(ROCPPconstExpr_Ptr expr, ROCPPUnc_Ptr unc);

// expr + expr
ROCPPExpr_Ptr operator+(ROCPPconstExpr_Ptr expr1, ROCPPconstExpr_Ptr expr2);
// expr - expr
ROCPPExpr_Ptr operator-(ROCPPconstExpr_Ptr expr1, ROCPPconstExpr_Ptr expr2);

// expr + term
ROCPPExpr_Ptr operator+(ROCPPconstExpr_Ptr expr, ROCPPconstCstrTermIF_Ptr term);
// expr - term
ROCPPExpr_Ptr operator-(ROCPPconstExpr_Ptr expr, ROCPPconstCstrTermIF_Ptr term);



// TERM + ...
// term + var
ROCPPExpr_Ptr operator+(ROCPPconstCstrTermIF_Ptr term, ROCPPVarIF_Ptr var);
// term - var
ROCPPExpr_Ptr operator-(ROCPPconstCstrTermIF_Ptr term, ROCPPVarIF_Ptr var);

// term + real
ROCPPExpr_Ptr operator+(ROCPPconstCstrTermIF_Ptr term, double real);
// term - real
ROCPPExpr_Ptr operator-(ROCPPconstCstrTermIF_Ptr term, double real);

// term + unc
ROCPPExpr_Ptr operator+(ROCPPconstCstrTermIF_Ptr term, ROCPPUnc_Ptr unc);
// term - unc
ROCPPExpr_Ptr operator-(ROCPPconstCstrTermIF_Ptr term, ROCPPUnc_Ptr unc);

// term + expr
ROCPPExpr_Ptr operator+(ROCPPconstCstrTermIF_Ptr term, ROCPPconstExpr_Ptr expr);
// term - expr
ROCPPExpr_Ptr operator-(ROCPPconstCstrTermIF_Ptr term, ROCPPconstExpr_Ptr expr);

// term + term
ROCPPExpr_Ptr operator+(ROCPPconstCstrTermIF_Ptr term1, ROCPPconstCstrTermIF_Ptr term2);
// term - term
ROCPPExpr_Ptr operator-(ROCPPconstCstrTermIF_Ptr term1, ROCPPconstCstrTermIF_Ptr term2);


// real * var
ROCPPExpr_Ptr operator*(double real, ROCPPVarIF_Ptr var);

// real * unc
ROCPPExpr_Ptr operator*(double real, ROCPPUnc_Ptr unc);

// real * expr
ROCPPExpr_Ptr operator*(double real, ROCPPconstExpr_Ptr expr);

// var1 * var2
ROCPPExpr_Ptr operator*(ROCPPVarIF_Ptr var, ROCPPVarIF_Ptr var2);

// var * unc
ROCPPExpr_Ptr operator*(ROCPPVarIF_Ptr var, ROCPPUnc_Ptr unc);

// var * expr
ROCPPExpr_Ptr operator*(ROCPPVarIF_Ptr var, ROCPPExpr_Ptr expr);

// unc * expr
ROCPPExpr_Ptr operator*(ROCPPUnc_Ptr unc, ROCPPExpr_Ptr expr);

// unc * var
ROCPPExpr_Ptr operator*(ROCPPUnc_Ptr unc, ROCPPVarIF_Ptr var);

// expr * unc
ROCPPExpr_Ptr operator*(ROCPPExpr_Ptr expr, ROCPPUnc_Ptr unc);

// expr * var
ROCPPExpr_Ptr operator*(ROCPPExpr_Ptr expr, ROCPPVarIF_Ptr var);

// expr * real
ROCPPExpr_Ptr operator*(ROCPPconstExpr_Ptr expr, double real);

// real * term
ROCPPExpr_Ptr operator*(double real, ROCPPconstCstrTermIF_Ptr term);


// REAL + ...
// expr <= real
ROCPPConstraintIF_Ptr operator<=(ROCPPExpr_Ptr expr, double real);
// expr>= real
ROCPPConstraintIF_Ptr operator>=(ROCPPExpr_Ptr expr, double real);
// expr == real
ROCPPConstraintIF_Ptr operator==(ROCPPExpr_Ptr expr, double real);

// real <= expr
ROCPPConstraintIF_Ptr operator<=(double real, ROCPPExpr_Ptr expr);
// real >= expr
ROCPPConstraintIF_Ptr operator>=(double real, ROCPPExpr_Ptr expr);
// real == expr
ROCPPConstraintIF_Ptr operator==(double real, ROCPPExpr_Ptr expr);


// var <= real
ROCPPConstraintIF_Ptr operator<=(ROCPPVarIF_Ptr var, double real);
// var>= real
ROCPPConstraintIF_Ptr operator>=(ROCPPVarIF_Ptr var, double real);
// var == real
ROCPPConstraintIF_Ptr operator==(ROCPPVarIF_Ptr var, double real);

// real <= var
ROCPPConstraintIF_Ptr operator<=(double real, ROCPPVarIF_Ptr var);
// real >= var
ROCPPConstraintIF_Ptr operator>=(double real, ROCPPVarIF_Ptr var);
// real == var
ROCPPConstraintIF_Ptr operator==(double real, ROCPPVarIF_Ptr var);


// unc <= real
ROCPPConstraintIF_Ptr operator<=(ROCPPUnc_Ptr unc, double real);
// unc >= real
ROCPPConstraintIF_Ptr operator>=(ROCPPUnc_Ptr unc, double real);
// unc == real
ROCPPConstraintIF_Ptr operator==(ROCPPUnc_Ptr unc, double real);

// real <= unc
ROCPPConstraintIF_Ptr operator<=(double real, ROCPPUnc_Ptr unc);
// real >= unc
ROCPPConstraintIF_Ptr operator>=(double real, ROCPPUnc_Ptr unc);
// real == unc
ROCPPConstraintIF_Ptr operator==(double real, ROCPPUnc_Ptr unc);


// term <= real
ROCPPConstraintIF_Ptr operator<=(ROCPPCstrTermIF_Ptr term, double real);
// term >= real
ROCPPConstraintIF_Ptr operator>=(ROCPPCstrTermIF_Ptr term, double real);
// term == real
ROCPPConstraintIF_Ptr operator==(ROCPPCstrTermIF_Ptr term, double real);

// real <= term
ROCPPConstraintIF_Ptr operator<=(double real, ROCPPCstrTermIF_Ptr term);
// real >= term
ROCPPConstraintIF_Ptr operator>=(double real, ROCPPCstrTermIF_Ptr term);
// real == term
ROCPPConstraintIF_Ptr operator==(double real, ROCPPCstrTermIF_Ptr term);



// EXPR + ...
// expr <= var
ROCPPConstraintIF_Ptr operator<=(ROCPPExpr_Ptr expr, ROCPPVarIF_Ptr var);
// expr>= var
ROCPPConstraintIF_Ptr operator>=(ROCPPExpr_Ptr expr, ROCPPVarIF_Ptr var);
// expr == var
ROCPPConstraintIF_Ptr operator==(ROCPPExpr_Ptr expr, ROCPPVarIF_Ptr var);

// var <= expr
ROCPPConstraintIF_Ptr operator<=(ROCPPVarIF_Ptr var, ROCPPExpr_Ptr expr);
// var>= expr
ROCPPConstraintIF_Ptr operator>=(ROCPPVarIF_Ptr var, ROCPPExpr_Ptr expr);
// var == expr
ROCPPConstraintIF_Ptr operator==(ROCPPVarIF_Ptr var, ROCPPExpr_Ptr expr);


// expr <= unc
ROCPPConstraintIF_Ptr operator<=(ROCPPExpr_Ptr expr, ROCPPUnc_Ptr unc);
// expr>= unc
ROCPPConstraintIF_Ptr operator>=(ROCPPExpr_Ptr expr, ROCPPUnc_Ptr unc);
// expr == unc
ROCPPConstraintIF_Ptr operator==(ROCPPExpr_Ptr expr, ROCPPUnc_Ptr unc);

// unc <= expr
ROCPPConstraintIF_Ptr operator<=(ROCPPUnc_Ptr unc, ROCPPExpr_Ptr expr);
// unc >= expr
ROCPPConstraintIF_Ptr operator>=(ROCPPUnc_Ptr unc, ROCPPExpr_Ptr expr);
// unc == expr
ROCPPConstraintIF_Ptr operator==(ROCPPUnc_Ptr unc, ROCPPExpr_Ptr expr);


// expr <= expr
ROCPPConstraintIF_Ptr operator<=(ROCPPExpr_Ptr expr1, ROCPPExpr_Ptr expr2);
// expr>= expr
ROCPPConstraintIF_Ptr operator>=(ROCPPExpr_Ptr expr1, ROCPPExpr_Ptr expr2);
// expr == expr
ROCPPConstraintIF_Ptr operator==(ROCPPExpr_Ptr expr1, ROCPPExpr_Ptr expr2);


// term <= expr
ROCPPConstraintIF_Ptr operator<=(ROCPPCstrTermIF_Ptr term, ROCPPExpr_Ptr expr);
// term >= expr
ROCPPConstraintIF_Ptr operator>=(ROCPPCstrTermIF_Ptr term, ROCPPExpr_Ptr expr);
// term == expr
ROCPPConstraintIF_Ptr operator==(ROCPPCstrTermIF_Ptr term, ROCPPExpr_Ptr expr);

// expr <= term
ROCPPConstraintIF_Ptr operator<=(ROCPPExpr_Ptr expr, ROCPPCstrTermIF_Ptr term);
// expr>= term
ROCPPConstraintIF_Ptr operator>=(ROCPPExpr_Ptr expr, ROCPPCstrTermIF_Ptr term);
// expr == term
ROCPPConstraintIF_Ptr operator==(ROCPPExpr_Ptr expr, ROCPPCstrTermIF_Ptr term);



// VAR + ...
// var <= var
ROCPPConstraintIF_Ptr operator<=(ROCPPVarIF_Ptr var1, ROCPPVarIF_Ptr var2);
// var>= var
ROCPPConstraintIF_Ptr operator>=(ROCPPVarIF_Ptr var1, ROCPPVarIF_Ptr var2);
// var == var
ROCPPConstraintIF_Ptr operator==(ROCPPVarIF_Ptr var1, ROCPPVarIF_Ptr var2);


// var <= unc
ROCPPConstraintIF_Ptr operator<=(ROCPPVarIF_Ptr var, ROCPPUnc_Ptr unc);
// var>= unc
ROCPPConstraintIF_Ptr operator>=(ROCPPVarIF_Ptr var, ROCPPUnc_Ptr unc);
// var == unc
ROCPPConstraintIF_Ptr operator==(ROCPPVarIF_Ptr var, ROCPPUnc_Ptr unc);

// unc <= var
ROCPPConstraintIF_Ptr operator<=(ROCPPUnc_Ptr unc, ROCPPVarIF_Ptr var);
// unc >= var
ROCPPConstraintIF_Ptr operator>=(ROCPPUnc_Ptr unc, ROCPPVarIF_Ptr var);
// unc == var
ROCPPConstraintIF_Ptr operator==(ROCPPUnc_Ptr unc, ROCPPVarIF_Ptr var);


// term <= var
ROCPPConstraintIF_Ptr operator<=(ROCPPCstrTermIF_Ptr term, ROCPPVarIF_Ptr var);
// term >= var
ROCPPConstraintIF_Ptr operator>=(ROCPPCstrTermIF_Ptr term, ROCPPVarIF_Ptr var);
// term == var
ROCPPConstraintIF_Ptr operator==(ROCPPCstrTermIF_Ptr term, ROCPPVarIF_Ptr var);

// var <= term
ROCPPConstraintIF_Ptr operator<=(ROCPPVarIF_Ptr var, ROCPPCstrTermIF_Ptr term);
// var>= term
ROCPPConstraintIF_Ptr operator>=(ROCPPVarIF_Ptr var, ROCPPCstrTermIF_Ptr term);
// var == term
ROCPPConstraintIF_Ptr operator==(ROCPPVarIF_Ptr var, ROCPPCstrTermIF_Ptr term);



// UNC + ...

// unc <= unc
ROCPPConstraintIF_Ptr operator<=(ROCPPUnc_Ptr unc1, ROCPPUnc_Ptr unc2);
// unc >= unc
ROCPPConstraintIF_Ptr operator>=(ROCPPUnc_Ptr unc1, ROCPPUnc_Ptr unc2);
// unc == unc
ROCPPConstraintIF_Ptr operator==(ROCPPUnc_Ptr unc1, ROCPPUnc_Ptr unc2);


// term <= unc
ROCPPConstraintIF_Ptr operator<=(ROCPPCstrTermIF_Ptr term, ROCPPUnc_Ptr unc);
// term >= unc
ROCPPConstraintIF_Ptr operator>=(ROCPPCstrTermIF_Ptr term, ROCPPUnc_Ptr unc);
// term == unc
ROCPPConstraintIF_Ptr operator==(ROCPPCstrTermIF_Ptr term, ROCPPUnc_Ptr unc);

// unc <= term
ROCPPConstraintIF_Ptr operator<=(ROCPPUnc_Ptr unc, ROCPPCstrTermIF_Ptr term);
// unc >= term
ROCPPConstraintIF_Ptr operator>=(ROCPPUnc_Ptr unc, ROCPPCstrTermIF_Ptr term);
// unc == term
ROCPPConstraintIF_Ptr operator==(ROCPPUnc_Ptr unc, ROCPPCstrTermIF_Ptr term);




#endif /* Interface_hpp */
