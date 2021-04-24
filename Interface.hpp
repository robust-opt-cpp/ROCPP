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
ROCPPExpr_Ptr operator+(double real, ROCPPconstCstrTerm_Ptr term);
// real - term
ROCPPExpr_Ptr operator-(double real, ROCPPconstCstrTerm_Ptr term);


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
ROCPPExpr_Ptr operator+(ROCPPVarIF_Ptr var, ROCPPconstCstrTerm_Ptr term);
// var - term
ROCPPExpr_Ptr operator-(ROCPPVarIF_Ptr var, ROCPPconstCstrTerm_Ptr term);


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
ROCPPExpr_Ptr operator+(ROCPPUnc_Ptr unc, ROCPPconstCstrTerm_Ptr term);
// unc - term
ROCPPExpr_Ptr operator-(ROCPPUnc_Ptr unc, ROCPPconstCstrTerm_Ptr term);


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
ROCPPExpr_Ptr operator+(ROCPPconstExpr_Ptr expr, ROCPPconstCstrTerm_Ptr term);
// expr - term
ROCPPExpr_Ptr operator-(ROCPPconstExpr_Ptr expr, ROCPPconstCstrTerm_Ptr term);



// TERM + ...
// term + var
ROCPPExpr_Ptr operator+(ROCPPconstCstrTerm_Ptr term, ROCPPVarIF_Ptr var);
// term - var
ROCPPExpr_Ptr operator-(ROCPPconstCstrTerm_Ptr term, ROCPPVarIF_Ptr var);

// term + real
ROCPPExpr_Ptr operator+(ROCPPconstCstrTerm_Ptr term, double real);
// term - real
ROCPPExpr_Ptr operator-(ROCPPconstCstrTerm_Ptr term, double real);

// term + unc
ROCPPExpr_Ptr operator+(ROCPPconstCstrTerm_Ptr term, ROCPPUnc_Ptr unc);
// term - unc
ROCPPExpr_Ptr operator-(ROCPPconstCstrTerm_Ptr term, ROCPPUnc_Ptr unc);

// term + expr
ROCPPExpr_Ptr operator+(ROCPPconstCstrTerm_Ptr term, ROCPPconstExpr_Ptr expr);
// term - expr
ROCPPExpr_Ptr operator-(ROCPPconstCstrTerm_Ptr term, ROCPPconstExpr_Ptr expr);

// term + term
ROCPPExpr_Ptr operator+(ROCPPconstCstrTerm_Ptr term1, ROCPPconstCstrTerm_Ptr term2);
// term - term
ROCPPExpr_Ptr operator-(ROCPPconstCstrTerm_Ptr term1, ROCPPconstCstrTerm_Ptr term2);


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
ROCPPExpr_Ptr operator*(double real, ROCPPconstCstrTerm_Ptr term);


// REAL + ...
// expr <= real
ROCPPConstraint_Ptr operator<=(ROCPPExpr_Ptr expr, double real);
// expr>= real
ROCPPConstraint_Ptr operator>=(ROCPPExpr_Ptr expr, double real);
// expr == real
ROCPPConstraint_Ptr operator==(ROCPPExpr_Ptr expr, double real);

// real <= expr
ROCPPConstraint_Ptr operator<=(double real, ROCPPExpr_Ptr expr);
// real >= expr
ROCPPConstraint_Ptr operator>=(double real, ROCPPExpr_Ptr expr);
// real == expr
ROCPPConstraint_Ptr operator==(double real, ROCPPExpr_Ptr expr);


// var <= real
ROCPPConstraint_Ptr operator<=(ROCPPVarIF_Ptr var, double real);
// var>= real
ROCPPConstraint_Ptr operator>=(ROCPPVarIF_Ptr var, double real);
// var == real
ROCPPConstraint_Ptr operator==(ROCPPVarIF_Ptr var, double real);

// real <= var
ROCPPConstraint_Ptr operator<=(double real, ROCPPVarIF_Ptr var);
// real >= var
ROCPPConstraint_Ptr operator>=(double real, ROCPPVarIF_Ptr var);
// real == var
ROCPPConstraint_Ptr operator==(double real, ROCPPVarIF_Ptr var);


// unc <= real
ROCPPConstraint_Ptr operator<=(ROCPPUnc_Ptr unc, double real);
// unc >= real
ROCPPConstraint_Ptr operator>=(ROCPPUnc_Ptr unc, double real);
// unc == real
ROCPPConstraint_Ptr operator==(ROCPPUnc_Ptr unc, double real);

// real <= unc
ROCPPConstraint_Ptr operator<=(double real, ROCPPUnc_Ptr unc);
// real >= unc
ROCPPConstraint_Ptr operator>=(double real, ROCPPUnc_Ptr unc);
// real == unc
ROCPPConstraint_Ptr operator==(double real, ROCPPUnc_Ptr unc);


// term <= real
ROCPPConstraint_Ptr operator<=(ROCPPCstrTerm_Ptr term, double real);
// term >= real
ROCPPConstraint_Ptr operator>=(ROCPPCstrTerm_Ptr term, double real);
// term == real
ROCPPConstraint_Ptr operator==(ROCPPCstrTerm_Ptr term, double real);

// real <= term
ROCPPConstraint_Ptr operator<=(double real, ROCPPCstrTerm_Ptr term);
// real >= term
ROCPPConstraint_Ptr operator>=(double real, ROCPPCstrTerm_Ptr term);
// real == term
ROCPPConstraint_Ptr operator==(double real, ROCPPCstrTerm_Ptr term);



// EXPR + ...
// expr <= var
ROCPPConstraint_Ptr operator<=(ROCPPExpr_Ptr expr, ROCPPVarIF_Ptr var);
// expr>= var
ROCPPConstraint_Ptr operator>=(ROCPPExpr_Ptr expr, ROCPPVarIF_Ptr var);
// expr == var
ROCPPConstraint_Ptr operator==(ROCPPExpr_Ptr expr, ROCPPVarIF_Ptr var);

// var <= expr
ROCPPConstraint_Ptr operator<=(ROCPPVarIF_Ptr var, ROCPPExpr_Ptr expr);
// var>= expr
ROCPPConstraint_Ptr operator>=(ROCPPVarIF_Ptr var, ROCPPExpr_Ptr expr);
// var == expr
ROCPPConstraint_Ptr operator==(ROCPPVarIF_Ptr var, ROCPPExpr_Ptr expr);


// expr <= unc
ROCPPConstraint_Ptr operator<=(ROCPPExpr_Ptr expr, ROCPPUnc_Ptr unc);
// expr>= unc
ROCPPConstraint_Ptr operator>=(ROCPPExpr_Ptr expr, ROCPPUnc_Ptr unc);
// expr == unc
ROCPPConstraint_Ptr operator==(ROCPPExpr_Ptr expr, ROCPPUnc_Ptr unc);

// unc <= expr
ROCPPConstraint_Ptr operator<=(ROCPPUnc_Ptr unc, ROCPPExpr_Ptr expr);
// unc >= expr
ROCPPConstraint_Ptr operator>=(ROCPPUnc_Ptr unc, ROCPPExpr_Ptr expr);
// unc == expr
ROCPPConstraint_Ptr operator==(ROCPPUnc_Ptr unc, ROCPPExpr_Ptr expr);


// expr <= expr
ROCPPConstraint_Ptr operator<=(ROCPPExpr_Ptr expr1, ROCPPExpr_Ptr expr2);
// expr>= expr
ROCPPConstraint_Ptr operator>=(ROCPPExpr_Ptr expr1, ROCPPExpr_Ptr expr2);
// expr == expr
ROCPPConstraint_Ptr operator==(ROCPPExpr_Ptr expr1, ROCPPExpr_Ptr expr2);


// term <= expr
ROCPPConstraint_Ptr operator<=(ROCPPCstrTerm_Ptr term, ROCPPExpr_Ptr expr);
// term >= expr
ROCPPConstraint_Ptr operator>=(ROCPPCstrTerm_Ptr term, ROCPPExpr_Ptr expr);
// term == expr
ROCPPConstraint_Ptr operator==(ROCPPCstrTerm_Ptr term, ROCPPExpr_Ptr expr);

// expr <= term
ROCPPConstraint_Ptr operator<=(ROCPPExpr_Ptr expr, ROCPPCstrTerm_Ptr term);
// expr>= term
ROCPPConstraint_Ptr operator>=(ROCPPExpr_Ptr expr, ROCPPCstrTerm_Ptr term);
// expr == term
ROCPPConstraint_Ptr operator==(ROCPPExpr_Ptr expr, ROCPPCstrTerm_Ptr term);



// VAR + ...
// var <= var
ROCPPConstraint_Ptr operator<=(ROCPPVarIF_Ptr var1, ROCPPVarIF_Ptr var2);
// var>= var
ROCPPConstraint_Ptr operator>=(ROCPPVarIF_Ptr var1, ROCPPVarIF_Ptr var2);
// var == var
ROCPPConstraint_Ptr operator==(ROCPPVarIF_Ptr var1, ROCPPVarIF_Ptr var2);


// var <= unc
ROCPPConstraint_Ptr operator<=(ROCPPVarIF_Ptr var, ROCPPUnc_Ptr unc);
// var>= unc
ROCPPConstraint_Ptr operator>=(ROCPPVarIF_Ptr var, ROCPPUnc_Ptr unc);
// var == unc
ROCPPConstraint_Ptr operator==(ROCPPVarIF_Ptr var, ROCPPUnc_Ptr unc);

// unc <= var
ROCPPConstraint_Ptr operator<=(ROCPPUnc_Ptr unc, ROCPPVarIF_Ptr var);
// unc >= var
ROCPPConstraint_Ptr operator>=(ROCPPUnc_Ptr unc, ROCPPVarIF_Ptr var);
// unc == var
ROCPPConstraint_Ptr operator==(ROCPPUnc_Ptr unc, ROCPPVarIF_Ptr var);


// term <= var
ROCPPConstraint_Ptr operator<=(ROCPPCstrTerm_Ptr term, ROCPPVarIF_Ptr var);
// term >= var
ROCPPConstraint_Ptr operator>=(ROCPPCstrTerm_Ptr term, ROCPPVarIF_Ptr var);
// term == var
ROCPPConstraint_Ptr operator==(ROCPPCstrTerm_Ptr term, ROCPPVarIF_Ptr var);

// var <= term
ROCPPConstraint_Ptr operator<=(ROCPPVarIF_Ptr var, ROCPPCstrTerm_Ptr term);
// var>= term
ROCPPConstraint_Ptr operator>=(ROCPPVarIF_Ptr var, ROCPPCstrTerm_Ptr term);
// var == term
ROCPPConstraint_Ptr operator==(ROCPPVarIF_Ptr var, ROCPPCstrTerm_Ptr term);



// UNC + ...

// unc <= unc
ROCPPConstraint_Ptr operator<=(ROCPPUnc_Ptr unc1, ROCPPUnc_Ptr unc2);
// unc >= unc
ROCPPConstraint_Ptr operator>=(ROCPPUnc_Ptr unc1, ROCPPUnc_Ptr unc2);
// unc == unc
ROCPPConstraint_Ptr operator==(ROCPPUnc_Ptr unc1, ROCPPUnc_Ptr unc2);


// term <= unc
ROCPPConstraint_Ptr operator<=(ROCPPCstrTerm_Ptr term, ROCPPUnc_Ptr unc);
// term >= unc
ROCPPConstraint_Ptr operator>=(ROCPPCstrTerm_Ptr term, ROCPPUnc_Ptr unc);
// term == unc
ROCPPConstraint_Ptr operator==(ROCPPCstrTerm_Ptr term, ROCPPUnc_Ptr unc);

// unc <= term
ROCPPConstraint_Ptr operator<=(ROCPPUnc_Ptr unc, ROCPPCstrTerm_Ptr term);
// unc >= term
ROCPPConstraint_Ptr operator>=(ROCPPUnc_Ptr unc, ROCPPCstrTerm_Ptr term);
// unc == term
ROCPPConstraint_Ptr operator==(ROCPPUnc_Ptr unc, ROCPPCstrTerm_Ptr term);




#endif /* Interface_hpp */
