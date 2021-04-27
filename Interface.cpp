//
//  Interface.cpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 The University of Southern California. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#include "IncludeFiles.hpp"
#include "DecisionVariable.hpp"
#include "Uncertainty.hpp"
#include "ConstraintTerm.hpp"
#include "Constraint.hpp"
#include "ObjectiveFunction.hpp"
#include "OptimizationModel.hpp"
#include "ReformulationOrchestrator.hpp"
#include "VariableConverter.hpp"
#include "DecisionRule.hpp"
#include "OptModelConverters.hpp"
#include "RobustifyEngine.hpp"
#include "FileRelations.hpp"
#include "SolverModeller.hpp"
#include "GurobiModeller.hpp"
#include "Interface.hpp"


// real + var
ROCPPExpr_Ptr operator+(double real, ROCPPVarIF_Ptr var)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(real);
    expr->add(1.,var);
    return expr;
}

// real-var
ROCPPExpr_Ptr operator-(double real, ROCPPVarIF_Ptr var)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(real);
    expr->add(-1.,var);
    return expr;
}


// real + unc
ROCPPExpr_Ptr operator+(double real, ROCPPUnc_Ptr unc)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(real);
    expr->add(1.,unc);
    return expr;
}

// real - unc
ROCPPExpr_Ptr operator-(double real, ROCPPUnc_Ptr unc)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(real);
    expr->add(-1.,unc);
    return expr;
}


// real + expr
ROCPPExpr_Ptr operator+(double real, ROCPPconstExpr_Ptr expr)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    exprout->add(real);
    exprout->add(expr);
    return exprout;
}

// real - expr
ROCPPExpr_Ptr operator-(double real, ROCPPconstExpr_Ptr expr)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    exprout->add(real);
    exprout->add(-1. * expr);
    return exprout;
}

// real + term
ROCPPExpr_Ptr operator+(double real, ROCPPconstCstrTerm_Ptr term)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    exprout->add(real);
    exprout->add(term);
    return exprout;
}

// real - term
ROCPPExpr_Ptr operator-(double real, ROCPPconstCstrTerm_Ptr term)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    exprout->add(term);
    exprout = -1.*exprout;
    exprout->add(real);
    return exprout;
}


// var + var
ROCPPExpr_Ptr operator+(ROCPPVarIF_Ptr var1, ROCPPVarIF_Ptr var2)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(1.0, var1);
    expr->add(1.0, var2);
    return expr;
}

// var - var
ROCPPExpr_Ptr operator-(ROCPPVarIF_Ptr var1, ROCPPVarIF_Ptr var2)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(1.0, var1);
    expr->add(-1.0, var2);
    return expr;
}


// var + real
ROCPPExpr_Ptr operator+(ROCPPVarIF_Ptr var, double real)
{
    return (real+var);
}

// var - real
ROCPPExpr_Ptr operator-(ROCPPVarIF_Ptr var, double real)
{
    return (-1.*(real-var));
}


// var + unc
ROCPPExpr_Ptr operator+(ROCPPVarIF_Ptr var, ROCPPUnc_Ptr unc)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(1.0, var);
    expr->add(1.0, unc);
    return expr;
}

// var - unc
ROCPPExpr_Ptr operator-(ROCPPVarIF_Ptr var, ROCPPUnc_Ptr unc)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(1.0, var);
    expr->add(-1.0, unc);
    return expr;
}

// var + expr
ROCPPExpr_Ptr operator+(ROCPPVarIF_Ptr var, ROCPPconstExpr_Ptr expr)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    exprout->add(1.,var);
    exprout->add(expr);
    return exprout;
}

// var - expr
ROCPPExpr_Ptr operator-(ROCPPVarIF_Ptr var, ROCPPconstExpr_Ptr expr)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    exprout->add(1.,var);
    exprout->add(-1. * expr);
    return exprout;
}


// var + term
ROCPPExpr_Ptr operator+(ROCPPVarIF_Ptr var, ROCPPconstCstrTerm_Ptr term)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(1.0, var);
    expr->add(term);
    return expr;
}


// var - term
ROCPPExpr_Ptr operator-(ROCPPVarIF_Ptr var, ROCPPconstCstrTerm_Ptr term)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    exprout->add(term);
    exprout = -1.*exprout;
    exprout->add(1.,var);
    return exprout;
}



// unc + var
ROCPPExpr_Ptr operator+(ROCPPUnc_Ptr unc, ROCPPVarIF_Ptr var)
{
    return (var+unc);
}

// unc - var
ROCPPExpr_Ptr operator-(ROCPPUnc_Ptr unc, ROCPPVarIF_Ptr var)
{
    return (-1.*(var-unc));
}


// unc + real
ROCPPExpr_Ptr operator+(ROCPPUnc_Ptr unc, double real)
{
    return (real+unc);
}


// unc - real
ROCPPExpr_Ptr operator-(ROCPPUnc_Ptr unc, double real)
{
    return (-1.*(real-unc));
}


// unc + unc
ROCPPExpr_Ptr operator+(ROCPPUnc_Ptr unc1, ROCPPUnc_Ptr unc2)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(1.,unc1);
    expr->add(1.,unc2);
    return expr;
}

// unc - unc
ROCPPExpr_Ptr operator-(ROCPPUnc_Ptr unc1, ROCPPUnc_Ptr unc2)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(1.,unc1);
    expr->add(-1.,unc2);
    return expr;
}

// unc + expr
ROCPPExpr_Ptr operator+(ROCPPUnc_Ptr unc, ROCPPconstExpr_Ptr expr)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    exprout->add(1.,unc);
    exprout->add(expr);
    return exprout;
}

// unc - expr
ROCPPExpr_Ptr operator-(ROCPPUnc_Ptr unc, ROCPPconstExpr_Ptr expr)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    exprout->add(1.,unc);
    exprout->add(-1. * expr);
    return exprout;
}

// unc + term
ROCPPExpr_Ptr operator+(ROCPPUnc_Ptr unc, ROCPPconstCstrTerm_Ptr term)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(1.0, unc);
    expr->add(term);
    return expr;
}


// unc - term
ROCPPExpr_Ptr operator-(ROCPPUnc_Ptr unc, ROCPPconstCstrTerm_Ptr term)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    exprout->add(term);
    exprout = -1.*exprout;
    exprout->add(1.,unc);
    return exprout;
}


// expr + var
ROCPPExpr_Ptr operator+(ROCPPconstExpr_Ptr expr, ROCPPVarIF_Ptr var)
{
    return (var+expr);
}

// expr - var
ROCPPExpr_Ptr operator-(ROCPPconstExpr_Ptr expr, ROCPPVarIF_Ptr var)
{
    return (-1.*(var-expr));
}

// expr + real
ROCPPExpr_Ptr operator+(ROCPPconstExpr_Ptr expr, double real)
{
    return (real+expr);
}

// expr - real
ROCPPExpr_Ptr operator-(ROCPPconstExpr_Ptr expr, double real)
{
    return (-1.*(real-expr));
}

// expr + unc
ROCPPExpr_Ptr operator+(ROCPPconstExpr_Ptr expr, ROCPPUnc_Ptr unc)
{
    return (unc+expr);
}

// expr - unc
ROCPPExpr_Ptr operator-(ROCPPconstExpr_Ptr expr, ROCPPUnc_Ptr unc)
{
    return (-1.*(unc-expr));
}

// expr + expr
ROCPPExpr_Ptr operator+(ROCPPconstExpr_Ptr expr1, ROCPPconstExpr_Ptr expr2)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    exprout->add(expr1);
    exprout->add(expr2);
    return exprout;
}

// expr - expr
ROCPPExpr_Ptr operator-(ROCPPconstExpr_Ptr expr1, ROCPPconstExpr_Ptr expr2)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    exprout->add(expr1);
    exprout->add(-1.*expr2);
    return exprout;
}


// expr + term
ROCPPExpr_Ptr operator+(ROCPPconstExpr_Ptr expr, ROCPPconstCstrTerm_Ptr term)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    exprout->add(expr);
    exprout->add(term);
    return exprout;
}

// expr - term
ROCPPExpr_Ptr operator-(ROCPPconstExpr_Ptr expr, ROCPPconstCstrTerm_Ptr term)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    exprout->add(term);
    exprout = -1.*exprout;
    exprout->add(expr);
    return exprout;
}


// term + var
ROCPPExpr_Ptr operator+(ROCPPconstCstrTerm_Ptr term, ROCPPVarIF_Ptr var)
{
    return (var+term);
}


// term - var
ROCPPExpr_Ptr operator-(ROCPPconstCstrTerm_Ptr term, ROCPPVarIF_Ptr var)
{
    return (-1.*(var-term));
}


// term + real
ROCPPExpr_Ptr operator+(ROCPPconstCstrTerm_Ptr term, double real)
{
    return (real+term);
}


// term - real
ROCPPExpr_Ptr operator-(ROCPPconstCstrTerm_Ptr term, double real)
{
    return (-1.*(real-term));
}


// term + unc
ROCPPExpr_Ptr operator+(ROCPPconstCstrTerm_Ptr term, ROCPPUnc_Ptr unc)
{
    return (unc+term);
}


// term - unc
ROCPPExpr_Ptr operator-(ROCPPconstCstrTerm_Ptr term, ROCPPUnc_Ptr unc)
{
    return (-1.*(unc-term));
}


// term + expr
ROCPPExpr_Ptr operator+(ROCPPconstCstrTerm_Ptr term, ROCPPconstExpr_Ptr expr)
{
    return (expr+term);
}


// term - expr
ROCPPExpr_Ptr operator-(ROCPPconstCstrTerm_Ptr term, ROCPPconstExpr_Ptr expr)
{
    return (-1.*(expr-term));
}

// term + term
ROCPPExpr_Ptr operator+(ROCPPconstCstrTerm_Ptr term1, ROCPPconstCstrTerm_Ptr term2)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    exprout->add(term1);
    exprout->add(term2);
    return exprout;
}

// term - term
ROCPPExpr_Ptr operator-(ROCPPconstCstrTerm_Ptr term1, ROCPPconstCstrTerm_Ptr term2)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    exprout->add(term2);
    exprout = -1.*exprout;
    exprout->add(term1);
    return exprout;
}


// real * var
ROCPPExpr_Ptr operator*(double real, ROCPPVarIF_Ptr var)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(real, var);
    
    return expr;
}

// real * unc
ROCPPExpr_Ptr operator*(double real, ROCPPUnc_Ptr unc)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(real, unc);
    
    return expr;
}

// var1 * var2
ROCPPExpr_Ptr operator*(ROCPPVarIF_Ptr var1, ROCPPVarIF_Ptr var2)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(1.0, var1, var2);
    
    return expr;
}

// var * unc
ROCPPExpr_Ptr operator*(ROCPPVarIF_Ptr var, ROCPPUnc_Ptr unc)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(1.0, unc, var);
    
    return expr;
}

// real * expr
ROCPPExpr_Ptr operator*(double real, ROCPPconstExpr_Ptr expr)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    
    exprout->add(real, expr);
    
    return exprout;
}

// var * expr
ROCPPExpr_Ptr operator*(ROCPPVarIF_Ptr var, ROCPPExpr_Ptr expr)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    
    exprout->add(1.0, expr, var);
    
    return exprout;
}


// unc * var
ROCPPExpr_Ptr operator*(ROCPPUnc_Ptr unc, ROCPPVarIF_Ptr var)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
        
    exprout->add(1.0, unc, var);
    
    return exprout;
}


// unc * expr
ROCPPExpr_Ptr operator*(ROCPPUnc_Ptr unc, ROCPPExpr_Ptr expr)
{
    ROCPPExpr_Ptr exprout(new LHSExpression());
    
    exprout->add(1.0, expr, unc);
    
    return exprout;
}

// expr * unc
ROCPPExpr_Ptr operator*(ROCPPExpr_Ptr expr, ROCPPUnc_Ptr unc)
{
    return unc*expr;
}

// expr * var
ROCPPExpr_Ptr operator*(ROCPPExpr_Ptr expr, ROCPPVarIF_Ptr var)
{
    return var*expr;
}

// expr * real
ROCPPExpr_Ptr operator*(ROCPPconstExpr_Ptr expr, double real)
{
    return real*expr;
}

// real * term
ROCPPExpr_Ptr operator*(double real, ROCPPconstCstrTerm_Ptr term)
{
    if (term->isNormTerm())
        throw MyException("cannot multiply norm term by a constant; consider normalizing the other terms");
    
    ROCPPExpr_Ptr exprout(new LHSExpression());
    exprout->add(term);
    exprout = real*exprout;
    
    return exprout;
}


// expr <= real
ROCPPConstraint_Ptr operator<=(ROCPPExpr_Ptr expr, double real)
{
    ROCPPConstraint_Ptr newConst = createConstraint(expr, real, false);
    
    return newConst;
}

// expr>= real
ROCPPConstraint_Ptr operator>=(ROCPPExpr_Ptr expr, double real)
{
    if (expr->hasNormTerm())
        throw MyException("this constraint is not convex");
    
    return ( -1.*expr <= -real );
}

// expr == cont
ROCPPConstraint_Ptr operator==(ROCPPExpr_Ptr expr, double cont)
{
    if (expr->hasNormTerm())
        throw MyException("this constraint is not convex");
    
    ROCPPConstraint_Ptr newConst;
    
    newConst = createConstraint(expr, cont, true);
    
    return newConst;
}


// real <= expr
ROCPPConstraint_Ptr operator<=(double real, ROCPPExpr_Ptr expr)
{
    if (expr->hasNormTerm())
        throw MyException("this constraint is not convex");
    return (-1.*expr <= -1.*real);
}

// real >= expr
ROCPPConstraint_Ptr operator>=(double real, ROCPPExpr_Ptr expr)
{
    return ( expr <= real);
}

// real == expr
ROCPPConstraint_Ptr operator==(double real, ROCPPExpr_Ptr expr)
{
    if (expr->hasNormTerm())
        throw MyException("this constraint is not convex");
    return (expr==real);
}




// var <= real
ROCPPConstraint_Ptr operator<=(ROCPPVarIF_Ptr var, double real)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(1.0, var);
    
    ROCPPConstraint_Ptr newConst = createConstraint(expr, real, false);
    
    return newConst;
}

// var>= real
ROCPPConstraint_Ptr operator>=(ROCPPVarIF_Ptr var, double real)
{
    return ( -1.*var <= -1.*real );
}

// var == real
ROCPPConstraint_Ptr operator==(ROCPPVarIF_Ptr var, double real)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(1.0, var);
    
    ROCPPConstraint_Ptr newConst = createConstraint(expr, real, true);
    
    return newConst;
}


// real <= var
ROCPPConstraint_Ptr operator<=(double real, ROCPPVarIF_Ptr var)
{
    return (-1.*var <= real);
}

// real >= var
ROCPPConstraint_Ptr operator>=(double real, ROCPPVarIF_Ptr var)
{
    return (-1.*var>= -1.*real);
}

// real == var
ROCPPConstraint_Ptr operator==(double real, ROCPPVarIF_Ptr var)
{
    return (var==real);
}




// unc <= real
ROCPPConstraint_Ptr operator<=(ROCPPUnc_Ptr unc, double real)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(1.0, unc);
    
    ROCPPConstraint_Ptr newConst = createConstraint(expr, real, false);
    
    return newConst;
}

// unc >= real
ROCPPConstraint_Ptr operator>=(ROCPPUnc_Ptr unc, double real)
{
    return ( -1.*unc <= -1.*real );
}


// unc == real
ROCPPConstraint_Ptr operator==(ROCPPUnc_Ptr unc, double real)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(1.0, unc);
    
    ROCPPConstraint_Ptr newConst = createConstraint(expr, real, true);
    
    return newConst;
}

// real <= unc
ROCPPConstraint_Ptr operator<=(double real, ROCPPUnc_Ptr unc)
{
    return (-1.*unc <= -real);
}

// real >= unc
ROCPPConstraint_Ptr operator>=(double real, ROCPPUnc_Ptr unc)
{
    return ( unc <= real );
}

// real == unc
ROCPPConstraint_Ptr operator==(double real, ROCPPUnc_Ptr unc)
{
    return (unc==real);
}




// term <= real
ROCPPConstraint_Ptr operator<=(ROCPPCstrTerm_Ptr term, double real)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(term);
    
    ROCPPConstraint_Ptr newConst = createConstraint(expr, real, false);
    
    return newConst;
}

// term >= real
ROCPPConstraint_Ptr operator>=(ROCPPCstrTerm_Ptr term, double real)
{
    if (term->isNormTerm())
        throw MyException("this constraint is not convex");
    
    return ( -1.*term <= -real );
}

// term == real
ROCPPConstraint_Ptr operator==(ROCPPCstrTerm_Ptr term, double real)
{
    ROCPPExpr_Ptr expr(new LHSExpression());
    expr->add(term);
    
    ROCPPConstraint_Ptr newConst = createConstraint(expr, real, true);
    
    return newConst;
}

// real <= term
ROCPPConstraint_Ptr operator<=(double real, ROCPPCstrTerm_Ptr term)
{
    if (term->isNormTerm())
        throw MyException("this constraint is not convex");
    
    return (-1.*term <= -real);
}

// real >= term
ROCPPConstraint_Ptr operator>=(double real, ROCPPCstrTerm_Ptr term)
{
    return ( term <= real );
}

// real == term
ROCPPConstraint_Ptr operator==(double real, ROCPPCstrTerm_Ptr term)
{
    return (term==real);
}






// expr <= var
ROCPPConstraint_Ptr operator<=(ROCPPExpr_Ptr expr, ROCPPVarIF_Ptr var)
{
    return (expr-var <= 0.);
}

// expr>= var
ROCPPConstraint_Ptr operator>=(ROCPPExpr_Ptr expr, ROCPPVarIF_Ptr var)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (expr-var>=0.);
}

// expr == var
ROCPPConstraint_Ptr operator==(ROCPPExpr_Ptr expr, ROCPPVarIF_Ptr var)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (expr-var ==0.);
}

// var <= expr
ROCPPConstraint_Ptr operator<=(ROCPPVarIF_Ptr var, ROCPPExpr_Ptr expr)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (var-expr <=0.);
}

// var>= expr
ROCPPConstraint_Ptr operator>=(ROCPPVarIF_Ptr var, ROCPPExpr_Ptr expr)
{
    return ( expr - var <= 0.);
}

// var == expr
ROCPPConstraint_Ptr operator==(ROCPPVarIF_Ptr var, ROCPPExpr_Ptr expr)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (var-expr ==0.);
}



// expr <= unc
ROCPPConstraint_Ptr operator<=(ROCPPExpr_Ptr expr, ROCPPUnc_Ptr unc)
{
    return (expr-unc <= 0.);
}

// expr>= unc
ROCPPConstraint_Ptr operator>=(ROCPPExpr_Ptr expr, ROCPPUnc_Ptr unc)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (expr-unc >=0.);
}

// expr == unc
ROCPPConstraint_Ptr operator==(ROCPPExpr_Ptr expr, ROCPPUnc_Ptr unc)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (expr-unc==0.);
}

// unc <= expr
ROCPPConstraint_Ptr operator<=(ROCPPUnc_Ptr unc, ROCPPExpr_Ptr expr)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (unc-expr<=0.);
}

// unc >= expr
ROCPPConstraint_Ptr operator>=(ROCPPUnc_Ptr unc, ROCPPExpr_Ptr expr)
{
    return ( expr <= unc);
}

// unc == expr
ROCPPConstraint_Ptr operator==(ROCPPUnc_Ptr unc, ROCPPExpr_Ptr expr)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (unc-expr==0.);
}




// expr <= expr
ROCPPConstraint_Ptr operator<=(ROCPPExpr_Ptr expr1, ROCPPExpr_Ptr expr2)
{
    if (expr2->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (expr1-expr2 <= 0.);
}

// expr>= expr
ROCPPConstraint_Ptr operator>=(ROCPPExpr_Ptr expr1, ROCPPExpr_Ptr expr2)
{
    if (expr1->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (expr1-expr2 >= 0.);
}

// expr == expr
ROCPPConstraint_Ptr operator==(ROCPPExpr_Ptr expr1, ROCPPExpr_Ptr expr2)
{
    if ( (expr1->hasNormTerm()) || (expr2->hasNormTerm()) )
        throw MyException("constraint is non-convex");
    
    return (expr1-expr2 == 0.);
}


// term <= expr
ROCPPConstraint_Ptr operator<=(ROCPPCstrTerm_Ptr term, ROCPPExpr_Ptr expr)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (term-expr <=0.);
}

// term >= expr
ROCPPConstraint_Ptr operator>=(ROCPPCstrTerm_Ptr term, ROCPPExpr_Ptr expr)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (term -expr>=0.);
}

// term == expr
ROCPPConstraint_Ptr operator==(ROCPPCstrTerm_Ptr term, ROCPPExpr_Ptr expr)
{
    if ( (expr->hasNormTerm()) || (term->isNormTerm()) )
        throw MyException("constraint is non-convex");
    
    return (term-expr== 0.);
}

// expr <= term
ROCPPConstraint_Ptr operator<=(ROCPPExpr_Ptr expr, ROCPPCstrTerm_Ptr term)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (expr-term <=0.);
}

// expr>= term
ROCPPConstraint_Ptr operator>=(ROCPPExpr_Ptr expr, ROCPPCstrTerm_Ptr term)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (expr-term >=0.);
}

// expr == term
ROCPPConstraint_Ptr operator==(ROCPPExpr_Ptr expr, ROCPPCstrTerm_Ptr term)
{
    if ( (expr->hasNormTerm()) || (term->isNormTerm()) )
        throw MyException("constraint is non-convex");
    
    return (expr-term==0.);
}








// var <= var
ROCPPConstraint_Ptr operator<=(ROCPPVarIF_Ptr var1, ROCPPVarIF_Ptr var2)
{
    return (var1-var2 <=0.);
}

// var>= var
ROCPPConstraint_Ptr operator>=(ROCPPVarIF_Ptr var1, ROCPPVarIF_Ptr var2)
{
    return (var1-var2 >=0.);
}

// var == var
ROCPPConstraint_Ptr operator==(ROCPPVarIF_Ptr var1, ROCPPVarIF_Ptr var2)
{
    return (var1-var2 ==0.);
}

// var <= unc
ROCPPConstraint_Ptr operator<=(ROCPPVarIF_Ptr var, ROCPPUnc_Ptr unc)
{
    return (var-unc <=0.);
}

// var>= unc
ROCPPConstraint_Ptr operator>=(ROCPPVarIF_Ptr var, ROCPPUnc_Ptr unc)
{
    return (var-unc>=0.);
}

// var == unc
ROCPPConstraint_Ptr operator==(ROCPPVarIF_Ptr var, ROCPPUnc_Ptr unc)
{
    return (var-unc==0.);
}

// unc <= var
ROCPPConstraint_Ptr operator<=(ROCPPUnc_Ptr unc, ROCPPVarIF_Ptr var)
{
    return (unc-var <=0.);
}

// unc >= var
ROCPPConstraint_Ptr operator>=(ROCPPUnc_Ptr unc, ROCPPVarIF_Ptr var)
{
    return (unc-var>=0.);
}

// unc == var
ROCPPConstraint_Ptr operator==(ROCPPUnc_Ptr unc, ROCPPVarIF_Ptr var)
{
    return (unc-var==0.);
}



// term <= var
ROCPPConstraint_Ptr operator<=(ROCPPCstrTerm_Ptr term, ROCPPVarIF_Ptr var)
{
    return (term-var <=0.);
}

// term >= var
ROCPPConstraint_Ptr operator>=(ROCPPCstrTerm_Ptr term, ROCPPVarIF_Ptr var)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (term-var>=0.);
}

// term == var
ROCPPConstraint_Ptr operator==(ROCPPCstrTerm_Ptr term, ROCPPVarIF_Ptr var)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (term-var==0.);
}

// var <= term
ROCPPConstraint_Ptr operator<=(ROCPPVarIF_Ptr var, ROCPPCstrTerm_Ptr term)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (var-term <=0.);
}

// var>= term
ROCPPConstraint_Ptr operator>=(ROCPPVarIF_Ptr var, ROCPPCstrTerm_Ptr term)
{
    return ( term - var <=0.);
}

// var == term
ROCPPConstraint_Ptr operator==(ROCPPVarIF_Ptr var, ROCPPCstrTerm_Ptr term)
{
    if ( term->isNormTerm() )
        throw MyException("constraint is non-convex");
    
    return (var-term==0.);
}


// unc <= unc
ROCPPConstraint_Ptr operator<=(ROCPPUnc_Ptr unc1, ROCPPUnc_Ptr unc2)
{
    return (unc1-unc2 <= 0.);
}

// unc >= unc
ROCPPConstraint_Ptr operator>=(ROCPPUnc_Ptr unc1, ROCPPUnc_Ptr unc2)
{
    return (unc1-unc2 >=0.);
}

// unc == unc
ROCPPConstraint_Ptr operator==(ROCPPUnc_Ptr unc1, ROCPPUnc_Ptr unc2)
{
    return (unc1-unc2 == 0.);
}



// term <= unc
ROCPPConstraint_Ptr operator<=(ROCPPCstrTerm_Ptr term, ROCPPUnc_Ptr unc)
{
    return (term-unc<=0.);
}

// term >= unc
ROCPPConstraint_Ptr operator>=(ROCPPCstrTerm_Ptr term, ROCPPUnc_Ptr unc)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (term -unc >=0.);
}

// term == unc
ROCPPConstraint_Ptr operator==(ROCPPCstrTerm_Ptr term, ROCPPUnc_Ptr unc)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (term-unc==0.);
}

// unc <= term
ROCPPConstraint_Ptr operator<=(ROCPPUnc_Ptr unc, ROCPPCstrTerm_Ptr term)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (unc-term <=0.);
}

// unc >= term
ROCPPConstraint_Ptr operator>=(ROCPPUnc_Ptr unc, ROCPPCstrTerm_Ptr term)
{
    return (term - unc <=0.);
}

// unc == term
ROCPPConstraint_Ptr operator==(ROCPPUnc_Ptr unc, ROCPPCstrTerm_Ptr term)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (unc-term==0.);
}









