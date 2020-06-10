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
#include "VariableConverter.hpp"
#include "DecisionRule.hpp"
#include "OptModelConverters.hpp"
#include "RobustifyEngine.hpp"
#include "FileRelations.hpp"
#include "DDUApproximator.hpp"
#include "SolverModeller.hpp"
#include "GurobiModeller.hpp"
#include "Interface.hpp"


// real + var
boost::shared_ptr<LHSExpression> operator+(double real, boost::shared_ptr<DecisionVariableIF> var)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(real);
    expr->add(1.,var);
    return expr;
}

// real-var
boost::shared_ptr<LHSExpression> operator-(double real, boost::shared_ptr<DecisionVariableIF> var)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(real);
    expr->add(-1.,var);
    return expr;
}


// real + unc
boost::shared_ptr<LHSExpression> operator+(double real, boost::shared_ptr<UncertaintyIF> unc)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(real);
    expr->add(1.,unc);
    return expr;
}

// real - unc
boost::shared_ptr<LHSExpression> operator-(double real, boost::shared_ptr<UncertaintyIF> unc)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(real);
    expr->add(-1.,unc);
    return expr;
}


// real + expr
boost::shared_ptr<LHSExpression> operator+(double real, boost::shared_ptr<const LHSExpression> expr)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    exprout->add(real);
    exprout->add(expr);
    return exprout;
}

// real - expr
boost::shared_ptr<LHSExpression> operator-(double real, boost::shared_ptr<const LHSExpression> expr)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    exprout->add(real);
    exprout->add(-1. * expr);
    return exprout;
}

// real + term
boost::shared_ptr<LHSExpression> operator+(double real, boost::shared_ptr<const ConstraintTermIF> term)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    exprout->add(real);
    exprout->add(term);
    return exprout;
}

// real - term
boost::shared_ptr<LHSExpression> operator-(double real, boost::shared_ptr<const ConstraintTermIF> term)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    exprout->add(term);
    exprout = -1.*exprout;
    exprout->add(real);
    return exprout;
}


// var + var
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<DecisionVariableIF> var1, boost::shared_ptr<DecisionVariableIF> var2)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(1.0, var1);
    expr->add(1.0, var2);
    return expr;
}

// var - var
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<DecisionVariableIF> var1, boost::shared_ptr<DecisionVariableIF> var2)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(1.0, var1);
    expr->add(-1.0, var2);
    return expr;
}


// var + real
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<DecisionVariableIF> var, double real)
{
    return (real+var);
}

// var - real
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<DecisionVariableIF> var, double real)
{
    return (-1.*(real-var));
}


// var + unc
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<UncertaintyIF> unc)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(1.0, var);
    expr->add(1.0, unc);
    return expr;
}

// var - unc
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<UncertaintyIF> unc)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(1.0, var);
    expr->add(-1.0, unc);
    return expr;
}

// var + expr
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<const LHSExpression> expr)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    exprout->add(1.,var);
    exprout->add(expr);
    return exprout;
}

// var - expr
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<const LHSExpression> expr)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    exprout->add(1.,var);
    exprout->add(-1. * expr);
    return exprout;
}


// var + term
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<const ConstraintTermIF> term)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(1.0, var);
    expr->add(term);
    return expr;
}


// var - term
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<const ConstraintTermIF> term)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    exprout->add(term);
    exprout = -1.*exprout;
    exprout->add(1.,var);
    return exprout;
}



// unc + var
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<DecisionVariableIF> var)
{
    return (var+unc);
}

// unc - var
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<DecisionVariableIF> var)
{
    return (-1.*(var-unc));
}


// unc + real
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<UncertaintyIF> unc, double real)
{
    return (real+unc);
}


// unc - real
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<UncertaintyIF> unc, double real)
{
    return (-1.*(real-unc));
}


// unc + unc
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<UncertaintyIF> unc1, boost::shared_ptr<UncertaintyIF> unc2)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(1.,unc1);
    expr->add(1.,unc2);
    return expr;
}

// unc - unc
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<UncertaintyIF> unc1, boost::shared_ptr<UncertaintyIF> unc2)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(1.,unc1);
    expr->add(-1.,unc2);
    return expr;
}

// unc + expr
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<const LHSExpression> expr)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    exprout->add(1.,unc);
    exprout->add(expr);
    return exprout;
}

// unc - expr
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<const LHSExpression> expr)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    exprout->add(1.,unc);
    exprout->add(-1. * expr);
    return exprout;
}

// unc + term
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<const ConstraintTermIF> term)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(1.0, unc);
    expr->add(term);
    return expr;
}


// unc - term
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<const ConstraintTermIF> term)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    exprout->add(term);
    exprout = -1.*exprout;
    exprout->add(1.,unc);
    return exprout;
}


// expr + var
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const LHSExpression> expr, boost::shared_ptr<DecisionVariableIF> var)
{
    return (var+expr);
}

// expr - var
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const LHSExpression> expr, boost::shared_ptr<DecisionVariableIF> var)
{
    return (-1.*(var-expr));
}

// expr + real
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const LHSExpression> expr, double real)
{
    return (real+expr);
}

// expr - real
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const LHSExpression> expr, double real)
{
    return (-1.*(real-expr));
}

// expr + unc
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const LHSExpression> expr, boost::shared_ptr<UncertaintyIF> unc)
{
    return (unc+expr);
}

// expr - unc
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const LHSExpression> expr, boost::shared_ptr<UncertaintyIF> unc)
{
    return (-1.*(unc-expr));
}

// expr + expr
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const LHSExpression> expr1, boost::shared_ptr<const LHSExpression> expr2)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    exprout->add(expr1);
    exprout->add(expr2);
    return exprout;
}

// expr - expr
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const LHSExpression> expr1, boost::shared_ptr<const LHSExpression> expr2)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    exprout->add(expr1);
    exprout->add(-1.*expr2);
    return exprout;
}


// expr + term
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const LHSExpression> expr, boost::shared_ptr<const ConstraintTermIF> term)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    exprout->add(expr);
    exprout->add(term);
    return exprout;
}

// expr - term
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const LHSExpression> expr, boost::shared_ptr<const ConstraintTermIF> term)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    exprout->add(term);
    exprout = -1.*exprout;
    exprout->add(expr);
    return exprout;
}


// term + var
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const ConstraintTermIF> term, boost::shared_ptr<DecisionVariableIF> var)
{
    return (var+term);
}


// term - var
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const ConstraintTermIF> term, boost::shared_ptr<DecisionVariableIF> var)
{
    return (-1.*(var-term));
}


// term + real
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const ConstraintTermIF> term, double real)
{
    return (real+term);
}


// term - real
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const ConstraintTermIF> term, double real)
{
    return (-1.*(real-term));
}


// term + unc
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const ConstraintTermIF> term, boost::shared_ptr<UncertaintyIF> unc)
{
    return (unc+term);
}


// term - unc
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const ConstraintTermIF> term, boost::shared_ptr<UncertaintyIF> unc)
{
    return (-1.*(unc-term));
}


// term + expr
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const ConstraintTermIF> term, boost::shared_ptr<const LHSExpression> expr)
{
    return (expr+term);
}


// term - expr
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const ConstraintTermIF> term, boost::shared_ptr<const LHSExpression> expr)
{
    return (-1.*(expr-term));
}

// term + term
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const ConstraintTermIF> term1, boost::shared_ptr<const ConstraintTermIF> term2)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    exprout->add(term1);
    exprout->add(term2);
    return exprout;
}

// term - term
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const ConstraintTermIF> term1, boost::shared_ptr<const ConstraintTermIF> term2)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    exprout->add(term2);
    exprout = -1.*exprout;
    exprout->add(term1);
    return exprout;
}


// real * var
boost::shared_ptr<LHSExpression> operator*(double real, boost::shared_ptr<DecisionVariableIF> var)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(real, var);
    
    return expr;
}

// real * unc
boost::shared_ptr<LHSExpression> operator*(double real, boost::shared_ptr<UncertaintyIF> unc)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(real, unc);
    
    return expr;
}

// var1 * var2
boost::shared_ptr<LHSExpression> operator*(boost::shared_ptr<DecisionVariableIF> var1, boost::shared_ptr<DecisionVariableIF> var2)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(1.0, var1, var2);
    
    return expr;
}

// var * unc
boost::shared_ptr<LHSExpression> operator*(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<UncertaintyIF> unc)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(1.0, unc, var);
    
    return expr;
}

// real * expr
boost::shared_ptr<LHSExpression> operator*(double real, boost::shared_ptr<const LHSExpression> expr)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    
    exprout->add(real, expr);
    
    return exprout;
}

// var * expr
boost::shared_ptr<LHSExpression> operator*(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<LHSExpression> expr)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    
    exprout->add(1.0, expr, var);
    
    return exprout;
}


// unc * var
boost::shared_ptr<LHSExpression> operator*(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<DecisionVariableIF> var)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
        
    exprout->add(1.0, unc, var);
    
    return exprout;
}


// unc * expr
boost::shared_ptr<LHSExpression> operator*(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<LHSExpression> expr)
{
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    
    exprout->add(1.0, expr, unc);
    
    return exprout;
}

// expr * unc
boost::shared_ptr<LHSExpression> operator*(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<UncertaintyIF> unc)
{
    return unc*expr;
}

// expr * var
boost::shared_ptr<LHSExpression> operator*(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<DecisionVariableIF> var)
{
    return var*expr;
}

// expr * real
boost::shared_ptr<LHSExpression> operator*(boost::shared_ptr<const LHSExpression> expr, double real)
{
    return real*expr;
}

// real * term
boost::shared_ptr<LHSExpression> operator*(double real, boost::shared_ptr<const ConstraintTermIF> term)
{
    if (term->isNormTerm())
        throw MyException("cannot multiply norm term by a constant; consider normalizing the other terms");
    
    boost::shared_ptr<LHSExpression> exprout(new LHSExpression());
    exprout->add(term);
    exprout = real*exprout;
    
    return exprout;
}


// expr <= real
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<LHSExpression> expr, double real)
{
    boost::shared_ptr<ConstraintIF> newConst = createConstraint(expr, real, false);
    
    return newConst;
}

// expr >= real
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<LHSExpression> expr, double real)
{
    if (expr->hasNormTerm())
        throw MyException("this constraint is not convex");
    
    return ( -1.*expr <= -real );
}

// expr == cont
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<LHSExpression> expr, double cont)
{
    if (expr->hasNormTerm())
        throw MyException("this constraint is not convex");
    
    boost::shared_ptr<ConstraintIF> newConst;
    
    newConst = createConstraint(expr, cont, true);
    
    return newConst;
}


// real <= expr
boost::shared_ptr<ConstraintIF> operator<=(double real, boost::shared_ptr<LHSExpression> expr)
{
    if (expr->hasNormTerm())
        throw MyException("this constraint is not convex");
    return (-1.*expr <= -1.*real);
}

// real >= expr
boost::shared_ptr<ConstraintIF> operator>=(double real, boost::shared_ptr<LHSExpression> expr)
{
    return ( expr <= real);
}

// real == expr
boost::shared_ptr<ConstraintIF> operator==(double real, boost::shared_ptr<LHSExpression> expr)
{
    if (expr->hasNormTerm())
        throw MyException("this constraint is not convex");
    return (expr==real);
}




// var <= real
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<DecisionVariableIF> var, double real)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(1.0, var);
    
    boost::shared_ptr<ConstraintIF> newConst = createConstraint(expr, real, false);
    
    return newConst;
}

// var >= real
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<DecisionVariableIF> var, double real)
{
    return ( -1.*var <= -1.*real );
}

// var == real
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<DecisionVariableIF> var, double real)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(1.0, var);
    
    boost::shared_ptr<ConstraintIF> newConst = createConstraint(expr, real, true);
    
    return newConst;
}


// real <= var
boost::shared_ptr<ConstraintIF> operator<=(double real, boost::shared_ptr<DecisionVariableIF> var)
{
    return (-1.*var <= real);
}

// real >= var
boost::shared_ptr<ConstraintIF> operator>=(double real, boost::shared_ptr<DecisionVariableIF> var)
{
    return (-1.*var >= -1.*real);
}

// real == var
boost::shared_ptr<ConstraintIF> operator==(double real, boost::shared_ptr<DecisionVariableIF> var)
{
    return (var==real);
}




// unc <= real
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<UncertaintyIF> unc, double real)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(1.0, unc);
    
    boost::shared_ptr<ConstraintIF> newConst = createConstraint(expr, real, false);
    
    return newConst;
}

// unc >= real
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<UncertaintyIF> unc, double real)
{
    return ( -1.*unc <= -1.*real );
}


// unc == real
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<UncertaintyIF> unc, double real)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(1.0, unc);
    
    boost::shared_ptr<ConstraintIF> newConst = createConstraint(expr, real, true);
    
    return newConst;
}

// real <= unc
boost::shared_ptr<ConstraintIF> operator<=(double real, boost::shared_ptr<UncertaintyIF> unc)
{
    return (-1.*unc <= -real);
}

// real >= unc
boost::shared_ptr<ConstraintIF> operator>=(double real, boost::shared_ptr<UncertaintyIF> unc)
{
    return ( unc <= real );
}

// real == unc
boost::shared_ptr<ConstraintIF> operator==(double real, boost::shared_ptr<UncertaintyIF> unc)
{
    return (unc==real);
}




// term <= real
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<ConstraintTermIF> term, double real)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(term);
    
    boost::shared_ptr<ConstraintIF> newConst = createConstraint(expr, real, false);
    
    return newConst;
}

// term >= real
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<ConstraintTermIF> term, double real)
{
    if (term->isNormTerm())
        throw MyException("this constraint is not convex");
    
    return ( -1.*term <= -real );
}

// term == real
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<ConstraintTermIF> term, double real)
{
    boost::shared_ptr<LHSExpression> expr(new LHSExpression());
    expr->add(term);
    
    boost::shared_ptr<ConstraintIF> newConst = createConstraint(expr, real, true);
    
    return newConst;
}

// real <= term
boost::shared_ptr<ConstraintIF> operator<=(double real, boost::shared_ptr<ConstraintTermIF> term)
{
    if (term->isNormTerm())
        throw MyException("this constraint is not convex");
    
    return (-1.*term <= -real);
}

// real >= term
boost::shared_ptr<ConstraintIF> operator>=(double real, boost::shared_ptr<ConstraintTermIF> term)
{
    return ( term <= real );
}

// real == term
boost::shared_ptr<ConstraintIF> operator==(double real, boost::shared_ptr<ConstraintTermIF> term)
{
    return (term==real);
}






// expr <= var
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<DecisionVariableIF> var)
{
    return (expr-var <= 0.);
}

// expr >= var
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<DecisionVariableIF> var)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (expr-var >=0.);
}

// expr == var
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<DecisionVariableIF> var)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (expr-var ==0.);
}

// var <= expr
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<LHSExpression> expr)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (var-expr <=0.);
}

// var >= expr
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<LHSExpression> expr)
{
    return ( expr - var <= 0.);
}

// var == expr
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<LHSExpression> expr)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (var-expr ==0.);
}



// expr <= unc
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<UncertaintyIF> unc)
{
    return (expr-unc <= 0.);
}

// expr >= unc
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<UncertaintyIF> unc)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (expr-unc >=0.);
}

// expr == unc
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<UncertaintyIF> unc)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (expr-unc==0.);
}

// unc <= expr
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<LHSExpression> expr)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (unc-expr<=0.);
}

// unc >= expr
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<LHSExpression> expr)
{
    return ( expr <= unc);
}

// unc == expr
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<LHSExpression> expr)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (unc-expr==0.);
}




// expr <= expr
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<LHSExpression> expr1, boost::shared_ptr<LHSExpression> expr2)
{
    if (expr2->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (expr1-expr2 <= 0.);
}

// expr >= expr
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<LHSExpression> expr1, boost::shared_ptr<LHSExpression> expr2)
{
    if (expr1->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (expr1-expr2 >= 0.);
}

// expr == expr
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<LHSExpression> expr1, boost::shared_ptr<LHSExpression> expr2)
{
    if ( (expr1->hasNormTerm()) || (expr2->hasNormTerm()) )
        throw MyException("constraint is non-convex");
    
    return (expr1-expr2 == 0.);
}


// term <= expr
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<LHSExpression> expr)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (term-expr <=0.);
}

// term >= expr
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<LHSExpression> expr)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (term -expr >=0.);
}

// term == expr
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<LHSExpression> expr)
{
    if ( (expr->hasNormTerm()) || (term->isNormTerm()) )
        throw MyException("constraint is non-convex");
    
    return (term-expr== 0.);
}

// expr <= term
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<ConstraintTermIF> term)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (expr-term <=0.);
}

// expr >= term
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<ConstraintTermIF> term)
{
    if (expr->hasNormTerm())
        throw MyException("constraint is non-convex");
    
    return (expr-term >=0.);
}

// expr == term
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<ConstraintTermIF> term)
{
    if ( (expr->hasNormTerm()) || (term->isNormTerm()) )
        throw MyException("constraint is non-convex");
    
    return (expr-term==0.);
}








// var <= var
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<DecisionVariableIF> var1, boost::shared_ptr<DecisionVariableIF> var2)
{
    return (var1-var2 <=0.);
}

// var >= var
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<DecisionVariableIF> var1, boost::shared_ptr<DecisionVariableIF> var2)
{
    return (var1-var2 >=0.);
}

// var == var
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<DecisionVariableIF> var1, boost::shared_ptr<DecisionVariableIF> var2)
{
    return (var1-var2 ==0.);
}

// var <= unc
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<UncertaintyIF> unc)
{
    return (var-unc <=0.);
}

// var >= unc
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<UncertaintyIF> unc)
{
    return (var-unc>=0.);
}

// var == unc
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<UncertaintyIF> unc)
{
    return (var-unc==0.);
}

// unc <= var
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<DecisionVariableIF> var)
{
    return (unc-var <=0.);
}

// unc >= var
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<DecisionVariableIF> var)
{
    return (unc-var>=0.);
}

// unc == var
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<DecisionVariableIF> var)
{
    return (unc-var==0.);
}



// term <= var
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<DecisionVariableIF> var)
{
    return (term-var <=0.);
}

// term >= var
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<DecisionVariableIF> var)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (term-var >=0.);
}

// term == var
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<DecisionVariableIF> var)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (term-var==0.);
}

// var <= term
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<ConstraintTermIF> term)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (var-term <=0.);
}

// var >= term
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<ConstraintTermIF> term)
{
    return ( term - var <=0.);
}

// var == term
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<ConstraintTermIF> term)
{
    if ( term->isNormTerm() )
        throw MyException("constraint is non-convex");
    
    return (var-term==0.);
}


// unc <= unc
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<UncertaintyIF> unc1, boost::shared_ptr<UncertaintyIF> unc2)
{
    return (unc1-unc2 <= 0.);
}

// unc >= unc
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<UncertaintyIF> unc1, boost::shared_ptr<UncertaintyIF> unc2)
{
    return (unc1-unc2 >=0.);
}

// unc == unc
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<UncertaintyIF> unc1, boost::shared_ptr<UncertaintyIF> unc2)
{
    return (unc1-unc2 == 0.);
}



// term <= unc
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<UncertaintyIF> unc)
{
    return (term-unc<=0.);
}

// term >= unc
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<UncertaintyIF> unc)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (term -unc >=0.);
}

// term == unc
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<UncertaintyIF> unc)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (term-unc==0.);
}

// unc <= term
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<ConstraintTermIF> term)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (unc-term <=0.);
}

// unc >= term
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<ConstraintTermIF> term)
{
    return (term - unc <=0.);
}

// unc == term
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<ConstraintTermIF> term)
{
    if (term->isNormTerm())
        throw MyException("constraint is non-convex");
    
    return (unc-term==0.);
}









