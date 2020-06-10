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
class DecisionVariableIF;
class UncertaintyIF;
class ConstraintTermIF;
class LHSExpression;
class ConstraintIF;


// REAL + ...
// real + var
boost::shared_ptr<LHSExpression> operator+(double real, boost::shared_ptr<DecisionVariableIF> var);
// real - var
boost::shared_ptr<LHSExpression> operator-(double real, boost::shared_ptr<DecisionVariableIF> var);

// real + unc
boost::shared_ptr<LHSExpression> operator+(double real, boost::shared_ptr<UncertaintyIF> unc);
// real - unc
boost::shared_ptr<LHSExpression> operator-(double real, boost::shared_ptr<UncertaintyIF> unc);

// real + expr
boost::shared_ptr<LHSExpression> operator+(double real, boost::shared_ptr<const LHSExpression> expr);
// real - expr
boost::shared_ptr<LHSExpression> operator-(double real, boost::shared_ptr<const LHSExpression> expr);

// real + term
boost::shared_ptr<LHSExpression> operator+(double real, boost::shared_ptr<const ConstraintTermIF> term);
// real - term
boost::shared_ptr<LHSExpression> operator-(double real, boost::shared_ptr<const ConstraintTermIF> term);


// VAR + ...
// var1 + var2
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<DecisionVariableIF> var1, boost::shared_ptr<DecisionVariableIF> var2);
// var1 - var2
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<DecisionVariableIF> var1, boost::shared_ptr<DecisionVariableIF> var2);

// var + real
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<DecisionVariableIF> var, double real);
// var - real
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<DecisionVariableIF> var, double real);

// var + unc
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<UncertaintyIF> unc);
// var - unc
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<UncertaintyIF> unc);

// var + expr
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<const LHSExpression> expr);
// var - expr
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<const LHSExpression> expr);

// var + term
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<const ConstraintTermIF> term);
// var - term
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<const ConstraintTermIF> term);


// UNC + ...
// unc + var
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<DecisionVariableIF> var);
// unc - var
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<DecisionVariableIF> var);

// unc + real
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<UncertaintyIF> unc, double real);
// unc - real
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<UncertaintyIF> unc, double real);

// unc + unc
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<UncertaintyIF> unc1);
// unc - unc
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<UncertaintyIF> unc1);

// unc + expr
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<const LHSExpression> expr);
// unc - expr
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<const LHSExpression> expr);

// unc + term
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<const ConstraintTermIF> term);
// unc - term
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<const ConstraintTermIF> term);


// EXPR + ...
// expr + var
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const LHSExpression> expr, boost::shared_ptr<DecisionVariableIF> var);
// expr - var
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const LHSExpression> expr, boost::shared_ptr<DecisionVariableIF> var);

// expr + real
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const LHSExpression> expr, double real);
// expr - real
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const LHSExpression> expr, double real);

// expr + unc
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const LHSExpression> expr, boost::shared_ptr<UncertaintyIF> unc);
// expr - unc
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const LHSExpression> expr, boost::shared_ptr<UncertaintyIF> unc);

// expr + expr
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const LHSExpression> expr1, boost::shared_ptr<const LHSExpression> expr2);
// expr - expr
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const LHSExpression> expr1, boost::shared_ptr<const LHSExpression> expr2);

// expr + term
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const LHSExpression> expr, boost::shared_ptr<const ConstraintTermIF> term);
// expr - term
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const LHSExpression> expr, boost::shared_ptr<const ConstraintTermIF> term);



// TERM + ...
// term + var
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const ConstraintTermIF> term, boost::shared_ptr<DecisionVariableIF> var);
// term - var
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const ConstraintTermIF> term, boost::shared_ptr<DecisionVariableIF> var);

// term + real
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const ConstraintTermIF> term, double real);
// term - real
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const ConstraintTermIF> term, double real);

// term + unc
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const ConstraintTermIF> term, boost::shared_ptr<UncertaintyIF> unc);
// term - unc
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const ConstraintTermIF> term, boost::shared_ptr<UncertaintyIF> unc);

// term + expr
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const ConstraintTermIF> term, boost::shared_ptr<const LHSExpression> expr);
// term - expr
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const ConstraintTermIF> term, boost::shared_ptr<const LHSExpression> expr);

// term + term
boost::shared_ptr<LHSExpression> operator+(boost::shared_ptr<const ConstraintTermIF> term1, boost::shared_ptr<const ConstraintTermIF> term2);
// term - term
boost::shared_ptr<LHSExpression> operator-(boost::shared_ptr<const ConstraintTermIF> term1, boost::shared_ptr<const ConstraintTermIF> term2);


// real * var
boost::shared_ptr<LHSExpression> operator*(double real, boost::shared_ptr<DecisionVariableIF> var);

// real * unc
boost::shared_ptr<LHSExpression> operator*(double real, boost::shared_ptr<UncertaintyIF> unc);

// real * expr
boost::shared_ptr<LHSExpression> operator*(double real, boost::shared_ptr<const LHSExpression> expr);

// var1 * var2
boost::shared_ptr<LHSExpression> operator*(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<DecisionVariableIF> var2);

// var * unc
boost::shared_ptr<LHSExpression> operator*(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<UncertaintyIF> unc);

// var * expr
boost::shared_ptr<LHSExpression> operator*(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<LHSExpression> expr);

// unc * expr
boost::shared_ptr<LHSExpression> operator*(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<LHSExpression> expr);

// unc * var
boost::shared_ptr<LHSExpression> operator*(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<DecisionVariableIF> var);

// expr * unc
boost::shared_ptr<LHSExpression> operator*(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<UncertaintyIF> unc);

// expr * var
boost::shared_ptr<LHSExpression> operator*(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<DecisionVariableIF> var);

// expr * real
boost::shared_ptr<LHSExpression> operator*(boost::shared_ptr<const LHSExpression> expr, double real);

// real * term
boost::shared_ptr<LHSExpression> operator*(double real, boost::shared_ptr<const ConstraintTermIF> term);


// REAL + ...
// expr <= real
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<LHSExpression> expr, double real);
// expr >= real
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<LHSExpression> expr, double real);
// expr == real
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<LHSExpression> expr, double real);

// real <= expr
boost::shared_ptr<ConstraintIF> operator<=(double real, boost::shared_ptr<LHSExpression> expr);
// real >= expr
boost::shared_ptr<ConstraintIF> operator>=(double real, boost::shared_ptr<LHSExpression> expr);
// real == expr
boost::shared_ptr<ConstraintIF> operator==(double real, boost::shared_ptr<LHSExpression> expr);


// var <= real
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<DecisionVariableIF> var, double real);
// var >= real
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<DecisionVariableIF> var, double real);
// var == real
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<DecisionVariableIF> var, double real);

// real <= var
boost::shared_ptr<ConstraintIF> operator<=(double real, boost::shared_ptr<DecisionVariableIF> var);
// real >= var
boost::shared_ptr<ConstraintIF> operator>=(double real, boost::shared_ptr<DecisionVariableIF> var);
// real == var
boost::shared_ptr<ConstraintIF> operator==(double real, boost::shared_ptr<DecisionVariableIF> var);


// unc <= real
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<UncertaintyIF> unc, double real);
// unc >= real
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<UncertaintyIF> unc, double real);
// unc == real
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<UncertaintyIF> unc, double real);

// real <= unc
boost::shared_ptr<ConstraintIF> operator<=(double real, boost::shared_ptr<UncertaintyIF> unc);
// real >= unc
boost::shared_ptr<ConstraintIF> operator>=(double real, boost::shared_ptr<UncertaintyIF> unc);
// real == unc
boost::shared_ptr<ConstraintIF> operator==(double real, boost::shared_ptr<UncertaintyIF> unc);


// term <= real
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<ConstraintTermIF> term, double real);
// term >= real
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<ConstraintTermIF> term, double real);
// term == real
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<ConstraintTermIF> term, double real);

// real <= term
boost::shared_ptr<ConstraintIF> operator<=(double real, boost::shared_ptr<ConstraintTermIF> term);
// real >= term
boost::shared_ptr<ConstraintIF> operator>=(double real, boost::shared_ptr<ConstraintTermIF> term);
// real == term
boost::shared_ptr<ConstraintIF> operator==(double real, boost::shared_ptr<ConstraintTermIF> term);



// EXPR + ...
// expr <= var
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<DecisionVariableIF> var);
// expr >= car
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<DecisionVariableIF> var);
// expr == var
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<DecisionVariableIF> var);

// var <= expr
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<LHSExpression> expr);
// var >= expr
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<LHSExpression> expr);
// var == expr
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<LHSExpression> expr);


// expr <= unc
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<UncertaintyIF> unc);
// expr >= unc
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<UncertaintyIF> unc);
// expr == unc
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<UncertaintyIF> unc);

// unc <= expr
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<LHSExpression> expr);
// unc >= expr
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<LHSExpression> expr);
// unc == expr
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<LHSExpression> expr);


// expr <= expr
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<LHSExpression> expr1, boost::shared_ptr<LHSExpression> expr2);
// expr >= expr
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<LHSExpression> expr1, boost::shared_ptr<LHSExpression> expr2);
// expr == expr
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<LHSExpression> expr1, boost::shared_ptr<LHSExpression> expr2);


// term <= expr
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<LHSExpression> expr);
// term >= expr
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<LHSExpression> expr);
// term == expr
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<LHSExpression> expr);

// expr <= term
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<ConstraintTermIF> term);
// expr >= term
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<ConstraintTermIF> term);
// expr == term
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<LHSExpression> expr, boost::shared_ptr<ConstraintTermIF> term);



// VAR + ...
// var <= var
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<DecisionVariableIF> var1, boost::shared_ptr<DecisionVariableIF> var2);
// var >= var
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<DecisionVariableIF> var1, boost::shared_ptr<DecisionVariableIF> var2);
// var == var
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<DecisionVariableIF> var1, boost::shared_ptr<DecisionVariableIF> var2);


// var <= unc
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<UncertaintyIF> unc);
// var >= unc
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<UncertaintyIF> unc);
// var == unc
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<UncertaintyIF> unc);

// unc <= var
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<DecisionVariableIF> var);
// unc >= var
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<DecisionVariableIF> var);
// unc == var
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<DecisionVariableIF> var);


// term <= var
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<DecisionVariableIF> var);
// term >= var
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<DecisionVariableIF> var);
// term == var
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<DecisionVariableIF> var);

// var <= term
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<ConstraintTermIF> term);
// var >= term
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<ConstraintTermIF> term);
// var == term
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<DecisionVariableIF> var, boost::shared_ptr<ConstraintTermIF> term);



// UNC + ...

// unc <= unc
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<UncertaintyIF> unc1, boost::shared_ptr<UncertaintyIF> unc2);
// unc >= unc
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<UncertaintyIF> unc1, boost::shared_ptr<UncertaintyIF> unc2);
// unc == unc
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<UncertaintyIF> unc1, boost::shared_ptr<UncertaintyIF> unc2);


// term <= unc
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<UncertaintyIF> unc);
// term >= unc
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<UncertaintyIF> unc);
// term == unc
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<ConstraintTermIF> term, boost::shared_ptr<UncertaintyIF> unc);

// unc <= term
boost::shared_ptr<ConstraintIF> operator<=(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<ConstraintTermIF> term);
// unc >= term
boost::shared_ptr<ConstraintIF> operator>=(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<ConstraintTermIF> term);
// unc == term
boost::shared_ptr<ConstraintIF> operator==(boost::shared_ptr<UncertaintyIF> unc, boost::shared_ptr<ConstraintTermIF> term);




#endif /* Interface_hpp */
