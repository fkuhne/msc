/* func.h */

#ifndef FUNC_H
#define FUNC_H

#include "mmath.h"

extern CVECTOR<double> f(const CVECTOR<double>, const CVECTOR<double>, const double);
extern MATRIX<double> jacob_F(const CVECTOR<double>, const CVECTOR<double>, const double);
extern CVECTOR<double> noise(const double, const double, const double);
 
#endif
