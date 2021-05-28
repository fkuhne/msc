/* func.h */

#ifndef FUNC_H
#define FUNC_H

#include "normal.h"
#include "mmath.h"
#include <math.h>

extern CVECTOR<double> f(const CVECTOR<double>, const CVECTOR<double>, const double);
extern MATRIX<double> jacob_F(const CVECTOR<double>, const CVECTOR<double>, const double);
extern CVECTOR<double> noise(const double, const double, const double);
extern CVECTOR<double> rcv_q (std::string);
extern CVECTOR<double> rcv_u (std::string);
extern MATRIX<double> rcv_E (std::string);
extern MATRIX<double> rcv_E (std::string);
extern CVECTOR<double> rcv_e (std::string);
 
#endif
