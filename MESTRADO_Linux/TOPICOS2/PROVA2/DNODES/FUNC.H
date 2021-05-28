/* func.h */

#ifndef FUNC_H
#define FUNC_H

#include "mmath.h"
#include <stdio.h>
#include <string.h>

extern CVECTOR<double> f(const CVECTOR<double>,const CVECTOR<double>,double);
extern MATRIX<double> jacob_F(const CVECTOR<double>, const CVECTOR<double>, double);

#endif
