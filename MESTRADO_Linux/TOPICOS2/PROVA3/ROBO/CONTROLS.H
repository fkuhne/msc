/* controls.h */

#ifndef CONTROLS_H
#define CONTROLS_H

extern MATRIX<double> R_OC (const CVECTOR<double>);
extern CVECTOR<double> ref_point(const CVECTOR<double>, const CVECTOR<double>);
extern CVECTOR<double> go_to_ref_point(const CVECTOR<double>, const CVECTOR<double>);
extern CVECTOR<double> go_to_center_point(const CVECTOR<double>, const CVECTOR<double>);
extern CVECTOR<double> linearization(const CVECTOR<double>, const CVECTOR<double>, const CVECTOR <double>);
extern CVECTOR<double> control(const CVECTOR<double>, const CVECTOR<double>, const MATRIX<double>);

#endif
