#ifndef DATATYPES_H
#define DATATYPES_H

#include <math.h>
typedef struct _sCplx
{
    float re;
    float im;
}tSComplex;

typedef struct _dCplx
{
    double re;
    double im;
}tDComplex;

#define TYPECPX	tSComplex
#define TYPEREAL float
#define MSIN(x) sin(x)
#define MCOS(x) cos(x)

#define K_2PI (2.0 * 3.14159265358979323846)
#define K_PI (3.14159265358979323846)

inline TYPECPX conj( TYPECPX a ) {
    TYPECPX w ;
    w.re = a.re ;
    w.im = -a.im ;
    return( w );
}

#endif // DATATYPES_H
