#ifndef CHIRP_H
#define CHIRP_H

#include "datatypes.h"

class Chirp
{
public:
    Chirp(TYPEREAL bandwidth , TYPEREAL sampleRate, long length, long blanking, bool up=true) ;
    void setLength( long L );
    void generate();
    int getSize() ;
    TYPECPX *data() { return( samples ) ; }

private:
    TYPEREAL m_bandWidth ;
    TYPEREAL m_sampleRate;
    long m_length ;
    long m_blanking ;
    bool m_up ;
    TYPECPX *samples ;

};

#endif // CHIRP_H
