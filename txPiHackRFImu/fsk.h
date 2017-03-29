#ifndef FSK_H
#define FSK_H

#include "datatypes.h"

class FSK
{
public:
    FSK(TYPEREAL bandwidth , TYPEREAL sampleRate) ;

    void addChar( char c );
    void raz() { m_length=0 ; }
    void generate();
    int getSize() { return( m_length* 10 * sines_per_bit); }
    unsigned char *data() { return( samples ) ; }

private:
    TYPEREAL m_bandWidth ;
    TYPEREAL m_sampleRate;
    int sines_per_bit ;
    int m_length ;
    int msg_size ;
    bool m_up ;

    unsigned char *samples ;

    unsigned char *mark ;
    unsigned char *space ;

    char *txmessage ;

    void generateMS();
};

#endif // FSK_H
