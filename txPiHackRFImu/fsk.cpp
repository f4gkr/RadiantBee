#include "fsk.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define FSK_SPEED (800)

FSK::FSK(TYPEREAL bandwidth , TYPEREAL sampleRate)
{
    m_bandWidth = bandwidth ;
    m_sampleRate = sampleRate;
    m_length = 0 ;
    samples = NULL ;
    msg_size = 0 ;
    txmessage = (char *)malloc( 128*sizeof(char));
    sines_per_bit = (int)(sampleRate / FSK_SPEED) ;

    generateMS();
}

void FSK::addChar( char c ) {

    if( m_length+1 >= 128 )
        return ;
    txmessage[m_length++] = c ;
}

void FSK::generate() {
    int length = getSize();
    if( samples != NULL )
        free(samples);

    samples = (unsigned char *)malloc( 2 * length * sizeof(unsigned char));
    if( samples == NULL ) {
        printf("ouf of memory #1\n");
        return;
    }
    memset( samples,0,2 * length * sizeof(unsigned char));
    int n = 0 ;

    for( int i=0 ; i < m_length ; i++ ) {
         int mask = 0x200 ;
         unsigned int symbol = (unsigned int )txmessage[i] << 1;
         symbol = symbol | 0x201 ;
         for( int bit=9 ; bit >= 0 ; bit-- ) {
              if( (symbol & mask) == mask) {
                  memcpy( &samples[n], mark, 2 * sines_per_bit * sizeof(unsigned char));
              } else {
                  memcpy( &samples[n], space, 2 * sines_per_bit * sizeof(unsigned char));
              }
              n += 2 * sines_per_bit * sizeof(unsigned char) ;
              mask /= 2 ;
         }
    }
}

void FSK::generateMS() {
    double Tech = 1.0/m_sampleRate ;
    double t = 0 ;
    float I,Q;

    double omegamark = K_2PI * (m_sampleRate/4+m_bandWidth/2);
    double omegaspace = K_2PI * (m_sampleRate/4-m_bandWidth/2);

    mark = (unsigned char *)malloc( 2 * sines_per_bit * sizeof(unsigned char));
    space = (unsigned char *)malloc( 2 * sines_per_bit * sizeof(unsigned char));

    for( int pulse=0 ; pulse < sines_per_bit ; pulse++ ) {
        I =  (float)(.8 * MCOS( omegamark * t)) ;
        Q  = (float)(.8 * MSIN( omegamark * t)) ;
        int rI = (int)( I * 127.0) ;
        int rQ = (int)( Q * 127.0) ;
        int p = 2*pulse ;
        mark[ p ] = (unsigned char)( rI & 0xFF) ;
        mark[ p+1 ] = (unsigned char)( rQ & 0xFF) ;

        I =  (float)(.8 * MCOS( omegaspace * t)) ;
        Q  = (float)(.8 * MSIN( omegaspace * t)) ;
        rI = (int)( I * 127.0) ;
        rQ = (int)( Q * 127.0) ;
        space[ p ] = (unsigned char)( rI & 0xFF) ;
        space[ p+1 ] = (unsigned char)( rQ & 0xFF) ;

        t += Tech ;
    }
}
