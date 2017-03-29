#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "chirp.h"


Chirp::Chirp(TYPEREAL bandwidth, TYPEREAL sampleRate , long length, long blanking, bool up)
{
    m_bandWidth = bandwidth ;
    m_sampleRate = sampleRate;
    m_length = length ;
    m_up = up ;
    samples = NULL ;
    m_blanking = blanking ;
}

void Chirp::setLength( long L ) {
    m_length = L ;
}

int Chirp::getSize() {
    return( m_length+m_blanking);
}

void Chirp::generate() {
    if( samples != NULL )
        free(samples);


    double Tech = 1.0/m_sampleRate ;
    double k = m_bandWidth / (m_length * Tech);
    double fstart = -m_bandWidth/2 ;
    samples = (TYPECPX *)malloc( (m_length+m_blanking) * sizeof( TYPECPX ));

    double t = 0 ;
    int n = 0 ;
    for( ; n < m_length ; n++ ) {
         double omega = K_2PI*( m_sampleRate/4.0 + fstart + k/2*t ) ;
         samples[n].re =  (float)(.8 * MCOS( omega * t)) ;
         samples[n].im  = (float)(.8 * MSIN( omega * t)) ;
         t += Tech ;
    }

    std::cout << "n=" << n << "\n";

    for( ; n < (m_length+m_blanking) ; n++ ) {
        samples[n].re = 0 ;
        samples[n].im = 0 ;
    }

    std::cout << "n=" << n << "\n" ;

    if( 0 ) {
            FILE *f = fopen( "Chirp.dat", "wb" );
            fwrite( samples, sizeof( TYPECPX), n, f );
            fclose( f );
    }
}
