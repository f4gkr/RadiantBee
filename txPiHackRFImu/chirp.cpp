//==========================================================================================
// + + +   This Software is released under the "Simplified BSD License"  + + +
// Copyright 2014-2017 F4GKR Sylvain AZARIAN . All rights reserved.
//
//Redistribution and use in source and binary forms, with or without modification, are
//permitted provided that the following conditions are met:
//
//   1. Redistributions of source code must retain the above copyright notice, this list of
//	  conditions and the following disclaimer.
//
//   2. Redistributions in binary form must reproduce the above copyright notice, this list
//	  of conditions and the following disclaimer in the documentation and/or other materials
//	  provided with the distribution.
//
//THIS SOFTWARE IS PROVIDED BY Sylvain AZARIAN F4GKR ``AS IS'' AND ANY EXPRESS OR IMPLIED
//WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
//FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Sylvain AZARIAN OR
//CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
//ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
//NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
//ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//The views and conclusions contained in the software and documentation are those of the
//authors and should not be interpreted as representing official policies, either expressed
//or implied, of Sylvain AZARIAN F4GKR.
//==========================================================================================
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
