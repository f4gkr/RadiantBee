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
