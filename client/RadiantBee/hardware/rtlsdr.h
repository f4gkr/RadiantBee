#ifndef RTLSDR_H
#define RTLSDR_H

//==========================================================================================
// + + +   This Software is released under the "Simplified BSD License"  + + +
//   Copyright 2010 F4GKR Sylvain AZARIAN . All rights reserved.
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
#include <rtl-sdr.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdint.h>
#include "common/datatypes.h"
#include "common/samplefifo.h"

class RTLSDR
{
public:
    explicit RTLSDR( int select_index = 0 );
    int getBoardCount() { return( device_count ) ; }

    int startAcquisition();
    int stopAcquisition();
    void close();

    int setRxSampleRate(uint32_t sampling_rate = 2200000 ) ;
    uint32_t getRxSampleRate() {
        if( sampling_rate == 0 ) {
             setRxSampleRate() ;
        }
        return( sampling_rate );
    }

    int getFifoSize() {
        return( fifo->getSize()) ;
    }

    SampleFifo *getFIFO() { return( fifo ); }

    int setRTLGain(float db ) ;
    float getRxGain();
    bool setAutoGainMode();
    int setRxCenterFreq( uint64_t freq_hz );
    uint64_t getRxCenterFreq();

    uint64_t getMin_HWRx_CenterFreq() ;
    uint64_t getMax_HWRx_CenterFreq() ;

    static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx) ;
    rtlsdr_dev_t *rtlsdr_device ;
    sem_t mutex;

private:
    bool m_stop ;
    char *hardwareName ;
    int dev_index  ;
    int device_count ;

    SampleFifo *fifo ;
    TYPECPX xn_1, yn_1 ;

    uint64_t freq_hz ; // current center freq
    uint32_t sampling_rate ; // current sampling rate

    float last_gain ;
    int *gain_value ;
    int gain_size ;
    float gainMin, gainMax ;
    pthread_t receive_thread ;
    uint64_t min_tuner_freq ;
    uint64_t max_tuner_freq ;

     int processData( unsigned char *buf, uint32_t len ) ;
};

#endif // RTLSDR_H
