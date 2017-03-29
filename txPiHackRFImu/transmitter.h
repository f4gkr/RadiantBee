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
#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include "datatypes.h"
#include "hackrf.h"
#include <math.h>
#include <liquid/liquid.h>
#include <pthread.h>
#include <semaphore.h>

#include "fsk.h"

class Transmitter
{
public:
    enum { stInit=0, stTxWaveForm=1 ,stTxSilence=2, stTxData=3, stTxStop=4, stTxWait=5 };
    Transmitter(hackrf_device *board_used);
    Transmitter(double samplerate=8e6);
    ~Transmitter();

    bool boardOK();

    int setTxCenterFreq( double freq );
    bool isTransmitting();

    void setTxLength( int L );
    void setSilenceLength( int L );
    void setData( TYPECPX *src, int index_write, int length );
    void set8bitData( unsigned char *src, int index_write, int length );

    void writeForDebug();
    void startTransmission();
    void stopTransmission();

    void setABC( float a, float b, float c );
    void setWait( bool no_tx );

    void setTelemetryData( double tlat, double tlon, int h, int r,int p, int y );

    long getFrameNo() { return(frame_no); }

private:
    double sample_rate ;
    int m_length ;
    int sil_lenght ;
    int read_pos ;
    int silence_pos ;
    int mState, pState ;
    unsigned char *tx_buff ;
    float A, B,C ;

    hackrf_device *board ;
    hackrf_device_list_t *devices ;
    long frame_no ;
    double lon,lat ;
    int alt ;
    int roll,pitch,yaw;
    FSK *modem ;
    sem_t mutex;
    pthread_t fsk_thread ;

    static int _hackrf_tx_callback( hackrf_transfer* transfer);
    int tx_callback(unsigned char *buf, uint32_t len);
};

#endif // TRANSMITTER_H
