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
#ifndef UAVPROCESSOR_H
#define UAVPROCESSOR_H

#include <QObject>
#include <fftw3.h>
#include "common/samplefifo.h"
#include "common/datatypes.h"

class UAVProcessor : public QObject
{
    Q_OBJECT
public:

    explicit UAVProcessor(QObject *parent = 0);

    // change detection threshold for the correlator
    // default is 30dB
    void setDetectionThreshold(float level);

    // call this function each time a paquet of samples is available
    void newData(TYPECPX* IQsamples, int L , int sampleRate );

    void raz();

signals:
    void detectionLevel( float level ) ;
    void frameDetected( float signal_level, float noise_level, QString message ) ;

public slots:

private:
     enum  { sInit=0 , sSearch=1,sChirpFound=2,sDecodeFSK,sMeasureNoise } ;
     int m_state, next_state ;
     float signal_plus_noise, noise_only ;
     QString lastFrameReceived ;

    TYPECPX *samples ;
    int m_bandwidth ;
    int chirp_length ;
    int fft_length ;
    int wr_pos ;
    int msg_wrpos ;
    int msg_length ;
    float ratio_min ;
    float ratio_max ;
    float threshold ;
    bool synched ;
    float detection_threshold ;
    fftwf_complex * fftin, *fftout,*fftchirp;
    fftwf_plan fft_plan,fft_plan_inv ;

    void generate(TYPEREAL bandwidth , TYPEREAL sampleRate, long length);
    int calc(float *pvmax);
    char *demod(TYPECPX* psamples , int Lmax, int *consumed );
    int demodByte(TYPECPX* psamples, int *pstart, int L );

    QString stateToS(int s);
};

#endif // UAVPROCESSOR_H
