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
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QObject>
#include <QThread>
#include <QSemaphore>
#include <stdint.h>
#include <fftw3.h>

#include "hardware/rtlsdr.h"
#include "dsp/overlapsave.h"
#include "dsp/uavprocessor.h"
/**
  In current version the frame is 100 KHz wide
  we sample 110 Khz. To avoid RX Dc at center, RX windows is 10 K above rx center
  example : RTLSDR rx center at 430.000
  window starts at 430.010
                center at 430.065
                   ends at 430.120
  */
#define FRAME_CENTER (10e3 + 55e3)
#define FRAME_OFFSET_LOW 10e3
#define FRAME_OFFSET_HIGH 120e3

class Controller : public QThread
{
    Q_OBJECT
public:

    static Controller& getInstance()  {
        static Controller instance;
        return instance;
    }

    void setRadio( RTLSDR* radio ) ;
    void setRxCenterFrequency( uint64_t frequency ) ;

    void startAcquisition();
    void close();
    void getSpectrum( double* values );

signals:
    void newSpectrumAvailable(int len);
    void detectionLevel( float level ) ;
    void frameDetected( float signal_level, float noise_level, QString message ) ;
                          //int frameid, float uavlongitude, float uavlatitude, float uavaltitude,
                         //) ;

public slots:
    void SLOT_detectionLevel( float level ) ;
    void SLOT_frameDetected( float signal_level, float noise_level, QString message ) ;
    void  SLOT_hasGpsFix( double latitude, double longitude , double altitude );
    void  SLOT_hasGpsTime( int year, int month, int day,
                     int hour, int min, int sec, int msec );

private:
    enum  { csInit=0, csIdle=1, csStart=2,csRun=3, csStop=4, csEnded=99 } ;

    RTLSDR *radio ;
    OverlapSave *channelizer ;
    UAVProcessor *processor ;

    uint64_t rx_center_frequency ;
    bool m_stop ;
    int m_state, next_state ;

    double *spectrum ;
    double *hamming_coeffs ;
    fftwf_complex * fftin ;
     fftwf_plan plan ;
     QSemaphore *semspectrum ;

     double m_Latitude ;
     double m_Longitude ;
     double m_Altitude ;
     QSemaphore *sempos ;

    Controller();
    Controller(const Controller &); // hide copy constructor
    Controller& operator=(const Controller &);

    void run();
    void process( TYPECPX*samples, int L );
    void generateSpectrum( TYPECPX *samples );
    void hamming_window(double *win,  int win_size) ;
    float distance_between (float lat1, float long1, float lat2, float long2);
    float course_to (float lat1, float long1, float lat2, float long2);
};

#endif // CONTROLLER_H
