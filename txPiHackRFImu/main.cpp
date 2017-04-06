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
#include "mainwindow.h"
#include <QApplication>
#include <QDebug>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <strings.h>
#include <gps.h>

#include "transmitter.h"
#include "chirp.h"
#include "fsk.h"
#include "ahrs/ahrs.h"
#include "ahrs/lps331.h"

#define HACKRF_SR (4000000)

int main(int argc, char *argv[])
{

    double lon = 0 ;
    double lat = 0 ;
    float alt = 0 ;
    long frameno = 0 ;
    long duree_chirp = HACKRF_SR/4 ;
    bool gps_ok = false ;
    struct gps_data_t gps_data;
    int rc ;

    AHRS ahrs ;
    LPS331 baro((const char *)"/dev/i2c-1") ;
    if( baro.isReady() ) {
        qDebug() << "Temperature:" << baro.readTemperatureC();
        qDebug() << " Pression" << baro.readPressureMillibars();
    }


    Chirp *c = new Chirp(100e3,HACKRF_SR,duree_chirp,3*duree_chirp,true);
    c->generate();

    Transmitter *tx = new Transmitter(HACKRF_SR);
    if( !tx->boardOK() ) {
        std::cout << "board ?" ;
        return(-1);
    }

    if( tx->setTxCenterFreq( 868.80e6 ) == 0 ) { //439.9
        std::cout << "frequency set error\n" ;
        return(-2);
    }


    if((rc=gps_open("localhost", "2947", &gps_data))<0){
        printf("GPS error\n");
        printf("code: %d, reason: %s\n", rc, gps_errstr(rc));
        gps_ok = false ;
    } else {
        printf("GPS OK\n");
        gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON , NULL);
        gps_ok = true ;
    }

    ahrs.start();

    tx->setTxLength( 4*duree_chirp );
    tx->setData( c->data(),0,c->getSize() );
    tx->startTransmission();

    tx->setTelemetryData( lat, lon, (int)(alt*10), ahrs.getRoll(),ahrs.getPitch(),ahrs.getYaw());
    while( tx->isTransmitting() ) {
        fflush(stdout);
        long fno = tx->getFrameNo() ;
        if( fno != frameno ) {
            printf("Transmitting frame %ld\n", fno );
            frameno = fno ;
        } else {
            if( gps_ok ) {
                if (!gps_waiting(&gps_data, 1200000)) {
                    printf("GPS error\n");
                    continue ;
                }
                if ((rc=gps_read(&gps_data)) == -1) {
                    printf("GPS error\n");
                    gps_ok = false ;
                    printf("error occured reading gps data. code: %d, reason: %s\n", rc, gps_errstr(rc));
                    continue ;
                }
                if ((gps_data.fix.mode == MODE_2D) || (gps_data.fix.mode == MODE_3D) ) {

                        lat = gps_data.fix.latitude ;
                        lon = gps_data.fix.longitude ;

                }
                float temperature = baro.readTemperatureC();
                float pressure = baro.readPressureMillibars() ;
                alt = baro.pressureToAltitudeMeters(pressure) ;
                qDebug() << "temperatuer:" << temperature << "pressure:" << pressure << " alt=" << alt ;
                tx->setTelemetryData( lat, lon, (int)(alt*10), ahrs.getRoll(),ahrs.getPitch(),ahrs.getYaw());
                printf("Set position: %0.8f,%0.8f,%0.1f,%d,%d,%d\n", lat,lon,alt, ahrs.getRoll(),ahrs.getPitch(),ahrs.getYaw());
            }
            usleep(1000 * 250);
        }
    }
    tx->stopTransmission();
    return(1);
}
