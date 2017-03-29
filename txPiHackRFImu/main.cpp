#include "mainwindow.h"
#include <QApplication>
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


    Chirp *c = new Chirp(100e3,HACKRF_SR,duree_chirp,3*duree_chirp,true);
    c->generate();

    Transmitter *tx = new Transmitter(HACKRF_SR);
    if( !tx->boardOK() ) {
        std::cout << "board ?" ;
        return(-1);
    }

    if( tx->setTxCenterFreq( 439.900e6 ) == 0 ) {
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
                    float dlat = lat - gps_data.fix.latitude ;
                    float dlon = lon - gps_data.fix.longitude ;
                    float dalt = alt - gps_data.fix.altitude ;

                    if( (fabs(dlat)>1e-4) || (fabs(dlon)>1e-4) || (fabs(dalt)>1)) {
                        lat = gps_data.fix.latitude ;
                        lon = gps_data.fix.longitude ;
                        alt = gps_data.fix.altitude ;                                                
                    }
                }
                tx->setTelemetryData( lat, lon, (int)(alt*10), ahrs.getRoll(),ahrs.getPitch(),ahrs.getYaw());
                printf("Set position: %0.8f,%0.8f,%0.1f,%d,%d,%d\n", lat,lon,alt, ahrs.getRoll(),ahrs.getPitch(),ahrs.getYaw());
            }
            usleep(1000 * 250);
        }
    }
    tx->stopTransmission();
    return(1);
}
