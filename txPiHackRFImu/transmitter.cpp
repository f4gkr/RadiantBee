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
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "transmitter.h"

#define GUARD_INTERVAL (1600)

Transmitter::Transmitter(hackrf_device *board_used)
{
    this->board = board_used ;
    m_length = 0 ;
    tx_buff = NULL ;
    read_pos = 0 ;
    mState = stInit;
    sil_lenght = 0 ;
    A = 1 ;
    B = 1 ;
    C = 0 ;
    sample_rate = 0 ;
    modem = NULL ;
    lon = 0 ;
    lat = 0 ;
    alt = 0 ;
    roll = pitch =yaw = 0 ;
    sem_init(&mutex, 0, 0);
}

Transmitter::Transmitter(double samplerate)
{
    board = NULL ;
    m_length = 0 ;
    tx_buff = NULL ;
    read_pos = 0 ;
    mState = stInit;
    sil_lenght = 0 ;
    A = 1 ;
    B = 1 ;
    C = 0 ;
    modem = NULL ;
    lon = 0 ;
    lat = 0 ;
    alt = 0 ;
    roll = pitch =yaw = 0 ;

    int rc = hackrf_init() ;
    if( rc != HACKRF_SUCCESS ) {
        std::cout << "Error init hack rf";
        return;
    }

    devices = hackrf_device_list();
    if( devices == NULL ) {
        std::cout << "Error : no hack rf";
        return ;
    }

    if( devices->devicecount == 0 ) {
        std::cout << "Error : no hack rf (count=0)";
        hackrf_device_list_free( devices );
        return ;
    }
    std::cout << "opening device\n" ;
    rc = hackrf_open( &board );
    if( rc != HACKRF_SUCCESS ) {
        std::cout << "Error :cannot open #0";
        hackrf_device_list_free( devices );
        return ;
    }

    std::cout << "setting sample rate\n" ;
    rc = hackrf_set_sample_rate( board, samplerate );
    if( rc != HACKRF_SUCCESS ) {
        std::cout << "Error :cannot set SR on #0\n";
        hackrf_close( board );
        hackrf_device_list_free( devices );
        return ;
    }
    sample_rate = samplerate ;
    rc = hackrf_set_txvga_gain( board, 30 );
    if( rc != HACKRF_SUCCESS ) {
        std::cout << "Error :cannot set tx gain on #0\n";
        hackrf_close( board );
        hackrf_device_list_free( devices );
        return ;
    }
    sem_init(&mutex, 0, 0);
}

Transmitter::~Transmitter() {
    if( board != NULL ) {
        hackrf_close( board );
        hackrf_device_list_free( devices );
    }
}

bool Transmitter::boardOK() {
    return( board != NULL );
}

bool Transmitter::isTransmitting() {
    return( hackrf_is_streaming( board ) == HACKRF_TRUE ) ;
}

int Transmitter::setTxCenterFreq(double freq) {
    long freq_hz = (long)(freq - sample_rate/4) ;
    long carrier = (long)(freq_hz + sample_rate/4) ;

    std::cout << "setting tx freq to " << freq_hz << ".\n";
    std::cout << "signal is tx on " << carrier << ".\n" ;
    int rc = hackrf_set_freq( board, freq_hz );
    if( rc != HACKRF_SUCCESS ) {
        std::cout << "Error :cannot set FREQ on #0\n";
        hackrf_close( board );
        hackrf_device_list_free( devices );
        return(0);
    }
    return(1);
}

void Transmitter::setTxLength( int L ) {
    m_length = 2*L ;
    if( tx_buff != NULL )
        free( tx_buff );
    tx_buff = (unsigned char *)malloc( m_length * sizeof( unsigned char ));
    for( int i=0 ; i < m_length ; i++ ) {
        tx_buff[i ]  = (unsigned char)0 ;
    }
    read_pos = 0 ;
    silence_pos = 0 ;

    std::cout << "setting tx length to " << L << " samples .\n";
}

void Transmitter::setSilenceLength(int L) {
    this->sil_lenght = 2*L ;
    this->silence_pos = m_length ;
}


void Transmitter::setABC( float a, float b, float c ) {
    A = a ;
    B = b ;
    C = c ;
}

void Transmitter::setData( TYPECPX *src, int index_write, int length ) {

    //printf("Adding float IQ from %d to %d\n", index_write, index_write+length);

    float x;
    for( int i=0 ; i < length ; i++ ) {
        TYPEREAL I = src[i].re ;
        TYPEREAL Q = src[i].im ;

        x = I ;
        I = I * A ;
        Q = x * C + Q * B;

        int rI = (int)( I * 127.0) ;
        int rQ = (int)( Q * 127.0) ;

        int p = 2*(index_write + i);
        tx_buff[ p ] = (unsigned char)( rI & 0xFF) ;
        tx_buff[ p+1 ] = (unsigned char)( rQ & 0xFF) ;
    }
}

void Transmitter::set8bitData( unsigned char *src, int index_write, int length ) {
    //printf("Adding 8bits IQ from %d to %d\n", index_write, index_write+length);
    for( int i=0 ; i < length ; i++ ) {
        int p = 2*(index_write + i);
        tx_buff[ p ] = src[2*i];
        tx_buff[ p+1 ] = src[2*i+1];
    }
}


void Transmitter::setWait( bool no_tx ) {
    if( no_tx ) {
        pState = mState ;
        mState = stTxWait ;
        return ;
    }
    mState = pState ;
}

int Transmitter::tx_callback(unsigned char *buf, uint32_t len) {
    unsigned int i = 0, j;
    int next_state = mState ;
    char txmsg[255];
    unsigned char *fskdata ;

    switch( mState ) {
    case stInit:
        read_pos = 0 ;
        memset( (void *)buf, 0, len );
        next_state = stTxWaveForm ;
        break;

    case stTxWaveForm:
        for( ; i < len ; i++ ) {
            *buf++   = tx_buff[read_pos++] ;
            if( read_pos == m_length ) {
                read_pos = 0 ;
                frame_no++ ;
                if( sil_lenght > 0 ) {
                    silence_pos = sil_lenght ;                    
                }
                next_state = stTxWait ;
                break ;
            }
        }

        break ;


    case stTxWait:
        modem->raz();
        sprintf(txmsg, "[%ld;%02.7f;%02.7f;%d;%d;%d;%d]", frame_no, lon, lat, alt,roll,pitch,yaw);
        printf("Ttx msg : %s\n", txmsg );
        for( j=0 ; j < strlen(txmsg); j++ ) {
            modem->addChar( txmsg[j]);
        }
        modem->generate();
        fskdata = modem->data() ;
        set8bitData( fskdata, (int)(sample_rate/4)+GUARD_INTERVAL, modem->getSize() );
        next_state = stTxWaveForm ;
        break ;

    case stTxSilence:
        for( ; (i < len) && (silence_pos-->=0); i++ ) {
            *buf++   = 0;
        }
        if( silence_pos < 0 ) {
            next_state = stTxWaveForm ;
            read_pos = 0 ;
        }

        break ;
    case stTxStop:
        return(1);
    }
    if( next_state != mState ) {
        mState = next_state ;
    }

    return(0);
}

void Transmitter::setTelemetryData(double tlat, double tlon, int h , int r, int p, int y) {
    lon = tlon ;
    lat = tlat ;
    alt = h ;
    roll = r ;
    pitch = p;
    yaw = y ;
}

void Transmitter::stopTransmission() {
    mState = stTxStop ;
}

int Transmitter::_hackrf_tx_callback( hackrf_transfer* transfer) {
    Transmitter *obj = (Transmitter *)transfer->tx_ctx;
    if( transfer->valid_length == 0) return(0);
    //printf("_hackrf_tx_callback %d\n", (int)transfer->valid_length ) ;
    return obj->tx_callback(transfer->buffer, transfer->valid_length);
}

void Transmitter::writeForDebug() {
    printf("%s:Saving %d samples for debug\n", __func__, m_length );
    FILE *f = fopen( "txbuff.dat", "wb" );
    fwrite( tx_buff, sizeof(unsigned char), m_length, f );
    char empty[1];
    empty[0] = (char)0 ;
    for( int i=0 ; i < sil_lenght ;i++) {
        fwrite( empty, sizeof(unsigned char), 1, f );
    }
    fclose( f );
}

void Transmitter::startTransmission() {
    char txmsg[255];
    frame_no = 1 ;
    modem = new FSK(50e3,sample_rate);
    sprintf(txmsg, "[%ld;%02.7f;%02.7f;%d;%d;%d;%d]", frame_no, lon, lat, alt,roll,pitch,yaw);
    for( size_t i=0 ; i < strlen(txmsg); i++ ) {
        modem->addChar( txmsg[i]);
    }
    modem->generate();
    unsigned char *fskdata = modem->data() ;
    set8bitData( fskdata, (int)(sample_rate/4)+GUARD_INTERVAL, modem->getSize() );

    fflush(stdout);
    printf("%s : preparing tx\n", __func__ );
    hackrf_set_lna_gain( board,  3 );
    hackrf_set_vga_gain( board, 3 );
    //hackrf_set_amp_enable( board, 0);
    printf("%s : starting tx\n", __func__ );
    fflush(stdout);
    read_pos = 0 ;
    int rc = hackrf_start_tx( board, _hackrf_tx_callback, (void *)this );
    if( rc != HACKRF_SUCCESS ) {
        printf("%s: error starting tx %s\n", __func__, hackrf_error_name( (enum hackrf_error)rc));
        return ;
    }
    if(hackrf_is_streaming( board ) == HACKRF_TRUE ) {
        printf("%s : streaming tx\n", __func__ );
        return;
    }
}
