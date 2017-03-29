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
#include "controller.h"
#include "common/samplefifo.h"
#include "hardware/gpdsd.h"

#define STEP_SIZE 16384
#define FFT_SPECTRUM 2048

Controller::Controller() : QThread(NULL)
{
    radio = NULL ;
    rx_center_frequency = 0 ;
    m_stop = false ;
    m_state = Controller::csInit ;
    channelizer = NULL ;
    processor = new UAVProcessor();
    fftin = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * FFT_SPECTRUM );
    plan = fftwf_plan_dft_1d(FFT_SPECTRUM, fftin, fftin, FFTW_FORWARD, FFTW_ESTIMATE );
    spectrum = (double *)malloc( FFT_SPECTRUM * sizeof(double));
    hamming_coeffs = (double *)malloc( FFT_SPECTRUM * sizeof( double ));
    memset( spectrum, 0, FFT_SPECTRUM );
    semspectrum = new QSemaphore(1);
    hamming_window( hamming_coeffs, FFT_SPECTRUM ) ;

    connect( processor, SIGNAL(detectionLevel(float)), this, SLOT(SLOT_detectionLevel(float)));
    connect( processor, SIGNAL(frameDetected(float,float,QString)), this,SLOT(SLOT_frameDetected(float,float,QString)));

    sempos = new QSemaphore(1);
    m_Latitude = m_Longitude = m_Altitude = 0 ;
    GPSD& gpsd= GPSD::getInstance() ;
    connect( &gpsd, SIGNAL(hasGpsFix(double,double,double)), this, SLOT(SLOT_hasGpsFix(double,double,double)), Qt::QueuedConnection );
    connect( &gpsd, SIGNAL(hasGpsTime(int,int,int,int,int,int,int)), this, SLOT(SLOT_hasGpsTime(int,int,int,int,int,int,int)),Qt::QueuedConnection);

}

void Controller::hamming_window(double *win,  int win_size)
{
    int    i;
    if( win == NULL ) return ;
    for (i = 0; i < win_size; i++)
    {
        win[i] = (0.3635819 - 0.4891775 * cos((K_2PI * i) / (win_size - 1))
                  + 0.1365995 * cos((2.0 * K_2PI * i) / (win_size - 1))
                  - 0.0106411 * cos((3.0 * K_2PI * i) / (win_size - 1)));
    }
}

void Controller::setRadio(RTLSDR *radio) {
    this->radio = radio ;
    channelizer = new OverlapSave( radio->getRxSampleRate(),
                                   110e3 );
    channelizer->configure( 128*1024, 16384 );
}

void Controller::setRxCenterFrequency(uint64_t frequency) {
    this->rx_center_frequency = frequency ;
    if( radio != NULL ) {
        radio->setRxCenterFreq( frequency );
    }
}

void Controller::startAcquisition() {
    qDebug() << "Controller::startAcquisition() " ;
    if( !this->isRunning() )
        return ;

    if( m_state == Controller::csIdle ) {
        next_state = Controller::csStart ;
        qDebug() << "Controller::startAcquisition() change state " ;
    }
    return ;
}

void Controller::close() {
    m_stop = true ;
    radio->stopAcquisition() ;
    while( m_state != Controller::csEnded ) {
        QThread::msleep(10);
    }
    radio->close();
}

void Controller::setDetectionThreshold(float level) {
     processor->setDetectionThreshold(level);
}

void Controller::run() {
    next_state = m_state ;
    SampleFifo *fifo = NULL ;
    TYPECPX* samples ;
    int sample_count ;

    qDebug() << "Controller::run() " ;

    while( !m_stop ) {

        if( next_state != m_state ) {
            qDebug() << "transition from " << m_state << " to " << next_state ;
            m_state = next_state ;
        }

        switch( m_state ) {

        case Controller::csInit:
            if( radio == NULL) {
                break ;
            }
            fifo = radio->getFIFO() ;
            next_state = Controller::csIdle ;
            break ;

        case Controller::csIdle:
            QThread::msleep(100);
            break ;

        case Controller::csStart:
            processor->raz();
            channelizer->reset();
            channelizer->setCenterOfWindow( FRAME_CENTER );
            if( radio->startAcquisition() == 1 ) {
                next_state = Controller::csRun ;
            }
            break ;

        case Controller::csRun:
            if( fifo == NULL ) {
                next_state = Controller::csInit ;
                break ;
            }
            samples = (TYPECPX *)fifo->DequeueData( &sample_count, 0,  NULL, true );
            if( (samples == NULL ) || (sample_count==0)) {
                continue ;
            }
            process( samples, sample_count );
            break ;

        case Controller::csStop:
            radio->stopAcquisition() ;
            next_state = Controller::csIdle ;
            break ;
        }
    }
    m_state = csEnded ;
    qDebug() << "Controller::run()  ends" ;
}

void Controller::process( TYPECPX*samples, int L ) {
    TYPECPX* pt = samples ;
    TYPECPX out[STEP_SIZE] ;
    int rc ;
    int left = L ;

    if( L > FFT_SPECTRUM ) {
        generateSpectrum(samples);
        emit newSpectrumAvailable(FFT_SPECTRUM);
    }

    while( left > 0 ) {

        int qty = qMin(left, (int)STEP_SIZE) ;
        // qDebug() << "put [";
        rc = channelizer->put(pt, qty) ;
        // qDebug() << "]";
        if( rc < 0 ) {
            break ;
        }
        left -= qty ;
        pt += qty ;

        if( rc == GET_DATA_OUT ) {
            rc = channelizer->get( out, STEP_SIZE )  ;
            while( rc > 0 ) {
                processor->newData( out, rc, 110e3 );
                rc = channelizer->get( out, STEP_SIZE )  ;
            }
        }
    }


    free(samples);
    return ;
}

void Controller::generateSpectrum( TYPECPX *samples ) {
    int i,j ;
    double cpow = 2.0/FFT_SPECTRUM ;
    for (i = 0;  i < FFT_SPECTRUM;i++)
    {
        fftin[i][0] = samples[i].re * hamming_coeffs[i];
        fftin[i][1] = samples[i].im * hamming_coeffs[i];
    }
    fftwf_execute( plan );

    semspectrum->acquire(1);
    // neg portion of spectrum
    j = 0 ;
    for( i=FFT_SPECTRUM/2 ; i < FFT_SPECTRUM ; i++ ) {
        float a = fftin[i][0];
        float b = fftin[i][1];
        float modulus = sqrtf( a*a + b*b );
        double dbFs = 20*log10( cpow * modulus + 1e-9 );
        spectrum[j++] = dbFs ;
    }
    // pos spectrum
    for( i=0 ; i < FFT_SPECTRUM/2 ; i++ ) {
        float a = fftin[i][0];
        float b = fftin[i][1];
        float modulus = sqrtf( a*a + b*b );
        double dbFs = 20*log10( cpow * modulus + 1e-9 );
        spectrum[j++] = dbFs ;
    }
    semspectrum->release(1);
}

void  Controller::getSpectrum( double* values ) {
    if( values == NULL )
        return ;
    semspectrum->acquire(1);
    memcpy( values, spectrum, FFT_SPECTRUM*sizeof(double));
    semspectrum->release(1);
}

void Controller::SLOT_detectionLevel( float level )  {
    emit detectionLevel(level);
}

void Controller::SLOT_frameDetected(float signal_level, float noise_level, QString message )  {
    qDebug() << "signal_level:" << signal_level ;
    qDebug() << "message : " << message ;


    if( message.length() < 5 )
        return ;

    // parse frame
    long frameid ;
    float longitude ;
    float latitude  ;
    int altitude ;
    int roll, pitch, yaw ;

    QByteArray tmp = message.toLocal8Bit() ;
    char *rawframe = tmp.data() ;
    sscanf( rawframe, "%ld;%f;%f;%d;%d;%d;%d", &frameid,
            &longitude, &latitude, &altitude,
            &roll,&pitch,&yaw);

    qDebug() << " received frameid:" << frameid ;
    qDebug() << " received longitude:" << longitude ;
    qDebug() << " received latitude:" << latitude ;
    qDebug() << " received altitude:" << altitude ;
    qDebug() << " received signal_level:" << signal_level ;
    qDebug() << " received noise_level:" << noise_level ;

    sempos->acquire(1);
    float d = distance_between( m_Latitude, m_Longitude, latitude, longitude );
    float azimuth = course_to( m_Latitude, m_Longitude, latitude, longitude );
    sempos->release(1);
    float dh = (altitude/10.0-m_Altitude) ;
    float elevation = atan2(dh,d) * 180/M_PI;
    float dtotal  = sqrt( d*d + dh*dh) ;
    qDebug() << " distance :" << dtotal ;
    qDebug() << " azimuth :" << azimuth  << " degrés";
    qDebug() << " elevation :" << elevation  << " degrés";
    qDebug() << "roll:" <<roll ;
    qDebug() << "pitch:" << pitch ;
    qDebug() << "yaw:" << yaw ;

    emit frameDetected(signal_level, noise_level, message,
                       frameid, longitude, latitude, altitude/10.0,
                       m_Longitude, m_Latitude, m_Altitude,
                       elevation, azimuth, dtotal ,
                       roll, pitch, yaw);

    FILE *rbee_log = fopen(  "rbee.dat", "a");
    if( rbee_log != NULL ) {
        fprintf( rbee_log, "%ld;%f;%f;", frameid, signal_level, noise_level );
        fprintf( rbee_log, "%01.7f;%01.7f;%f;", longitude, latitude, (float)(altitude/10.0) );
        fprintf( rbee_log, "%01.7f;%01.7f;%f;", m_Longitude, m_Latitude, m_Altitude );
        fprintf( rbee_log, "%f;%f;%f;", elevation, azimuth, dtotal );
        fprintf( rbee_log, "%d;%d;%d\n", roll, pitch, yaw );
        fclose( rbee_log );
    }

}

void Controller::SLOT_hasGpsFix(double latitude, double longitude, double altitude) {
    sempos->acquire(1);
    m_Latitude = latitude ;
    m_Longitude = longitude ;
    m_Altitude = altitude ;
    sempos->release(1);
}

void Controller::SLOT_hasGpsTime(int year, int month, int day, int hour, int min, int sec, int msec) {

}

static float radians(float angle) { return( angle/180*3.14159265358979323846) ; }
static float degrees(float rad  ) { return( rad/3.14159265358979323846*180); }
static float sq( float x ) { return( x*x ) ; }


float Controller::distance_between (float lat1, float long1, float lat2, float long2)
{
    // returns distance in meters between two positions, both specified
    // as signed decimal-degrees latitude and longitude. Uses great-circle
    // distance computation for hypothetical sphere of radius 6372795 meters.
    // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
    // Courtesy of Maarten Lamers
    float delta = radians(long1-long2);
    float sdlong = sin(delta);
    float cdlong = cos(delta);
    lat1 = radians(lat1);
    lat2 = radians(lat2);
    float slat1 = sin(lat1);
    float clat1 = cos(lat1);
    float slat2 = sin(lat2);
    float clat2 = cos(lat2);
    delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
    delta = sq(delta);
    delta += sq(clat2 * sdlong);
    delta = sqrt(delta);
    float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
    delta = atan2(delta, denom);
    return delta * 6372795;
}

float Controller::course_to (float lat1, float long1, float lat2, float long2)
{
    // returns course in degrees (North=0, West=270) from position 1 to position 2,
    // both specified as signed decimal-degrees latitude and longitude.
    // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
    // Courtesy of Maarten Lamers
    float dlon = radians(long2-long1);
    lat1 = radians(lat1);
    lat2 = radians(lat2);
    float a1 = sin(dlon) * cos(lat2);
    float a2 = sin(lat1) * cos(lat2) * cos(dlon);
    a2 = cos(lat1) * sin(lat2) - a2;
    a2 = atan2(a1, a2);
    if (a2 < 0.0)
    {
        a2 += M_PI*2;
    }
    return degrees(a2);
}
