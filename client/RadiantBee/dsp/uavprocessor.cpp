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
#include "uavprocessor.h"
#include <stdlib.h>
#include <QDebug>
#include <math.h>


#define UAV_BITRATE (800)
#define UAV_BYTE_LENGTH (UAV_BITRATE*10)
#define UAV_MSG_LENGTHAT4MHZ (1e6+800)

UAVProcessor::UAVProcessor(QObject *parent) : QObject(parent)
{
    samples = NULL ;
    m_bandwidth = 0 ;
    chirp_length = 0 ;
    fft_length = 32768*2  ;
    wr_pos = 0 ;
    synched = false ;
    detection_threshold = 30 ;
    fftin = (fftwf_complex *) fftwf_malloc(sizeof(fftwf_complex) * fft_length);
    fftout = (fftwf_complex *) fftwf_malloc(sizeof(fftwf_complex) * fft_length);
    fftchirp  = (fftwf_complex *) fftwf_malloc(sizeof(fftwf_complex) * fft_length);

    fft_plan = fftwf_plan_dft_1d(fft_length, fftin, fftout, FFTW_FORWARD, FFTW_ESTIMATE );
    fft_plan_inv = fftwf_plan_dft_1d(fft_length, fftout, fftin, FFTW_BACKWARD, FFTW_ESTIMATE );

    m_state = next_state = UAVProcessor::sInit ;
}

void UAVProcessor::setDetectionThreshold(float level) {
     detection_threshold = level ;
}

//  IFFT( FFT(signal entrant) .* conj( FFT(Signal reference))
// on calcule ici conj( FFT(Signal reference)
void UAVProcessor::generate(TYPEREAL bandwidth, TYPEREAL sampleRate, long length) {
    double t = 0 ;
    int n = 0 ;

    msg_length = (int)(sampleRate*.75) ;
    if( samples != NULL ) {
         free( samples );
    }

    samples =(TYPECPX *)malloc( msg_length * sizeof(TYPECPX));
    msg_wrpos = 0 ;

    double Tech = 1.0/sampleRate ;
    double k = bandwidth / (length * Tech);
    double fstart = -bandwidth/2 ;

    for(n = 0  ; n < length ; n++ ) {
       double omega = K_2PI*( fstart + k/2*t ) ;
       float I =  (float)(  MCOS( omega * t)) ;
       float Q  = (float)(  MSIN( omega * t)) ;
       fftin[n][0] = I  ;
       fftin[n][1] = Q ;
        t += Tech ;
    }
    // zero padding
    for( ; n < fft_length ; n++ ) {
        fftin[n][0] = 0;
        fftin[n][1] = 0 ;
    }
    fftwf_execute( fft_plan );

    // store complex conjugate of FFT(chirp)
    for( n = 0  ; n < fft_length ; n++ ) {
          fftchirp[n][0] =   1.0/fft_length  * fftout[n][0] ;
          fftchirp[n][1] =  -1.0/fft_length  * fftout[n][1] ;
    }

    if( false ) {
        FILE *f = fopen( "chirp.dat", "wb" );
        fwrite( fftchirp, sizeof( TYPECPX), fft_length, f );
        fclose( f );
    }

    ratio_min = 100 ;
    ratio_max = -100 ;
    threshold = 0 ;
}

void UAVProcessor::raz() {
    wr_pos = 0 ;
    msg_wrpos = 0 ;
    synched = false ;
}

QString UAVProcessor::stateToS(int s) {
    switch(s) {
    case sInit: return( QString("sInit"));
    case sSearch: return( QString("sSearch"));
    case sChirpFound: return( QString("sChirpFound"));
    case sDecodeFSK: return( QString("sDecodeFSK"));
    case sMeasureNoise: return( QString("sMeasureNoise"));
    }
    return("");
}

void UAVProcessor::newData( TYPECPX* IQsamples, int L , int sampleRate ) {
    int i,k ;
    int p ;
   double v ;
  char *msg ;

    if( sampleRate < 100e3 ) {
        qDebug() << "bandwith " << sampleRate << " too small" ;
        return ;
    }

    if(  sampleRate != m_bandwidth ) {
        // regenerate the reference chirp
        // ref chirp is 100 KHz wide,  250 millisecond long
        chirp_length = sampleRate/4 ;
        m_bandwidth = sampleRate ;
        generate( 100e3, sampleRate, chirp_length );
        qDebug() << "'chirp length is " << chirp_length ;
        synched = false ;
        return ;
    }
    i= 0 ;
    while( i < L ) {

        // concatenate received samples with previous ones
        for(  ; (i < L) && (msg_wrpos <msg_length) ; i++  ) {
            samples[msg_wrpos++] = IQsamples[i] ;
        }

        if( next_state != m_state ) {
            m_state = next_state ;
            //qDebug() << "UAVProcessor::newData : " << stateToS( m_state ) << " msg_wrpos=" << msg_wrpos;
        }


        switch( m_state ) {
        case sInit:
            signal_plus_noise = 0 ;
            noise_only = 0 ;
            next_state = sSearch ;
            break ;

        case sSearch:
            // check we have enough samples
            if( msg_wrpos < fft_length )  {
                continue ;
            }
            // search for chirp - synchro
            // copy n (=fft_length) first samples to FFT buffer and search for chirp inside
            memcpy( (void *)fftin, (void *)samples, fft_length*sizeof(TYPECPX));
            p = calc(NULL);
            // check that we found a chirp starting in the n first samples
            // if not shift left and exit
            if( p < 0 ) {
                int shift_samples_count = chirp_length ;
                memmove( (void *)samples, (void *)&samples [shift_samples_count] ,(msg_length-shift_samples_count)*sizeof(TYPECPX));
                msg_wrpos -= shift_samples_count ;
                break ;
            }
            next_state = sChirpFound ;
            if( p > 0 ) {
                // a chirp was found at position p, discard samples before p
                memmove( (void *)samples, (void *)&samples [p] ,(msg_length - p)*sizeof(TYPECPX));
                msg_wrpos -= p ;
            }
            break ;

        case sChirpFound:
            if( msg_wrpos < fft_length )  {
                continue ;
            }
            signal_plus_noise = rmsp( samples, chirp_length );
            if( false ) {
                FILE *f = fopen( "signal_plus_noise.dat", "wb" );
                fwrite( samples, sizeof( TYPECPX), chirp_length, f );
                fclose( f );
            }
            // discard and shift left by chirp_length samples - one bit @UAV_BITRATE
            k = (int)(.8 * chirp_length) ; // - 10*m_bandwidth/UAV_BITRATE ;
            memmove( (void *)samples, (void *)&samples [k] ,(msg_length - k)*sizeof(TYPECPX));
            msg_wrpos -= k ;
            next_state = sDecodeFSK ;
            break ;

        case sDecodeFSK:

            if( msg_wrpos < 3*chirp_length )  {
                continue ;
            }
            k = 0 ;
            msg = demod( samples,  msg_wrpos, &k  ) ;
            if( k > 0 ) {
                // we consumed k samples for the FSK frame, shift
                memmove( (void *)samples, (void *)&samples [k] ,(msg_length - k)*sizeof(TYPECPX));
                msg_wrpos -= k ;
            }

            if( msg == NULL ) {
                next_state = sSearch ;
                continue ;
            }

            lastFrameReceived =  QString::fromLocal8Bit(msg);
            next_state = sMeasureNoise ;
            break;

        case sMeasureNoise:
            if( msg_wrpos <chirp_length )  {
                continue ;
            }
             noise_only = rmsp( samples+5000, chirp_length/2 );
            v = 20*log10( signal_plus_noise - noise_only ) ;
            emit frameDetected( v, 20*log10(noise_only),lastFrameReceived);
            memmove( (void *)samples, (void *)&samples [chirp_length] ,(msg_length - chirp_length)*sizeof(TYPECPX));
            msg_wrpos -= chirp_length ;
            next_state = sSearch ;
            break;
        }
    }

}

float UAVProcessor::rmsp( TYPECPX *samples, int L ) {
      float res = 0 ;
      int k = 0 ;
      for( k=0 ; k < L ; k++ ) {
              res += samples[k].re*samples[k].re + samples[k].im*samples[k].im ;
      }
      res = sqrtf( 1.0/L * res );
      qDebug() << "rmsp = " << res ;
      return(res);
}

//Normalize to [-180,180):
inline double constrainAngle(double x){
    x = fmod(x + M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}
// convert to [-360,360]
inline double angleConv(double angle){
    return fmod(constrainAngle(angle),2*M_PI);
}
inline double angleDiff(double a,double b){
    double dif = fmod(b - a + M_PI,2*M_PI);
    if (dif < 0)
        dif += 2*M_PI;
    return dif - M_PI;
}

inline double unwrap(double previousAngle,double newAngle){
    return previousAngle - angleDiff(newAngle,angleConv(previousAngle));
}

#define LPF_TAPS (11)
double lpf[] = {
       0.014591982220007269,  0.030622195905371408,  0.072593412916822475, 0.12448046405006548,
       0.16646244448643438, 0.1824990008425979, 0.16646244448643438, 0.12448046405006548,
       0.072593412916822475,  0.030622195905371408,  0.014591982220007269};

char*  UAVProcessor::demod(TYPECPX* psamples, int Lmax , int *consumed) {
         float last_angle = 0 ;
         char rxmsg[255];
         int decimation = m_bandwidth/UAV_BITRATE ;
         int symbol = (800 * m_bandwidth) / 4e6 + decimation/2 ;
         TYPECPX work[Lmax] ;
        float P = 0 ;
         if( false ) {
             FILE *f = fopen( "samples.dat", "wb" );
             fwrite( psamples, sizeof( TYPECPX), Lmax, f );
             fclose( f );
             qDebug() << "writing debug files samples.dat with " << Lmax << " samples" ;
         }

        (*consumed) = 0 ;
         // calc dphi/dt
         float pmoy = 0 ;
         float pmin = 1 ;
         float pmax = 0 ;
         for( int i=0 ; i < Lmax ; i++ ) {
                 float I =  psamples[i].re ;
                 float Q =  psamples[i].im ;
                 float angle = atan2f(Q , I)  ;

                 angle = unwrap( last_angle, angle );
                 float diff = angle-last_angle ;
                 // partie réelle = phase
                 work[i].re = diff ;

                 // partie imaginaire = puissance
                 P = .8*P + .2*sqrtf( I*I + Q*Q ) ;
                 pmoy += P ;
                 work[i].im = P ;
                 last_angle = angle ;
                 if( P < pmin ) {
                      pmin = P ;
                 }
                 if( P > pmax ) {
                      pmax = P ;
                 }
         }
        pmoy /= Lmax ;
        float power_threshold  = pmin + (pmax-pmin)/3 ;
        //qDebug() << "threshold:" << power_threshold << " pmin=" << pmin << "pmax=" << pmax << " pmoy=" << pmoy ;
        // low pass filter phase sur psamples[i].re
        int s = 0 ;
        for( int i=LPF_TAPS-1 ; i < Lmax ; i++ ) {
                double acc = 0 ;
                for( int p=0 ; p < 11 ; p++ ) {
                        acc += lpf[p] * work[i-p].re ;
                }
                work[s++].re = acc ;
        }

        if( false ) {
           FILE *f = fopen( "demod.dat", "wb" );
           fwrite( work, sizeof( TYPECPX), Lmax, f );
           qDebug() << "writing debug files demod.dat with " << Lmax << " samples" ;
           fclose( f );
       }

        // search for silence between chirp and fsk sequence
        int start = 0 ;
        int L = 0 ;
        // search up
        while( (work[start].im < power_threshold) && (start < Lmax)) {
            start++ ;
        }
        // stay up
        while( (work[start].im > power_threshold) && (start < Lmax)) {
            start++ ;
        }
        if( start == Lmax ) {
            qDebug() << "did not find down transition" ;
            (*consumed) = Lmax/2 ;
            return(NULL);
        }
        //qDebug() << "down transition at " << start ;
        start += 10 ;
        while( (work[start].im < power_threshold) && (start < Lmax)) {
            start++ ;
            L++ ;
        }

        if( (start == Lmax) || (L==0)) {
           // qDebug() << "did not find up transition" ;
            (*consumed) = Lmax/2 ;
            return(NULL);
        }

         TYPECPX *position = work ;
         position += start ;
         int len = 0 ;
         bool frame = false ;
         rxmsg[len] = (char)0 ;
         QByteArray deb ;
         (*consumed) = start + LPF_TAPS ;

         while( symbol < s ) {
                // demod FSK octet / octet
                int v = demodByte( position, &symbol,Lmax-start);
                if( v < 0 ) {
                      return(NULL);
                }
                deb.append((char)v);
                //qDebug() << deb ;
                if( !frame && (v==(int)'[')) {
                    frame = true ;
                    continue ;
                }
                if( frame ) {
                     if( v == (char)']')  {
                          frame = false ;
                          break ;
                     }
                     rxmsg[len++] = (char)v ;
                     rxmsg[len] = (char)0 ;
                }
         }

         if( strlen(rxmsg) == 0 ) {
             //qDebug() << "empty frame"  << deb ;
             return(NULL);
         }

         //(*consumed) += (int)( (strlen(rxmsg) *10*m_bandwidth)/UAV_BITRATE );
         (*consumed) = symbol  + (10*m_bandwidth)/UAV_BITRATE;
         char *rx = (char *)malloc( strlen(rxmsg) * sizeof(char));
         strcpy( rx, rxmsg) ;
         return( rx );
}

int UAVProcessor::demodByte( TYPECPX* psamples, int *pstart, int L ) {
    int mask = 128 ;
    int v = 0 ;
    int cpt   ;
    int pos = (*pstart);
    int decimation = m_bandwidth/UAV_BITRATE ;

    // search start and stop bit
    while( ((pos+10*decimation)<L) &&
           (psamples[pos].re <  0  )  && // start bit
           (psamples[pos+10*decimation].re < 0) // stop bit
           ) {
        pos+=decimation/4 ;
    }

    if( pos >= L ) {
         return(-1);
    }
    if( (pos+10*decimation)>=L)
        return(-1);

    pos += decimation ;
    // built byte from 8 bits
    // MSB first
    for( cpt = 0 ; cpt < 8 ; cpt++ ) {
            int bit = ( psamples[pos].re > 0 ? 1:0) ;
            v += bit * mask ;
            pos+=decimation ;
            mask /= 2 ;
    }
    // recalage en fonction decimation non entiere
    int comp = (int)( (10*m_bandwidth)/UAV_BITRATE - 10*decimation );
    pos+=decimation + comp ;
    (*pstart ) = pos ;
    return(v);
}

int  UAVProcessor::calc( float *pvmax  ) {
     int i ;
     float vmax = -9999 ;
     int pmax = -1 ;

     if( false ) {
         FILE *f = fopen( "fftin.dat", "wb" );
         fwrite( fftin, sizeof( TYPECPX), fft_length, f );
         fclose( f );
     }

     fftwf_execute( fft_plan );
     // kron
     for( int k=0 ; k < fft_length ; k++ ) {
            float A = 1.0/fft_length * fftout[k][0] ;
            float B =  1.0/fft_length * fftout[k][1] ;
            float C = fftchirp[k][0];
            float D = fftchirp[k][1];
           // (A+jB) * (C+jD)
            fftout[k][0] = A*C  - B*D ;
            fftout[k][1] = A*D + B*C ;
     }
     // calc IFFT( FFT( signal ) x conj( FFT(chirp)) )
      fftwf_execute( fft_plan_inv );

      if( false ) {
          FILE *f = fopen( "kro.dat", "wb" );
          fwrite( fftin, sizeof( TYPECPX), fft_length, f );
          fclose( f );
      }

      // search for max
      float vmoy = 0 ;
      for( i=0 ; i < fft_length ; i++ ) {
            float v = sqrtf( fftin[i][0]*fftin[i][0] + fftin[i][1]*fftin[i][1]) ;
            vmoy += v ;
            if( v > vmax ) {
                 vmax = v ;
                 pmax = i ;
            }
      }
      vmoy /= fft_length ;

      float ratio =  20*log10( vmax/vmoy + 1e-9) ;
      if( pvmax != NULL ) {
           (*pvmax) = vmax ;
      }

     // qDebug() << "ratio:" << ratio << " vmax=" << vmax << " vmoy=" << vmoy << " threshold=" << detection_threshold;
     emit detectionLevel(ratio);
      if( ratio < detection_threshold ) {
          //qDebug() << "ratio < detection_threshold  " << ratio << " vmax=" << vmax << " vmoy=" << vmoy << " threshold=" << detection_threshold;
          return(-1);
       }


      return(pmax);
}
