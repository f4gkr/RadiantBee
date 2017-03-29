//==========================================================================================
// + + +   This Software is released under the "Simplified BSD License"  + + +
// Copyright 2014 F4GKR Sylvain AZARIAN . All rights reserved.
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
#include <QDebug>
#include "radarview.h"
#include "accessories/antennasystem.h"
#include "core/globaldata.h"
#include "qwt/qwt_symbol.h"

#define RV_DEBUG (0)

class MyPicker: public QwtPolarPicker
{
public:
    MyPicker( QwtPolarCanvas *canvas ):
        QwtPolarPicker( canvas )
    {
        setStateMachine( new QwtPickerDragPointMachine() );
        setRubberBand( QwtPicker::NoRubberBand );
        setTrackerMode( ActiveOnly );
    }

    virtual QwtText trackerTextPolar( const QwtPointPolar &pos ) const
    {
        QColor bg( Qt::white );
        bg.setAlpha( 200 );

        QwtText text = QwtPolarPicker::trackerTextPolar( pos );
        text.setBackgroundBrush( QBrush( bg ) );
        return text;
    }
};

// Pointless synthetic data, showing something nice

class SpectrogramData: public QwtRasterData
{
public:
    SpectrogramData( double *pt, int length )
    {
        setInterval( Qt::ZAxis, QwtInterval( 0.0, 10.0 ) );
        m_mode = MODE_SPECTRO_POLAR ;
        m_azimuth_divs = 360 ;
        m_radius_divs = 100 ;
        data_length = length ;
        this->datas = pt ;
    }

    ~SpectrogramData() {
        free(datas);
    }

    virtual double value( double azimuth, double radius ) const
    {
        if( azimuth > 360 ) azimuth = 360 ;
        if( radius > 100  ) radius  = 100 ;
        const int a = (int)( azimuth / 360 * m_azimuth_divs );
        const int r = (int)( radius / 100 * m_radius_divs );
        const int k = a*m_radius_divs + r ;
        if( k < data_length ) {
            double v = datas[k] ;
            return(v );
        }
        return(0);
    }

private:
    int m_mode ;
    int m_azimuth_divs ;
    int m_radius_divs ;
    double *datas ;
    int data_length ;
};

class AzimuthScaleDraw: public QwtRoundScaleDraw
{
public:
    virtual QwtText label( double value ) const
    {
        QwtText text;

        if ( qFuzzyCompare( fmod( value, 360.0 ), 0.0 ) )
        {
            return QString( "0" );
        }

        if ( qFuzzyCompare( fmod( value, 45.0 ), 0.0 ) )
        {
            text = QLocale().toString( value );
            return text;
        }

        return QwtRoundScaleDraw::label( value );
    }
};



RadarView::RadarView(QWidget *parent) :
    QwtPolarPlot(parent )
{
    // init vars for data storage
    m_mode = MODE_SPECTRO_POLAR ;
    m_azimuth_divs = 360 ;
    m_radius_divs = 100 ;
    current_azimuth = 0 ;
    tabLength = m_azimuth_divs * m_radius_divs * sizeof( double ) ;
    sem = new QSemaphore(1);

    datas = (double *)malloc( tabLength );
    for( int y=0 ; y < m_azimuth_divs ; y++ ) {
         int k = y*m_radius_divs ;
         for( int x=0 ; x < m_radius_divs ; x++ ) {
              datas[k+x] = 0 ;
         }
    }

    //--------------------------------
    setAutoReplot( false );
    setPlotBackground( Qt::darkBlue );
    setAzimuthOrigin( M_PI/2 );

    // scales
    setScale( QwtPolar::Azimuth, 0.0, 360.0, 30.0 );
    setScaleMaxMinor( QwtPolar::Azimuth, 2 );

    setScale( QwtPolar::Radius, 0.0, 100.0 );
    setScaleMaxMinor( QwtPolar::Radius, 2 );

    // grids
    d_grid = new QwtPolarGrid();
    d_grid->setPen( QPen( Qt::white ) );
    for ( int scaleId = 0; scaleId < QwtPolar::ScaleCount; scaleId++ )
    {
        d_grid->showGrid( scaleId );
        d_grid->showMinorGrid( scaleId );

        QPen minorPen( Qt::gray );
        d_grid->setMinorGridPen( scaleId, minorPen );
    }
    d_grid->setAxisPen( QwtPolar::AxisAzimuth, QPen( Qt::black ) );
    d_grid->setAzimuthScaleDraw( new AzimuthScaleDraw() );
    d_grid->showAxis( QwtPolar::AxisAzimuth, true );
    d_grid->showAxis( QwtPolar::AxisLeft, false );
    d_grid->showAxis( QwtPolar::AxisRight, true );
    d_grid->showAxis( QwtPolar::AxisTop, false );
    d_grid->showAxis( QwtPolar::AxisBottom, false );
    d_grid->showGrid( QwtPolar::Azimuth, true );
    d_grid->showGrid( QwtPolar::Radius, true );
    d_grid->attach( this );

    // spectrogram

    d_spectrogram = new QwtPolarSpectrogram();
    d_spectrogram->setPaintAttribute(
                QwtPolarSpectrogram::ApproximatedAtan, true );
    d_spectrogram->setRenderThreadCount( 1 );
    d_spectrogram->setData( new SpectrogramData( datas, tabLength ) );
    d_spectrogram->attach( this );

    d_spectrogram->setZ( 1.0 );
    d_grid->setZ( 2.0 );

    QwtPolarPicker *picker = new MyPicker( canvas() );
    picker->setMousePattern( QwtEventPattern::MouseSelect1, Qt::RightButton );

    QwtPolarMagnifier *magnifier = new QwtPolarMagnifier( canvas() );
    magnifier->setMouseButton( Qt::RightButton, Qt::ShiftModifier );

    new QwtPolarPanner( canvas() );


    AntennaSystem& as = AntennaSystem::getInstance();
    connect( &as, SIGNAL(SIG_AntennaOrientationChanged(float,float)),
             this, SLOT(SLOT_AntennaOrientationChanged(float,float)), Qt::QueuedConnection );

    // markers
    marker = new QwtPolarMarker();
    marker->setPosition( QwtPointPolar( as.getAntennaAzimuth(), 99 ) );
    marker->setSymbol( new QwtSymbol( QwtSymbol::Ellipse, QBrush( Qt::white ), QPen( Qt::green ), QSize( 9, 9 ) ) );
    marker->setLabelAlignment( Qt::AlignHCenter | Qt::AlignTop );

    QwtText text( tr("Antenne") );
    text.setColor( Qt::black );
    QColor bg( Qt::white );
    bg.setAlpha( 200 );
    text.setBackgroundBrush( QBrush( bg ) );

    marker->setLabel( text );
    marker->attach( this );
    setAutoReplot( true );

    timer = new QTimer();

    connect(timer, SIGNAL(timeout()),
                this, SLOT(SLOT_update()));
}

void RadarView::start() {
     timer->start( 1000 );
}


QwtPolarSpectrogram *RadarView::spectrogram()
{
    return d_spectrogram;
}

void RadarView::SLOT_AntennaOrientationChanged( float newAzimuth, float newElevation ) {
    Q_UNUSED(newElevation);
    if( newAzimuth < 0 ) newAzimuth = 0 ;

    sem->acquire(1);
    marker->setPosition( QwtPointPolar( newAzimuth, 99 ) );
    if( RV_DEBUG ) qDebug() << "RadarView::SLOT_AntennaOrientationChanged" << newAzimuth ;
    current_azimuth = (int)( newAzimuth / 360.0 * m_azimuth_divs );

    if( current_azimuth < 0 ) current_azimuth = 0 ;
    if( current_azimuth > m_azimuth_divs )
        current_azimuth = m_azimuth_divs ;
    sem->release(1);
    //SLOT_update();
}

void RadarView::SLOT_update() {
     double *power_dB ;
     int length ;
     int bw ;

     power_dB = NULL ;
     // recover a copy of last saved spectrum
     GlobalRFData& gd = GlobalRFData::getInstance();
     gd.getLastGlobalSpectrum( &length, &power_dB, &bw );
     if( length == 0 ) return ;
     if( power_dB == NULL ) return ;

     if( RV_DEBUG ) qDebug() << "RadarView::SLOT_update" << length  ;
     setSpectrum( power_dB, length, (double)bw);
     free( power_dB );
}

void RadarView::setSpectrum(double *power_dB, int length, double bw ) {
    int i,j,k,  SUBSAMPLE, z  ;
    double x,y ;

    Q_UNUSED(bw);
    sem->acquire(1);
    datas = (double *)malloc( tabLength );
    int l = current_azimuth*m_radius_divs ;
    if( m_mode == MODE_SPECTRO_POLAR ) {        
        SUBSAMPLE = (int)( length / m_radius_divs ) ;
        if( RV_DEBUG ) qDebug() << "RadarView::setSpectrum" << length << SUBSAMPLE << current_azimuth ;
        k = 0 ;
        for( i=0 ; i < length ; i+=SUBSAMPLE ) {
            x = power_dB[i];
            for( j=i ; j < (i+SUBSAMPLE) && j<length ; j++ ) {
                if( power_dB[j] > x ) {
                    x = power_dB[j];
                }
            }
            //x /= SUBSAMPLE ;
            y = x/7.0 + 15 ; // convert -120 -40 dB to [0..10]

            if( y > 10 ) y = 10 ;
            if( y < 0 )  y = 0 ;
            z = l+k ;
            if( z < tabLength )
                datas[z] = y ;
            k++ ;
        }
    } else {
        for( i=0 ; i < m_radius_divs ; i++ ) {
            z = l+i ;
            if( z < tabLength )
                datas[z] = 0 ;
        }

    }
    SpectrogramData *sd = new SpectrogramData( datas, tabLength ) ;
    d_spectrogram->setData( sd );
    sem->release(1);
    //repaint();


}
