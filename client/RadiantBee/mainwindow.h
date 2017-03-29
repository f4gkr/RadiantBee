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
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLCDNumber>

#include "hardware/rtlsdr.h"
#include "ui/freqctrl.h"
#include "ui/spectrumplot.h"
#include "ui/indicatorwidget.h"
#include "ui/qcustomplot.h"
#include "ui/gkdial.h"
#include "qwt/qwt_plot_zoneitem.h"
// inspired from the Qwt examples
class SpectrumSegment: public QwtPlotZoneItem
{
public:
    SpectrumSegment( const QString &title )
    {
        setTitle( title );
        setZ( 999 ); // on top the the grid
        setOrientation( Qt::Vertical );
        setItemAttribute( QwtPlotItem::Legend, true );
    }

    void setColor( const QColor &color )
    {
        QColor c = color;

        c.setAlpha( 10 );
        setPen( c );

        c.setAlpha( 80 );
        setBrush( c );
    }

    void setInterval( double fmin, double fmax )
    {
        QwtPlotZoneItem::setInterval( fmin,fmax );
    }
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void setRadio( RTLSDR* device );

public slots:
    void SLOT_detectionLevel( float level ) ;
    void SLOT_frameDetected(float signal_level, float noise_level, QString message , int frameid, float uavlongitude, float uavlatitude, float uavaltitude, float antenna_longitude, float antenna_latitude, float antenna_altitude, float elevation, float azimuth, float distance, float uav_roll, float uav_pitch, float uav_yaw) ;

    void  SLOT_hasGpsFix( double latitude, double longitude , double altitude );
    void  SLOT_hasGpsTime( int year, int month, int day,
                     int hour, int min, int sec, int msec );

private slots:
    void SLOT_userTunesFreqWidget(qint64 newFrequency);
    void SLOT_newSpectrum(int len  );
    void SLOT_startPressed();

    void SLOT_setRxGain(int) ;
    void SLOT_setDetectionThreshold(int);

private:
    int received_frame ;
     SpectrumSegment *seg_rx ;
     CFreqCtrl *mainFDisplay ;
     gkDial *gain_rx ;
     gkDial *detection_threshold ;

     QLineEdit *uavFrame ;

     QLCDNumber *zuluDisplay ;
     QLCDNumber *z_latitude ;
     QLCDNumber *z_longitude ;
     QLCDNumber *z_altitude  ;

     QLineEdit *uavFrameID ;
     QLineEdit *uavLatitude ;
     QLineEdit *uavLongitude ;
     QLineEdit *uavAltitude ;

     QLineEdit *uavDistance ;
     QLineEdit *uavElevation;
     QLineEdit *uavAzimuth ;

     RTLSDR* radio ;
     SpectrumPlot *plot ;
     IndicatorWidget *levelWidget ;
     QCustomPlot *detection_plot ;

};

#endif // MAINWINDOW_H
