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
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QDesktopWidget>
#include <QApplication>
#include <QLabel>
#include <QPalette>
#include <stdint.h>

#include "core/controller.h"
#include "hardware/gpdsd.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    radio = NULL ;
    setAttribute(Qt::WA_DeleteOnClose);

    QWidget *center_widget = new QWidget;
    QVBoxLayout *vlayout = new QVBoxLayout;
    center_widget->setLayout(vlayout);
    vlayout->setContentsMargins( 1,1,1,1);
    vlayout->setAlignment( Qt::AlignTop );

    // Create top band
    QWidget *top_band = new QWidget;
    QHBoxLayout *tb_layout = new QHBoxLayout;
    tb_layout->setAlignment( Qt::AlignLeft );
    tb_layout->setContentsMargins( 1,1,1,1);

    QPushButton *startButton = new QPushButton("START");
    startButton->setMaximumWidth(50);
    startButton->setToolTip(tr("Start SDR"));
    tb_layout->addWidget( startButton );

    QPushButton *stopButton = new QPushButton("STOP");
    stopButton->setMaximumWidth(50);
    stopButton->setToolTip(tr("Stop SDR"));
    tb_layout->addWidget( stopButton );

    mainFDisplay = new CFreqCtrl();
    mainFDisplay->setMinimumWidth(200);
    mainFDisplay->setMinimumWidth(500);
    tb_layout->addWidget(mainFDisplay);
    top_band->setMaximumHeight(40);
    top_band->setLayout(tb_layout);
    vlayout->addWidget( top_band );

    // center band
    QWidget* center_band = new QWidget();
    QHBoxLayout *cb_layout = new QHBoxLayout;
    center_band->setLayout(cb_layout);
    cb_layout->setAlignment( Qt::AlignLeft | Qt::AlignTop );
    cb_layout->setContentsMargins( 1,1,1,1);

    plot = new SpectrumPlot();
    plot->setMinMaxScales( -90, -40 );
    plot->setMinimumHeight( 150 );
    cb_layout->addWidget( plot );

    // center left
    QWidget *cl_widget = new QWidget;
    cl_widget->setMaximumWidth(170);
    QVBoxLayout *cllayout = new QVBoxLayout;
    cl_widget->setLayout(cllayout);
    cllayout->setContentsMargins( 1,1,1,1);
    cllayout->setAlignment(Qt::AlignLeft | Qt::AlignTop  );

    gain_rx = new gkDial(4,tr("RF Gain"));
    gain_rx->setScale(0,40);
    cllayout->addWidget(gain_rx);

    detection_threshold= new gkDial(4,tr("Correlator"));
    detection_threshold->setScale(5,40);
    detection_threshold->setValue(30);
    cllayout->addWidget(detection_threshold);

    zuluDisplay = new QLCDNumber(11);
    zuluDisplay->setSegmentStyle(QLCDNumber::Flat);
    zuluDisplay->display( "00:00:00.0" );
    zuluDisplay->setToolTip(tr("UTC Time"));
    QPalette zpalette = zuluDisplay->palette() ;
    zpalette.setColor( zpalette.WindowText, QColor(85, 85, 255)) ;
    zpalette.setColor( zpalette.Background, QColor(0, 170, 255));
    zpalette.setColor (zpalette.Dark, QColor(0, 255, 0));
    zuluDisplay->setPalette(zpalette) ;
    cllayout->addWidget(zuluDisplay);

    z_latitude = new QLCDNumber(11);
    z_latitude->setSegmentStyle(QLCDNumber::Flat);
    z_latitude->display( "0.0000000" );
    z_latitude->setToolTip(tr("Latitude"));
    z_latitude->setPalette(zpalette) ;
    cllayout->addWidget(z_latitude);

    z_longitude = new QLCDNumber(11);
    z_longitude->setSegmentStyle(QLCDNumber::Flat);
    z_longitude->display( "0.0000000" );
    z_longitude->setToolTip(tr("Longitude"));
    z_longitude->setPalette(zpalette) ;
    cllayout->addWidget(z_longitude);

    z_altitude = new QLCDNumber(8);
    z_altitude->setSegmentStyle(QLCDNumber::Flat);
    z_altitude->display( "0.0" );
    z_altitude->setToolTip(tr("Altitude"));
    z_altitude->setPalette(zpalette) ;
    cllayout->addWidget(z_altitude);

    levelWidget = new IndicatorWidget( "Level", 0, 80, "dBc");
    levelWidget->setMinimumWidth(150);
    levelWidget->setMaximumHeight(150);
    cllayout->addWidget( levelWidget);
    //cllayout->addWidget( new QLabel(tr("Max Level:")));

    uavFrameID = new QLineEdit();
    uavFrameID->setToolTip(tr("UAV FrameID"));
    uavFrameID->setReadOnly(true);
    uavFrameID->setMaxLength(12);
    uavFrameID->setMaximumWidth(150);
    cllayout->addWidget( uavFrameID);

    uavLongitude = new QLineEdit();
    uavLongitude->setToolTip(tr("UAV Longitude"));
    uavLongitude->setReadOnly(true);
    uavLongitude->setMaxLength(12);
    uavLongitude->setMaximumWidth(150);
    cllayout->addWidget( uavLongitude);

    uavLatitude = new QLineEdit();
    uavLatitude->setToolTip(tr("UAV Latitude"));
    uavLatitude->setReadOnly(true);
    uavLatitude->setMaxLength(12);
    uavLatitude->setMaximumWidth(150);
    cllayout->addWidget( uavLatitude);

    uavAltitude = new QLineEdit();
    uavAltitude->setToolTip(tr("UAV Altitude"));
    uavAltitude->setReadOnly(true);
    uavAltitude->setMaxLength(10);
    uavAltitude->setMaximumWidth(150);
    cllayout->addWidget( uavAltitude);

    uavDistance = new QLineEdit();
    uavDistance->setToolTip(tr("UAV uavDistance"));
    uavDistance->setReadOnly(true);
    uavDistance->setMaxLength(5);
    uavDistance->setMaximumWidth(150);
    cllayout->addWidget( uavDistance);

    uavElevation = new QLineEdit();
    uavElevation->setToolTip(tr("UAV uavElevation"));
    uavElevation->setReadOnly(true);
    uavElevation->setMaxLength(5);
    uavElevation->setMaximumWidth(150);
    cllayout->addWidget( uavElevation);

    uavAzimuth = new QLineEdit();
    uavAzimuth->setToolTip(tr("UAV uavAzimuth"));
    uavAzimuth->setReadOnly(true);
    uavAzimuth->setMaxLength(5);
    uavAzimuth->setMaximumWidth(150);
    cllayout->addWidget( uavAzimuth);


    cb_layout->addWidget( cl_widget);

    // Create the bands showing the received portion
    seg_rx = new SpectrumSegment("DATA");
    seg_rx->setColor( Qt::green   );
    seg_rx->setInterval( FRAME_OFFSET_LOW/1e6, FRAME_OFFSET_HIGH/1e6 );
    seg_rx->setVisible( true );
    seg_rx->attach( plot );

    vlayout->addWidget( center_band );

    uavFrame = new QLineEdit();
    uavFrame->setMaximumHeight(20);
     vlayout->addWidget( uavFrame );

    // detection levels etc
    QWidget *plotWidget = new QWidget();
    QVBoxLayout *pwlayout = new QVBoxLayout();
    pwlayout->setContentsMargins( 0, 0, 0, 0 );
    plotWidget->setLayout( pwlayout );

    detection_plot = new QCustomPlot();
    detection_plot->addGraph();
    detection_plot->xAxis->setLabel(tr("Frame"));
    detection_plot->yAxis->setLabel("SNR");
    detection_plot->xAxis->setRange(0, 1);
    detection_plot->yAxis->setRange(0, 70);
    detection_plot->setMinimumHeight(150);
    pwlayout->addWidget( detection_plot );

    levelplot = new QCustomPlot();
    levelplot->addGraph();
    levelplot->xAxis->setLabel(tr("Frame"));
    levelplot->yAxis->setLabel("Level");
    levelplot->xAxis->setRange(0, 1);
    levelplot->yAxis->setRange(-70, -10);
    levelplot->setMinimumHeight(150);
    levelplot->addGraph();
    levelplot->graph(1)->setPen(QPen(Qt::red));
    min_level = 0 ;
    max_level = -100 ;

    pwlayout->addWidget( levelplot );

    vlayout->addWidget( plotWidget );


    setCentralWidget(center_widget);
    // resize
    QDesktopWidget *desktop = QApplication::desktop();
    resize( desktop->availableGeometry(this).size() * .7 );

    connect( startButton, SIGNAL(pressed()), this, SLOT(SLOT_startPressed()));
    connect( stopButton, SIGNAL(pressed()), this, SLOT(SLOT_stopPressed()));

    connect( mainFDisplay, SIGNAL(newFrequency(qint64)),
             this, SLOT(SLOT_userTunesFreqWidget(qint64)) );
    connect( gain_rx, SIGNAL(valueChanged(int)), this, SLOT(SLOT_setRxGain(int)));
    connect( detection_threshold, SIGNAL(valueChanged(int)), this, SLOT(SLOT_setDetectionThreshold(int)));

    GPSD& gpsd= GPSD::getInstance() ;
    connect( &gpsd, SIGNAL(hasGpsFix(double,double,double)), this,
             SLOT(SLOT_hasGpsFix(double,double,double)), Qt::QueuedConnection );
    connect( &gpsd, SIGNAL(hasGpsTime(int,int,int,int,int,int,int)), this,
             SLOT(SLOT_hasGpsTime(int,int,int,int,int,int,int)),Qt::QueuedConnection);

}

void MainWindow::setRadio( RTLSDR* device ) {
    radio = device ;
    mainFDisplay->setup( 11, radio->getMin_HWRx_CenterFreq() ,
                         radio->getMax_HWRx_CenterFreq(),
                         10, UNITS_MHZ );
    mainFDisplay->resetToFrequency( radio->getRxCenterFreq() );
    plot->setNewParams( radio->getRxCenterFreq(), radio->getRxSampleRate() ) ;

    Controller& ctrl = Controller::getInstance() ;
    connect( &ctrl, SIGNAL(newSpectrumAvailable(int, double, double)),  this,
             SLOT(SLOT_newSpectrum(int, double, double)), Qt::QueuedConnection );
    connect( &ctrl, SIGNAL(detectionLevel(float)), this, SLOT(SLOT_detectionLevel(float)), Qt::QueuedConnection );
    connect( &ctrl, SIGNAL(frameDetected(float,float,QString,int,float,float,float,float,float,float,float,float,float,float,float,float)),
             this, SLOT(SLOT_frameDetected(float,float,QString,int,float,float,float,float,float,float,float,float,float,float,float,float)),
             Qt::QueuedConnection );

    gain_rx->setValue( device->getRxGain()  );
}

void MainWindow::SLOT_userTunesFreqWidget(qint64 newFrequency) {    

    plot->razMaxHold();

    double fmin = (double)newFrequency + FRAME_OFFSET_LOW ;
    double fmax = (double)newFrequency + FRAME_OFFSET_HIGH ;

    seg_rx->setInterval(  fmin/1e6, fmax/1e6);

    Controller& ctrl = Controller::getInstance() ;
    ctrl.setRxCenterFrequency( newFrequency );
}

// start SDR pressed
void MainWindow::SLOT_startPressed() {
   qint64 newFrequency = 144832000 - 55000 ; //439795000 ;
   Controller& ctrl = Controller::getInstance() ;

    if( radio == NULL )
        return ;

    if( ctrl.isAcquiring()  )
            return ;

    received_frame = msg_count = 0 ;
    mainFDisplay->resetToFrequency(  newFrequency );
    plot->setNewParams(  newFrequency, radio->getRxSampleRate() ) ;
    double fmin = (double)newFrequency + FRAME_OFFSET_LOW ;
    double fmax = (double)newFrequency + FRAME_OFFSET_HIGH ;
    seg_rx->setInterval(  fmin/1e6, fmax/1e6);

    ctrl.setRxCenterFrequency( newFrequency  );
    ctrl.startAcquisition();
}

void MainWindow::SLOT_stopPressed() {
    Controller& ctrl = Controller::getInstance() ;
    if( radio == NULL )
        return ;
    if( !ctrl.isAcquiring() ) {
        return ;
    }
    ctrl.stopAcquisition();
}


void MainWindow::SLOT_newSpectrum( int len , double smin,  double smax ) {
    double power_dB[len] ;
    float bw ;
    bool rescale = false ;

    uint64_t rx_center_frequency = radio->getRxCenterFreq() ;
    bw = radio->getRxSampleRate() ;
    Controller& ctrl = Controller::getInstance() ;
    ctrl.getSpectrum( power_dB );

    if( smin < plot->getMinScale() ) {
         rescale = true ;
    } else {
        if( abs( smin - plot->getMinScale() ) > 25 ) {
             rescale = true ;
        }
    }

    if( smax > plot->getMaxScale() ) {
         rescale = true ;
    } else {
        if( abs( smax - plot->getMaxScale() ) > 25 ) {
             rescale = true ;
        }
    }

    if( rescale ) {
        plot->setMinMaxScales( .1*smin + .9*plot->getMinScale(),
                               .1*smax  + .9*plot->getMaxScale()  );
    }

    plot->setPowerTab(rx_center_frequency, power_dB,  len, bw );
    double fmin = (double)rx_center_frequency + FRAME_OFFSET_LOW ;
    double fmax = (double)rx_center_frequency +  FRAME_OFFSET_HIGH ;
    seg_rx->setInterval(  fmin/1e6, fmax/1e6);
}

MainWindow::~MainWindow()
{
    SLOT_stopPressed();

    GPSD& gpsd= GPSD::getInstance() ;
    gpsd.shutdown();

    Controller& ctrl = Controller::getInstance() ;
    ctrl.close();

}

void MainWindow::SLOT_detectionLevel( float level )  {
    //qDebug() << "level=" << level ;
    levelWidget->setValue( level );    
    detection_plot->graph(0)->addData( msg_count/10.0, level);
    detection_plot->xAxis->setRange( msg_count/10.0,60,Qt::AlignRight );
    detection_plot->replot();
    msg_count++ ;
}

void MainWindow::SLOT_frameDetected(  float signal_level, float noise_level, QString message,
                                       int frameid, float uavlongitude, float uavlatitude, float uavaltitude,
                                       float antenna_longitude, float antenna_latitude, float antenna_altitude,
                                       float elevation, float azimuth, float distance,
                                       float uav_roll, float uav_pitch, float uav_yaw
                                      )  {

    if( qMin(signal_level, noise_level) < min_level ) {
         min_level = qMin(signal_level, noise_level) ;
    }
    if( qMax(signal_level, noise_level) > max_level ) {
         max_level = qMax(signal_level, noise_level) ;
    }

    levelplot->graph(0)->addData( received_frame/10,  signal_level );
    levelplot->graph(1)->addData( received_frame/10,  noise_level );
    levelplot->xAxis->setRange( received_frame/10.0,60,Qt::AlignRight );
    levelplot->yAxis->setRange(min_level,max_level);
    levelplot->replot();
    received_frame++ ;

    uavLatitude->setText(QString::number( uavlatitude, 'f', 8));
    uavLongitude->setText(QString::number( uavlongitude, 'f', 8));
    uavAltitude->setText(QString::number( uavaltitude/10, 'f', 3));
    uavFrameID->setText( QString::number(frameid));

    uavDistance->setText(QString::number( distance, 'f', 2) + " m.");
    uavElevation->setText(QString::number( elevation, 'f', 1) + "  deg.");
    uavAzimuth->setText(QString::number( azimuth, 'f', 1) + "  deg.");
    uavFrame->setText( message );
}


void MainWindow::SLOT_hasGpsFix(double latitude, double longitude, double altitude) {
    z_latitude->display( QString::number( latitude, 'f', 8));
    z_longitude->display( QString::number( longitude, 'f', 8));
    z_altitude->display( QString::number( altitude, 'f', 2));
}

void MainWindow::SLOT_hasGpsTime(int year, int month, int day, int hour, int min, int sec, int msec) {
    Q_UNUSED(msec);
    QString zuluTime = QString("%1").arg(hour, 2, 10, QChar('0')) + ":" +
            QString("%1").arg(min, 2, 10, QChar('0')) + ":" +
            QString("%1").arg(sec, 2, 10, QChar('0')) + "." + QString::number(msec);


    zuluDisplay->display( zuluTime );
}

void MainWindow::SLOT_setRxGain(int g) {
        if( radio == NULL )
            return ;
        radio->setRTLGain( g );
}

void MainWindow::SLOT_setDetectionThreshold(int level) {
      Controller& ctrl = Controller::getInstance() ;
      ctrl.setDetectionThreshold(level);
}
