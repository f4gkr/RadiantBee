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
#ifndef RADARVIEW_H
#define RADARVIEW_H

#include <QWidget>
#include <QTimer>
#include <qpen.h>
#include <qlocale.h>
#include "qwt/qwt_raster_data.h"
#include "qwt/qwt_scale_div.h"
#include "qwt/qwt_round_scale_draw.h"
#include "qwt/qwt_picker_machine.h"

#include "qwtPolar/qwt_polar_plot.h"
#include "qwtPolar/qwt_polar_panner.h"
#include "qwtPolar/qwt_polar_magnifier.h"
#include "qwtPolar/qwt_polar_grid.h"
#include "qwtPolar/qwt_polar_spectrogram.h"
#include "qwtPolar/qwt_polar_renderer.h"
#include "qwtPolar/qwt_polar_picker.h"
#include "qwtPolar/qwt_polar_marker.h"
#include <QSemaphore>

class QwtPolarGrid;
class QwtPolarSpectrogram;

#define MODE_SPECTRO_POLAR (0)
#define MODE_SCAN_FOR_MAX (1)

class RadarView : public QwtPolarPlot
{
    Q_OBJECT
public:
    explicit RadarView(QWidget *parent = 0);
    void start();
    QwtPolarSpectrogram *spectrogram();
    void setSpectrum(double *power_dB, int length, double bw );

signals:

public slots:
     void SLOT_AntennaOrientationChanged( float newAzimuth, float newElevation );

private slots:
     void SLOT_update() ;

private:
     QSemaphore *sem;

     QwtPolarGrid *d_grid;
     QwtPolarSpectrogram *d_spectrogram;
     QwtPolarMarker *marker ;
     QTimer *timer ;

     int tabLength ;
     int current_azimuth ;
     int m_mode ;
     int m_azimuth_divs ;
     int m_radius_divs ;
     double *datas ; // [[m_radius_divs for azimuth 0],[azimuth 1],...[azimuth m_azimuth_divs]]
};

#endif // RADARVIEW_H
