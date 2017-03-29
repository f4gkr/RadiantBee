#ifndef UAVPROCESSOR_H
#define UAVPROCESSOR_H

#include <QObject>
#include <fftw3.h>
#include "common/samplefifo.h"
#include "common/datatypes.h"
#include "common/sdr_types.h"

class UAVProcessor : public QObject
{
    Q_OBJECT
public:

    explicit UAVProcessor(QObject *parent = 0);
    void newData(TYPECPX* IQsamples, int L , int sampleRate );
    void raz();

signals:

public slots:
private:
    TYPECPX *samples ;
    int m_bandwidth ;
    int chirp_length ;
    int fft_length ;
    int wr_pos ;
    int msg_wrpos ;
    int msg_length ;
    float ratio_min ;
    float ratio_max ;
    float threshold ;
    bool synched ;
    fftwf_complex * fftin, *fftout,*fftchirp;
    fftwf_plan fft_plan,fft_plan_inv ;

    void generate(TYPEREAL bandwidth , TYPEREAL sampleRate, long length);
    int calc(float *pvmax);
    char *demod(TYPECPX* psamples , int L);
    int demodByte(TYPECPX* psamples, int *pstart, int L );
};

#endif // UAVPROCESSOR_H
