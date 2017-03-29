#ifndef OVERLAPSAVE_H
#define OVERLAPSAVE_H

#include <QObject>
#include <fftw3.h>
#include "common/datatypes.h"

#define INPUT_TOO_LONG (-1)
#define NEED_MORE_DATA (-2)
#define GET_DATA_OUT (1)
#define NO_OUT_DATA (-1)
#define NOT_ENOUGH_DATA (-1)



class OverlapSave : public QObject
{
    Q_OBJECT
public:
    explicit OverlapSave(int inSampleRate, int outSampleRate, QObject *parent = 0);
    ~OverlapSave();

    void reset();

    int getOLASOutSampleRate() ;
    int getOLASInSampleRate() { return( m_inSampleRate ) ; }

    void configure(int maxInSize , int use_fft_size );
    int put(TYPECPX *insamples, int length );
    int get(TYPECPX *data , int max_read=-1);

    void setCenterOfWindow( float freq );
    float getCenterOfWindow();

    bool shiftInTimeDomain( int samples );
    bool shiftOutTimeDomain( int samples );
    bool shiftOutPhase( float angleRadians );
    void razOutPhase();
    void shiftOutputCenterFrequency( float offset_hz );

    static int FILTER_KERNEL_SIZE ;

signals:

public slots:
private:
    int m_inSampleRate ;
    int m_outSampleRate;
    int DecimFactor ;
    int Nrequired ;
    double *H ;
    int NTaps ;
    int fft_size ;
    float m_center_freq ;
    double mix_offset ;
    double mix_phase ;
    bool apply_postmixer;
    fftwf_complex * fftin,  *_H, *_H0 ; // *fftout, *fft_conv, *fft_filtered, ;
    fftwf_plan plan ;
    fftwf_plan plan_rev ;

    TYPECPX *datas_in ;
    TYPECPX *datas_out;
    long data_size ;
    long wr_pos ;
    long rd_pos ;
    long overlap ;
    long out_wpos ;
    quint64 decim_r_pos ;
    long ttl_put ;
    long ttl_read ;

    void step1();
    void step2();

    double bessi0( double x );
    double *calc_filter(double Fs, double Fa, double Fb, int M, double Att, int *Ntaps ) ;
};

#endif // OVERLAPSAVE_H
