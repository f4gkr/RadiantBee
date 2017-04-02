# RadiantBee
(c) F4GKR and F5OEO

RadiantBee is a tentative to use a UAV for antenna calibration. The UAV transmits a specific signal (chirp) and telemetry to say where it is. The base station receives the signal, locks on the chirp by using a matched filter to estimate channel and decodes lat/lon/altitude and UAV attitude to estimate the transmitting conditions

Current version transmits a 100 KHz wide FMCM chirp for 250ms followed by a FSK message and a silence (250 ms).
The base station estimate power of received signal during the chirp (correlator) and does the same during the 250ms of silence. The difference between chirp and silence gives the SNR.

Transmitter (the flying BEE) is currently working on RPI + hackRF + Pololu AltIMU + GPS.
Receiver uses GPS + RTLSDR
