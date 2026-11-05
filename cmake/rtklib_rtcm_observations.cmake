# rtcm-rs v0.11.0 DF397/399/400/401/404/405/406 invalid sentinels are
# distinct from zero. Apply only to MSM4-7 storage; never infer missingness
# from a valid decoded zero. See test_rtcm_reference.py and R2 reference hashes.
gnss_receiver_replace(2 "for (j=0;j<h.nsat;j++) r[j]=0.0;"
  "for (j=0;j<h.nsat;j++) r[j]=NAN;")
gnss_receiver_replace(2 "r[j]=rr[j]=0.0; ex[j]=15;"
  "r[j]=rr[j]=NAN; ex[j]=15;")
gnss_receiver_replace(4 "if (r[j]!=0.0) r[j]+=rng_m*P2_10*RANGE_MS;"
  "if (isfinite(r[j])) r[j]+=rng_m*P2_10*RANGE_MS;")
gnss_receiver_replace(1 "                /* pseudorange (m) */" [=[                /* A cell may reuse a satellite from an earlier fragment. */
                rtcm->obs.data[index].P[idx[k]]=NAN;
                rtcm->obs.data[index].L[idx[k]]=NAN;
                rtcm->obs.data[index].D[idx[k]]=NAN;
                /* pseudorange (m) */]=])
gnss_receiver_replace(1 "if (r[i]!=0.0&&pr[j]>-1E12)"
  "if (isfinite(r[i])&&pr[j]>-1E12)")
gnss_receiver_replace(1 "if (r[i]!=0.0&&cp[j]>-1E12)"
  "if (isfinite(r[i])&&cp[j]>-1E12&&freq>0.0)")
gnss_receiver_replace(1 "if (rr&&rrf&&rrf[j]>-1E12)"
  "if (rr&&rrf&&isfinite(rr[i])&&rrf[j]>-1E12&&freq>0.0)")
gnss_receiver_replace(1 "rtcm->obs.data[index].SNR [idx[k]]=cnr[j];"
  "rtcm->obs.data[index].SNR [idx[k]]=cnr[j]>0.0 ? cnr[j] : NAN;")

# R2 legacy observations: DF011/041 all-ones range is invalid; phase
# differences and L2 range differences have independent signed sentinels.
# Legacy messages contain no Doppler. Retire the current signal's previous
# values when another fragment fills the same cell, preserving valid zeros.
gnss_receiver_replace(2 "        pr1=pr1*0.02+amb*PRUNIT_GPS;"
  "        pr1=pr1==16777215.0 ? NAN : pr1*0.02+amb*PRUNIT_GPS;")
gnss_receiver_replace(2 "        pr1=pr1*0.02+amb*PRUNIT_GLO;"
  "        pr1=pr1==33554431.0 ? NAN : pr1*0.02+amb*PRUNIT_GLO;")
gnss_receiver_replace(4 "        rtcm->obs.data[index].P[0]=pr1;" [=[        rtcm->obs.data[index].P[0]=pr1;
        rtcm->obs.data[index].L[0]=rtcm->obs.data[index].D[0]=NAN;]=])
gnss_receiver_replace(4 "if (ppr1!=(int)0xFFF80000)"
  "if (isfinite(pr1)&&ppr1!=(int)0xFFF80000)")
gnss_receiver_replace(2 "        if (pr21!=(int)0xFFFFE000) {" [=[        rtcm->obs.data[index].P[1]=rtcm->obs.data[index].L[1]=rtcm->obs.data[index].D[1]=NAN;
        if (isfinite(pr1)&&pr21!=(int)0xFFFFE000) {]=])
gnss_receiver_replace(2 "if (ppr2!=(int)0xFFF80000)"
  "if (isfinite(pr1)&&ppr2!=(int)0xFFF80000)")
gnss_receiver_replace(2 "freq1=code2freq(SYS_GLO,CODE_L1C,fcn-7);"
  "freq1=fcn<=13 ? code2freq(SYS_GLO,CODE_L1C,fcn-7) : NAN;")
gnss_receiver_replace(1 "freq2=code2freq(SYS_GLO,CODE_L2C,fcn-7);"
  "freq2=fcn<=13 ? code2freq(SYS_GLO,CODE_L2C,fcn-7) : NAN;")
gnss_receiver_replace(4 "rtcm->obs.data[index].SNR[0]=snratio(cnr1*0.25);"
  "rtcm->obs.data[index].SNR[0]=cnr1>0 ? snratio(cnr1*0.25) : NAN;")
gnss_receiver_replace(2 "rtcm->obs.data[index].SNR[1]=snratio(cnr2*0.25);"
  "rtcm->obs.data[index].SNR[1]=cnr2>0 ? snratio(cnr2*0.25) : NAN;")
