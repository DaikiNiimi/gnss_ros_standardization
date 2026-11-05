# Bounds and epoch-state fixes for the pinned backend. Applied after receiver fixes.
if(receiver STREQUAL "ublox")
  gnss_receiver_replace(1 [==[    trace(4,"decode_rxmraw: len=%d\n",raw->len);]==] [==[    trace(4,"decode_rxmraw: len=%d\n",raw->len);
    if (raw->len < 16) return -1;]==])
  gnss_receiver_replace(1 [==[if (raw->len<12+24*nsat)]==] [==[if (raw->len<16+24*nsat)]==])
  gnss_receiver_replace(1 [==[    tow =U4(p  );]==] [==[    tow =U4(p  );
    if (tow >= 604800000.0) return -1;]==])
  gnss_receiver_replace(1 [==[    trace(4,"decode_trkmeas: len=%d\n",raw->len);]==] [==[    trace(4,"decode_trkmeas: len=%d\n",raw->len);
    if (raw->len < 112) return -1;]==])
  gnss_receiver_replace(2 [==[for (i=0,p=raw->buff+110;i<nch;i++,p+=56)]==] [==[for (i=0,p=raw->buff+110;i<nch && n<MAXOBS;i++,p+=56)]==])
  gnss_receiver_replace(1 [==[    trace(4,"decode_trkd5: len=%d\n",raw->len);]==] [==[    trace(4,"decode_trkd5: len=%d\n",raw->len);
    if (raw->len < 9) return -1;]==])
  gnss_receiver_replace(1 [==[        default: off=78; len=56; break;
    }]==] [==[        default: off=78; len=56; break;
    }
    if (raw->len < off+2 || (raw->len-off-2)%len) return -1;]==])
  gnss_receiver_replace(2 [==[for (i=0,p=raw->buff+off;p-raw->buff<raw->len-2;i++,p+=len)]==] [==[for (i=0,p=raw->buff+off;p-raw->buff+len<=raw->len-2 && n<MAXOBS;i++,p+=len)]==])
  gnss_receiver_replace(1 [==[if (ubx_sys(U1(p+56))==SYS_GLO) t-=10800.0+utc_gpst;]==] [==[if (type==6 && ubx_sys(U1(p+56))==SYS_GLO) t-=10800.0+utc_gpst;]==])
elseif(receiver STREQUAL "septentrio")
  gnss_receiver_replace(1 [==[  gtime_t current_time;]==] [==[  gtime_t current_time;
  gtime_t meas3_reference_time;
  int meas3_reference_valid;
  int meas3_active;]==])
  gnss_receiver_replace(1 [==[    int n = 0, idx = 0, ret = 0;]==] [==[    int n = 0, idx = 0, ret = 0;
    if (raw->len < 20 || (U2(raw->buff+4)>>13) != 0 || U1(p+11)>31 || (U1(p+6)&0x80)) return -1;]==])
  gnss_receiver_replace(1 [==[    uint8_t constellations = U2(p+8);]==] [==[    uint16_t constellations = U2(p+8);
    if (constellations & ~0x7f) goto invalid_meas3;]==])
  gnss_receiver_replace(1 [==[        sbf->meas3_refEpoch.TOW = TOW;]==] [==[        sbf->meas3_refEpoch.TOW = TOW;
        sbf->meas3_reference_time = raw->time;
        sbf->meas3_reference_valid = 1;]==])
  gnss_receiver_replace(1 [==[    p += 12; // jump to start of data]==] [==[    p += 12; // jump to start of data
#define MEAS3_NEED(offset, count) do { \
    if ((offset)<0 || (size_t)(offset)+(size_t)(count) > (size_t)(raw->len-20)) goto invalid_meas3; \
} while (0)
    sbf->meas3_active = 1;]==])
  gnss_receiver_replace(1 [==[        (sbf->meas3_refEpoch.TOW != (uint32_t)(TOW / refEpochInterval)*refEpochInterval)) { // or reference epoch is available
        raw->obuf.n = 0;
        return ret;
    }]==] [==[        (!sbf->meas3_reference_valid ||
         fabs(timediff(raw->time,sbf->meas3_reference_time)-(TOW%refEpochInterval)*.001)>1e-6)) {
        goto invalid_meas3;
    }]==])
  gnss_receiver_replace(1 [==[            uint8_t gloFncs[8] = {0, 0, 0, 0, 0, 0, 0, 0};]==] [==[            uint8_t gloFncs[32] = {0};]==])
  gnss_receiver_replace(1 [==[            // read satellite data
            if (U1(p+idx) == 0)
                p_navsys = sbf->meas3_refEpoch.constellationHeader[navsys];]==] [==[            // Bound both wire and reused reference headers before every read.
            MEAS3_NEED(idx,1);
            if (U1(p+idx) == 0) {
                if (TOW%refEpochInterval==0 || sbf->meas3_refEpoch.constellationHeader[navsys][0]==0)
                    goto invalid_meas3;
                p_navsys = sbf->meas3_refEpoch.constellationHeader[navsys];
            }
            size_t header_size = p_navsys == p+idx ? (size_t)(raw->len-20-idx) : 32;]==])
  gnss_receiver_replace(1 [==[            // read satellite mask]==] [==[            if ((size_t)idx_navsys+nB > header_size) goto invalid_meas3;
            // read satellite mask]==])
  gnss_receiver_replace(1 [==[            // Read GLONASS fcn list]==] [==[            if ((size_t)idx_navsys+(navsys==1 ? (nSats+1)/2 : navsys==3 ? 2 : 0)+signalExcludedPresent > header_size)
                goto invalid_meas3;
            // Read GLONASS fcn list]==])
  gnss_receiver_replace(1 [==[                if (idx_navsys > 23)
                    trace(2, "sbf meas3ranges idx_navsys too large\n");]==] [==[                if (idx_navsys > sizeof(sbf->meas3_refEpoch.constellationHeader[navsys]))
                    goto invalid_meas3;]==])
  gnss_receiver_replace(1 [==[                int masterFreqIndex;]==] [==[                int masterFreqIndex = -1;]==])
  gnss_receiver_replace(1 [==[                double prbase = (bdsLongRange & (1 << satCnt)) != 0 ? 34e6 : PRBase[navsys];]==] [==[                double prbase = (satCnt<16 && (bdsLongRange & (1u << satCnt))) ? 34e6 : PRBase[navsys];
                MEAS3_NEED(idx,1);
                if (n>=MAXOBS) goto invalid_meas3;]==])
  gnss_receiver_replace(1 [==[                    // Master short
                    uint32_t BF1]==] [==[                    // Master short
                    MEAS3_NEED(idx,prrAvailable ? 10 : 8);
                    uint32_t BF1]==])
  gnss_receiver_replace(1 [==[                    // Master long
                    uint32_t BF1]==] [==[                    // Master long
                    MEAS3_NEED(idx,10);
                    uint32_t BF1]==])
  gnss_receiver_replace(1 [==[                    uint8_t  BF3    = U1(p+idx+10);]==] [==[                    unsigned continuation = (BF2>>15)&1;
                    MEAS3_NEED(idx,10+continuation+(prrAvailable ? 2 : 0));
                    uint8_t BF3 = continuation ? U1(p+idx+10) : 0;]==])
  gnss_receiver_replace(1 [==[                    slaveSignalMask = signalMask ^ 1UL << (masterSignalIndex);]==] [==[                    if (masterSignalIndex>=MEAS3_SIG_MAX) goto invalid_meas3;
                    slaveSignalMask = signalMask ^ (1u << masterSignalIndex);]==])
  gnss_receiver_replace(1 [==[                    // Master long delta
                    masterSignalIndex]==] [==[                    // Master long delta
                    MEAS3_NEED(idx,5);
                    if (TOW%refEpochInterval==0 || !sbf->meas3_refEpoch.constellationHeader[navsys][0]) goto invalid_meas3;
                    masterSignalIndex]==])
  gnss_receiver_replace(1 [==[                    // Master short delta
                    uint32_t]==] [==[                    // Master short delta
                    MEAS3_NEED(idx,4);
                    if (TOW%refEpochInterval==0 || !sbf->meas3_refEpoch.constellationHeader[navsys][0]) goto invalid_meas3;
                    uint32_t]==])
  gnss_receiver_replace(1 [==[if (TOW % refEpochInterval == 0  && satNo > 0)]==] [==[if (TOW % refEpochInterval == 0 && satNo > 0 && masterFreqIndex>=0 && masterFreqIndex<NFREQ+NEXOBS)]==])
  gnss_receiver_replace(1 [==[if (satNo > 0  && raw->lockt[satNo-1][masterFreqIndex]]==] [==[if (satNo > 0 && masterFreqIndex>=0 && masterFreqIndex<NFREQ+NEXOBS && raw->lockt[satNo-1][masterFreqIndex]]==])
  gnss_receiver_replace(1 [==[                        int slaveFreqIndex;]==] [==[                        int slaveFreqIndex = -1;]==])
  gnss_receiver_replace(1 [==[                        int blockTypeSlave = U1(p+idx);]==] [==[                        MEAS3_NEED(idx,1);
                        int blockTypeSlave = U1(p+idx);]==])
  gnss_receiver_replace(1 [==[                            /* Slave Short */]==] [==[                            /* Slave Short */
                            MEAS3_NEED(idx,5);]==])
  gnss_receiver_replace(1 [==[                            /* Slave Long */]==] [==[                            /* Slave Long */
                            MEAS3_NEED(idx,7);]==])
  gnss_receiver_replace(1 [==[                            /* Slave Delta */]==] [==[                            /* Slave Delta */
                            MEAS3_NEED(idx,3);]==])
  gnss_receiver_replace(3 [==[if ((slaveFreqIndex = meas3_sig2idx(navsys, slaveSignalIndex, raw->opt, &codeSlave, sigTable)) >= 0 && satNo > 0)]==] [==[if (masterFreqIndex>=0 && masterFreqIndex<NFREQ+NEXOBS && freqMaster>0 &&
                                (slaveFreqIndex = meas3_sig2idx(navsys, slaveSignalIndex, raw->opt, &codeSlave, sigTable)) >= 0 && satNo > 0)]==])
  gnss_receiver_replace(1 [==[                                raw->obuf.data[n].L[slaveFreqIndex] = (slaveReference->L[slaveRefFreqIdx]]==] [==[                                if (TOW%refEpochInterval==0 || masterRefFreqIdx<0 || slaveRefFreqIdx<0 ||
                                    masterRefFreqIdx>=NFREQ+NEXOBS || slaveRefFreqIdx>=NFREQ+NEXOBS) goto invalid_meas3;
                                raw->obuf.data[n].L[slaveFreqIndex] = (slaveReference->L[slaveRefFreqIdx]]==])
  gnss_receiver_replace(1 [==[sbf->meas3_refEpoch.lockt[navsys][svid][slaveCnt+1];]==] [==[sbf->meas3_refEpoch.lockt[navsys][svid][slaveRefFreqIdx];]==])
  gnss_receiver_replace(1 [==[if (TOW % refEpochInterval == 0 && satNo > 0)]==] [==[if (TOW % refEpochInterval == 0 && satNo > 0 && slaveFreqIndex>=0 && slaveFreqIndex<NFREQ+NEXOBS)]==])
  gnss_receiver_replace(1 [==[if (raw->lockt[satNo-1][slaveFreqIndex] >]==] [==[if (satNo>0 && slaveFreqIndex>=0 && slaveFreqIndex<NFREQ+NEXOBS && raw->lockt[satNo-1][slaveFreqIndex] >]==])
  gnss_receiver_replace(1 [==[                n++;
                satCnt++;]==] [==[                for (int k=0;k<NFREQ+NEXOBS;k++) if (raw->obuf.data[n].code[k]) {
                    raw->obuf.data[n].D[k]=NAN;
                    if (raw->obuf.data[n].L[k]==0) raw->obuf.data[n].L[k]=NAN;
                }
                if (navsys!=1) raw->obuf.data[n].freq=255;
                n++;
                satCnt++;]==])
  gnss_receiver_replace(1 [==[    raw->obuf.n = n;

    return ret;
}

int32_t meas3_DopplerPrRate]==] [==[    raw->obuf.n = n;
#undef MEAS3_NEED
    return ret;
invalid_meas3:
    trace(2,"SBF Meas3 rejected: bounds, signal mask or missing reference\n");
    raw->obuf.n=0;
    sbf->meas3_reference_valid=0;
    sbf->meas3_active=0;
    return ret ? ret : -1;
}

int32_t meas3_DopplerPrRate]==])
  gnss_receiver_replace(1 [==[    if (raw->len < 52) {
        trace(2, "sbf navicraw]==] [==[    if (raw->len < 60) {
        trace(2, "sbf navicraw]==])
  gnss_receiver_replace(1 [==[memset(sbf->meas3_freqAssignment, -1,]==] [==[memset(sbf->meas3_freqAssignment, -2,]==])
  gnss_receiver_replace(1 [==[                /* keep reference measurement to decode the delta measurements */]==] [==[                sbf->meas3_freqAssignment[navsys][svid][0]=masterFreqIndex;
                /* keep reference measurement to decode the delta measurements */]==])
  gnss_receiver_replace(1 [==[                        slaveCnt++;
                        /* delete this bit]==] [==[                        sbf->meas3_freqAssignment[navsys][svid][slaveCnt+1]=slaveFreqIndex;
                        slaveCnt++;
                        /* delete this bit]==])
  gnss_receiver_replace(1 [==[    n1   = U1(p);]==] [==[    sbf->meas3_active=0;
    n1   = U1(p);]==])
  gnss_receiver_replace(1 [==[  int meas3_active;]==] [==[  int meas3_active;
  unsigned char meas3_reference_sat[MEAS3_SYS_MAX][MEAS3_SAT_MAX];]==])
  gnss_receiver_replace(1 [==[        sbf->meas3_reference_valid = 1;]==] [==[        sbf->meas3_reference_valid = 1;
        memset(sbf->meas3_reference_sat,0,sizeof(sbf->meas3_reference_sat));]==])
  gnss_receiver_replace(2 [==[if (TOW%refEpochInterval==0 || !sbf->meas3_refEpoch.constellationHeader[navsys][0]) goto invalid_meas3;]==] [==[if (TOW%refEpochInterval==0 || !sbf->meas3_reference_sat[navsys][svid]) goto invalid_meas3;]==])
  gnss_receiver_replace(1 [==[                sbf->meas3_freqAssignment[navsys][svid][0]=masterFreqIndex;]==] [==[                if (slaveSignalMask & ~0xffffu) goto invalid_meas3;
                if (masterFreqIndex>=0 && masterFreqIndex<NFREQ+NEXOBS &&
                    (freqMaster<=0 || raw->obuf.data[n].L[masterFreqIndex]==0)) raw->obuf.data[n].L[masterFreqIndex]=NAN;
                sbf->meas3_freqAssignment[navsys][svid][0]=masterFreqIndex;]==])
  gnss_receiver_replace(1 [==[                    sbf->meas3_refEpoch.signalIdx[navsys][svid][0] = (uint8_t)masterSignalIndex;]==] [==[                    sbf->meas3_reference_sat[navsys][svid]=1;
                    sbf->meas3_refEpoch.signalIdx[navsys][svid][0] = (uint8_t)masterSignalIndex;]==])
  gnss_receiver_replace(1 [==[                        /* keep reference measurement to decode delta measurements */]==] [==[                        if (slaveFreqIndex>=0 && slaveFreqIndex<NFREQ+NEXOBS && raw->obuf.data[n].L[slaveFreqIndex]==0)
                            raw->obuf.data[n].L[slaveFreqIndex]=NAN;
                        /* keep reference measurement to decode delta measurements */]==])
  gnss_receiver_replace(1 [==[    if (timediff(raw->time, sbf->current_time) != 0) {
        sbf->current_time = raw->time;
        ret = flushobuf(raw);
    }

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF Measurement Data Extra");
    }]==] [==[    if (sbf->meas3_active || raw->obuf.n==0 ||
        timediff(raw->time,raw->obuf.data[0].time)!=0) return 0;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF Measurement Data Extra");
    }]==])
  gnss_receiver_replace(1 [==[    if (raw->len < 20 || (U2(raw->buff+4)>>13) != 0 || U1(p+11)>31 || (U1(p+6)&0x80)) return -1;]==] [==[    if (raw->len < 20 || (U2(raw->buff+4)>>13) != 0 || U1(p+11)>31 || (U1(p+6)&0x80)) return -1;
    double saved_lockt[MAXSAT][NFREQ+NEXOBS];
    memcpy(saved_lockt,raw->lockt,sizeof(saved_lockt));]==])
  gnss_receiver_replace(1 [==[invalid_meas3:
    trace]==] [==[invalid_meas3:
    memcpy(raw->lockt,saved_lockt,sizeof(saved_lockt));
    trace]==])
  string(FIND "${_receiver_source}" "int32_t meas3_DopplerPrRate" extra_start)
  string(SUBSTRING "${_receiver_source}" ${extra_start} -1 extra_tail)
  string(FIND "${extra_tail}" "/* Navigation Page Blocks */" extra_length)
  string(SUBSTRING "${_receiver_source}" ${extra_start} ${extra_length} extra_old)
  string(SHA256 extra_hash "${extra_old}")
  if(NOT extra_hash STREQUAL "1dccb56abb0bce22ef22f13a7d145abf5d06ce8f3045ec793051ca0872b7e0bb")
    message(FATAL_ERROR "Meas3 extension source changed; review the pinned backend replacement")
  endif()
  set(extra_file "${CMAKE_CURRENT_LIST_DIR}/rtklib_meas3_extensions.c")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${extra_file}")
  file(READ "${extra_file}" extra_new)
  gnss_receiver_replace(1 "${extra_old}" "${extra_new}")
elseif(receiver STREQUAL "rtcm3")
  gnss_receiver_replace(4 [==[        rtcm->obsflag=!sync;  /* header ok, so return sync bit */
        return sync?0:1;]==] [==[        rtcm->obs.n=0;
        rtcm->obsflag=1;
        return -1; /* A truncated final fragment never completes an epoch. */]==])
  gnss_receiver_replace(1 [==[    if (decode_msm_head(rtcm,sys,&sync,&iod,&h,&i)<0) return -1;
    rtcm->obsflag=!sync;
    return sync?0:1;]==] [==[    (void)decode_msm_head(rtcm,sys,&sync,&iod,&h,&i);
    rtcm->obs.n=0;
    rtcm->obsflag=1;
    return -1; /* MSM1/2/3 are not decoded; do not complete prior fragments. */]==])
  gnss_receiver_replace(1 [==[            tod   =getbitu(rtcm->buff,i,27)*0.001; i+=27;
            adjday_glot]==] [==[            tod   =getbitu(rtcm->buff,i,27)*0.001; i+=27;
            if (dow>6 || tod>=86400.0) return -1;
            adjday_glot]==])
  gnss_receiver_replace(2 [==[            tow   =getbitu(rtcm->buff,i,30)*0.001; i+=30;]==] [==[            tow   =getbitu(rtcm->buff,i,30)*0.001; i+=30;
            if (tow>=604800.0) return -1;]==])
endif()
