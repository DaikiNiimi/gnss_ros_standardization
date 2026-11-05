/* Package-owned Meas3 optional blocks. Preflight the complete payload before
 * modifying observations; a late/wrong-epoch block cannot flush another epoch. */
static int gnss_meas3_extra(raw_t *raw, int doppler)
{
    sbf_t *sbf=(sbf_t *)raw->rcv_data;
    size_t offset=16, nibble=0;
    int n,k,sys,prn,navsys,ant=0;
    float values[MAXOBS][NFREQ+NEXOBS];
    unsigned char present[MAXOBS][NFREQ+NEXOBS]={{0}};
    if (raw->len<16 || (U2(raw->buff+4)>>13)!=0) return -1;
    if (strstr(raw->opt,"-NO_MEAS3")) return 0;
    if (strstr(raw->opt,"-AUX1")) ant=1;
    else if (strstr(raw->opt,"-AUX2")) ant=2;
    if ((U2(raw->buff+14)&7)!=ant) return 0;
    if (!sbf->meas3_active || raw->obuf.n==0 ||
        timediff(raw->time,raw->obuf.data[0].time)!=0) return 0;
    for (n=0;n<raw->obuf.n;n++) {
        obsd_t *obs=raw->obuf.data+n;
        sys=satsys(obs->sat,&prn);
        for (navsys=0;navsys<MEAS3_SYS_MAX;navsys++) if (Meas3_NavSys[navsys]==sys) break;
        if (navsys==MEAS3_SYS_MAX) return -1;
        int svid=prn-Meas3_SVIDBase[navsys];
        if (svid<0 || svid>=MEAS3_SAT_MAX) return -1;
        double master_rate=NAN;
        for (k=0;k<MEAS3_SIG_MAX;k++) {
            int index=sbf->meas3_freqAssignment[navsys][svid][k];
            if (index==-2) break;
            if (index>=NFREQ+NEXOBS || index < -1) return -1;
            if (doppler) {
                if (offset>=(size_t)raw->len) return -1;
                unsigned first=raw->buff[offset], bytes=(first&2)==0 ? 1 : (first&6)==2 ? 2 : (first&14)==6 ? 3 : 4;
                if (offset+bytes>(size_t)raw->len) return -1;
                uint32_t encoded=0;
                for (unsigned b=0;b<bytes;b++) encoded|=(uint32_t)raw->buff[offset+b]<<(8*b);
                offset+=bytes;
                double rate=(double)(encoded>>(bytes==1 ? 2 : bytes==2 ? 3 : 4));
                if (encoded&1) rate=-rate;
                if (rate==-268435455) rate=NAN;
                if (k==0) master_rate=rate+(double)sbf->meas3_refEpoch.prRate[navsys][svid]*64.;
                if (index>=0) {
                    double freq=code2freq(sys,obs->code[index],obs->freq==255 ? -8 : obs->freq-7);
                    values[n][index]=freq>0 ? (float)(-(k==0 ? master_rate : master_rate+rate)*.001*freq/CLIGHT) : NAN;
                    present[n][index]=1;
                }
            } else {
                if (16+nibble/2>=(size_t)raw->len) return -1;
                unsigned fraction=(raw->buff[16+nibble/2]>>((nibble%2)*4))&15;
                ++nibble;
                if (index>=0) {
                    // Recover the integer C/N0 from Ranges; repeated extras are idempotent.
                    values[n][index]=floor(obs->SNR[index]+.5)+fraction*.0625-.5;
                    present[n][index]=1;
                }
            }
        }
    }
    for (n=0;n<raw->obuf.n;n++) for (k=0;k<NFREQ+NEXOBS;k++) if (present[n][k]) {
        if (doppler) raw->obuf.data[n].D[k]=values[n][k];
        else raw->obuf.data[n].SNR[k]=values[n][k];
    }
    return 0;
}
int decode_meas3Doppler(raw_t *raw) { return gnss_meas3_extra(raw,1); }
int decode_meas3CN(raw_t *raw) { return gnss_meas3_extra(raw,0); }

