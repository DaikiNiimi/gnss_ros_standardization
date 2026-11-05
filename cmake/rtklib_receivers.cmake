# Narrow fixes to the pinned receiver decoders. Keep the submodule unchanged and
# fail configuration if an upstream edit changes any expected patch context.
function(gnss_receiver_replace expected before after)
  set(rest "${_receiver_source}")
  set(count 0)
  string(LENGTH "${before}" length)
  while(TRUE)
    string(FIND "${rest}" "${before}" offset)
    if(offset LESS 0)
      break()
    endif()
    math(EXPR count "${count} + 1")
    math(EXPR next "${offset} + ${length}")
    string(SUBSTRING "${rest}" ${next} -1 rest)
  endwhile()
  if(NOT count EQUAL expected)
    message(FATAL_ERROR "RTKLIB ${receiver} patch expected ${expected} matches, got ${count}; review dependency update: ${before}")
  endif()
  string(REPLACE "${before}" "${after}" patched "${_receiver_source}")
  set(_receiver_source "${patched}" PARENT_SCOPE)
endfunction()

foreach(receiver ublox novatel septentrio)
  set(input "${RTKLIB_ROOT}/src/rcv/${receiver}.c")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${input}")
  file(READ "${input}" _receiver_source)
  if(receiver STREQUAL "ublox")
    # U1 RXM-SFRBX / U2 32.18.7: validate the envelope before reading any
    # satellite fields or modifying assembly state. Version1 byte2 is Reserved.
    gnss_receiver_replace(1 [=[    trace(4,"decode_rxmsfrbx: len=%d\n",raw->len);]=]
      [=[    trace(4,"decode_rxmsfrbx: len=%d\n",raw->len);
    if (raw->len < 16) return GNSS_RX_FAIL(GNSS_RX_LENGTH);
    if (U1(p+6)!=1 && U1(p+6)!=2) return GNSS_RX_FAIL(GNSS_RX_VERSION);
    if (U1(p+4)>16 || raw->len != 16+4*U1(p+4)) return GNSS_RX_FAIL(GNSS_RX_LENGTH);
    if (U1(p+6)==1) p[2]=0; /* no sigId in protocol17 */]=])
    # U2 NAV-SOL pp386-387, NAV-TIMEGPS pp397-398, TIM-TM2 pp429-430.
    # Never read a short time packet from stale bytes left in raw->buff.
    gnss_receiver_replace(1 "    trace(4,\"decode_navsol: len=%d\\n\",raw->len);"
      "    trace(4,\"decode_navsol: len=%d\\n\",raw->len);\n    if (raw->len < 60) return GNSS_RX_FAIL(GNSS_RX_LENGTH);")
    gnss_receiver_replace(1 "    trace(4,\"decode_navtime: len=%d\\n\",raw->len);"
      "    trace(4,\"decode_navtime: len=%d\\n\",raw->len);\n    if (raw->len < 24) return GNSS_RX_FAIL(GNSS_RX_LENGTH);")
    gnss_receiver_replace(1 "    trace(4, \"decode_timtm2: len=%d\\n\", raw->len);"
      "    trace(4, \"decode_timtm2: len=%d\\n\", raw->len);\n    if (raw->len < 36) return GNSS_RX_FAIL(GNSS_RX_LENGTH);")
    # Both time messages carry a signed week and +/-500000 fractional ns.
    # Apply ranges only when the receiver declares week/TOW valid; otherwise
    # preserve its existing 'no time update' behavior. Fractional week carry
    # remains legal at the rounded millisecond boundary.
    gnss_receiver_replace(2 "        raw->time=gpst2time(week,itow*1E-3+ftow*1E-9);"
      [=[        if (week>=32768) week-=65536;
        if (week<=0 || U4(p)>=604800000u || ftow < -500000 || ftow > 500000)
            return GNSS_RX_FAIL(GNSS_RX_TIME);
        raw->time=gpst2time(week,itow*1E-3+ftow*1E-9);]=])
    # U4 section36.4 pp185-186: RXM-RAW quality gates components separately.
    # Keep the explicit wire LLI; "likely" loss for mesQI<6 is not another bit.
    gnss_receiver_replace(1 "        raw->obs.data[n].L[0]  =R8(p   )-toff*FREQL1;"
      "        raw->obs.data[n].L[0]  =I1(p+21)>=5 ? R8(p)-toff*FREQL1 : NAN;")
    gnss_receiver_replace(1 "        raw->obs.data[n].P[0]  =R8(p+ 8)-toff*CLIGHT;"
      "        raw->obs.data[n].P[0]  =I1(p+21)>=4 ? R8(p+8)-toff*CLIGHT : NAN;")
    gnss_receiver_replace(1 "        raw->obs.data[n].D[0]  =R4(p+16);"
      "        raw->obs.data[n].D[0]  =I1(p+21)>=4 ? R4(p+16) : NAN;")
    # The week is I2, not U2. Reject unresolved/nonpositive weeks before
    # modifying raw time or lock history (a ROS timestamp check is too late).
    gnss_receiver_replace(1 "    week=U2(p+4);\n    time=gpst2time(week,tow*0.001);"
      "    week=U2(p+4);\n    if (week>=32768) week-=65536; /* signed I2 without implementation-defined cast */\n    if (week<=0) return GNSS_RX_FAIL(GNSS_RX_TIME);\n    time=gpst2time(week,tow*0.001);")
    gnss_receiver_replace(1 [=[    if (week==0) {
        trace(3,"ubx rxmraw week=0 error: len=%d nsat=%d\n",raw->len,nsat);
        return 0;
    }
]=] "")
    # -MULTICODE retains the actual RAWX signal ID. Give same-band data/pilot
    # components stable distinct slots, including across reordered epochs.
    # NFREQ/NEXOBS and obsd_t ABI stay unchanged; do not pretend E6/B3 fit too.
    gnss_receiver_replace(1 "        if (code==CODE_L2S) return (nex<1)?-1:NFREQ;   /* L2CM */\n    }\n    else if (sys == SYS_GAL)" [=[        if (code==CODE_L2S) return (nex<1)?-1:NFREQ;   /* L2CM */
        if (code==CODE_L5I) return (nex<2)?-1:NFREQ+1; /* L5 data, Q in primary */
    }
    else if (sys == SYS_GAL)]=])
    gnss_receiver_replace(1 "        if (code==CODE_L7I) return (nex<2)?-1:NFREQ+1; /* E5bI */" [=[        if (code==CODE_L7I) return (nex<2)?-1:NFREQ+1; /* E5bI */
        if (code==CODE_L5I) return (nex<3)?-1:NFREQ+2; /* E5aI */]=])
    gnss_receiver_replace(1 "        if (code==CODE_L1Z) return (nex<2)?-1:NFREQ+1; /* L1S */" [=[        if (code==CODE_L1Z) return (nex<2)?-1:NFREQ+1; /* L1S */
        if (code==CODE_L5I) return (nex<3)?-1:NFREQ+2; /* L5I */
    }
    else if (sys == SYS_CMP) {
        if (code==CODE_L1D) return (nex<1)?-1:NFREQ;   /* B1C data */
        if (code==CODE_L1P) return (nex<2)?-1:NFREQ+1; /* B1C pilot */
        if (code==CODE_L5D) return (nex<3)?-1:NFREQ+2; /* B2a data, pilot in primary */]=])
    # obsd_t.freq follows FCN+7; zero is a real channel, never an unknown default.
    gnss_receiver_replace(4 "raw->obs.data[n].sat=sat;" "raw->obs.data[n].sat=sat;\n        raw->obs.data[n].freq=255;")
    gnss_receiver_replace(1 "        raw->obs.data[j].D[idx]=(float)D;" [=[        raw->obs.data[j].freq=sys==SYS_GLO&&frqid<=13 ? frqid : 255;
        raw->obs.data[j].D[idx]=(float)D;]=])
    gnss_receiver_replace(2 "        raw->obs.data[n].P[0]=tau*CLIGHT;" [=[        raw->obs.data[n].freq=sys==SYS_GLO&&frq>=-7&&frq<=6 ? frq+7 : 255;
        raw->obs.data[n].P[0]=tau*CLIGHT;]=])
    gnss_receiver_replace(1 [=[        if (sys==SYS_GLO&&!raw->nav.glo_fcn[prn-1]) {
            raw->nav.glo_fcn[prn-1]=frqid-7+8;
        }]=] [=[        if (sys==SYS_GLO) {
            raw->nav.glo_fcn[prn-1]=frqid<=13 ? frqid+1 : 0;
        }]=])
    gnss_receiver_replace(1 "    ver  =U1(p+13); /* version ([5] 5.15.3.1) */" [=[    ver  =U1(p+13); /* version ([5] 5.15.3.1) */
    if (strstr(raw->opt,"-RAWX0")) ver=0;
    if (strstr(raw->opt,"-RAWX1L")) {
        if (ver!=1) return GNSS_RX_FAIL(GNSS_RX_VERSION);
        ver=0; /* documented legacy one-signal-per-system interpretation */
    }
    if (!isfinite(tow) || tow < 0.0 || tow >= 604800.0 || ver > 1) {
        trace(2,"ubx rxmrawx unsupported version or invalid tow\n");
        return -1;
    }]=])
    gnss_receiver_replace(1 "        sigid=U1(p+22);    /* sigId ([5] 5.15.3.1) */" "        sigid=ver ? U1(p+22) : 0; /* ignore legacy Reserved for mapping and quality */")
    # Keep the backend's phase-quality policy and slip bookkeeping. Convert its
    # explicit missing components to NaN only when storing the selected signal.
    gnss_receiver_replace(1 "        raw->obs.data[j].L[idx]=L;" [=[        raw->obs.data[j].L[idx]=(!(tstat&2)||R8(p+8)==-0.5||cpstd>cpstd_valid) ? NAN : L;]=])
    gnss_receiver_replace(1 "        raw->obs.data[j].P[idx]=P;" "        raw->obs.data[j].P[idx]=(tstat&1) ? P : NAN;")
  elseif(receiver STREQUAL "novatel")
    # N1 QZSSRAWEPHEM p823 / N3 p582: payload is 12+3*30=102 bytes.
    # raw->len excludes the four-byte OEM CRC, like GPS RAWEPHEM.
    gnss_receiver_replace(1 "    if (raw->len<OEM4HLEN+106) {"
                            "    if (raw->len<OEM4HLEN+102) {")
    gnss_receiver_replace(1 "    obs->data[i].sat=sat;" "    obs->data[i].sat=sat;\n    obs->data[i].freq=255;")
    # A valid new epoch containing only filtered signals must not return old obs.
    foreach(row_bytes 24 44)
      gnss_receiver_replace(1 "    for (i=0,p+=4;i<nobs;i++,p+=${row_bytes}) {"
        "    if (raw->obs.n > 0 && fabs(timediff(raw->obs.data[0].time,raw->time)) > 1E-9) raw->obs.n=0;\n    for (i=0,p+=4;i<nobs;i++,p+=${row_bytes}) {")
    endforeach()
    # OEM6/OEM7 RANGECMP table186 carries FCN+7 in bits170..175.
    # Phase ambiguity reconstruction must use that observation, not cached nav.
    gnss_receiver_replace(1 "        if ((freq=sat2freq(sat,(uint8_t)code,&raw->nav))!=0.0) {" [=[        if (sys==SYS_GLO) {
            const int encoded=(U1(p+21)>>2)&63;
            freq=encoded<=13 ? code2freq(sys,(uint8_t)code,encoded-7) : 0.0;
            raw->nav.glo_fcn[prn-1]=encoded<=13 ? encoded+1 : 0;
        }
        else freq=sat2freq(sat,(uint8_t)code,&raw->nav);
        if (freq!=0.0) {]=])
    gnss_receiver_replace(1 "            raw->obs.data[index].L  [idx]=adr;" [=[            const int encoded=(U1(p+21)>>2)&63;
            raw->obs.data[index].freq=sys==SYS_GLO&&encoded<=13 ? encoded : 255;
            raw->obs.data[index].L  [idx]=adr;]=])
    # OEM6/OEM7 RANGE wire is FCN+7 (N1 p831, N3 RANGE), unlike the
    # backend nav.glo_fcn cache (FCN+8). Use this observation for bias, too.
    gnss_receiver_replace(1 "        gfrq =U2(p+ 2); /* GLONASS FCN+8 */"
                            "        gfrq =U2(p+ 2); /* GLONASS wire FCN+7 */")
    gnss_receiver_replace(1 "            freq=sat2freq(sat,(uint8_t)code,&raw->nav);"
                            "            freq=gfrq<=13 ? code2freq(sys,(uint8_t)code,gfrq-7) : 0.0;")
    # RANGE also carries its own FCN. Preserve it directly; publication must
    # not need an unaged navigation cache.
    gnss_receiver_replace(1 "            raw->obs.data[index].L  [idx]=-adr;" [=[            raw->obs.data[index].freq=sys==SYS_GLO&&gfrq>=0&&gfrq<=13 ? gfrq : 255;
            raw->obs.data[index].L  [idx]=-adr;]=])
    gnss_receiver_replace(1 [=[            if (!raw->nav.glo_fcn[prn-1]) {
                raw->nav.glo_fcn[prn-1]=gfrq; /* fcn+8 */
            }]=] [=[            raw->nav.glo_fcn[prn-1]=gfrq>=0&&gfrq<=13 ? gfrq+1 : 0; /* wire FCN+7 -> cache FCN+8 */]=])
    # OEM7 RANGE/RANGECMP: code lock gates pseudorange AND Doppler; phase lock
    # gates ADR only. Preserve valid zero Doppler and valid code-only tracking.
    gnss_receiver_replace(2 [=[        if (!clock) psr=0.0;     /* code unlock */
        if (!plock) adr=dop=0.0; /* phase unlock */]=] [=[        if (!clock) psr=dop=NAN; /* code unlock */
        if (!plock) adr=NAN;     /* phase unlock */]=])
    gnss_receiver_replace(1 "            adr=1e-9;" "            adr=NAN; /* unavailable carrier frequency */")
    gnss_receiver_replace(1 "    tow =U4(raw->buff+16)*0.001;" [=[    tow =U4(raw->buff+16)*0.001;
    if (tow >= 604800.0) return -1;]=])
    # Bound unsigned counts by the actual buffer before multiplication or reads.
    gnss_receiver_replace(1 [=[    nobs=U4(p);
    if (raw->len<OEM4HLEN+4+nobs*44) {]=] [=[    if (raw->len < OEM4HLEN+4) return -1;
    nobs=U4(p);
    if (U4(p) > (unsigned)(raw->len-OEM4HLEN-4)/44) {]=])
    gnss_receiver_replace(1 [=[    nobs=U4(p);
    if (raw->len<OEM4HLEN+4+nobs*24) {]=] [=[    if (raw->len < OEM4HLEN+4) return -1;
    nobs=U4(p);
    if (U4(p) > (unsigned)(raw->len-OEM4HLEN-4)/24) {]=])
  else()
    # S1 pp328-339/381/423: bounds before auxiliary decoder state mutation.
    # These blocks do not add ROS fields, but share raw_t with observation/nav.
    gnss_receiver_replace(1 "    uint32_t tow = U4(raw->buff+8);" [=[    const uint8_t *aux = raw->buff + 8;
    const int aux_len = raw->len - 8;
    int aux_count = 0, aux_stride = 0;
    switch (type) {
      case ID_EXTEVENT:
        if (aux_len < 20 || (revision == 1 && aux_len < 24))
            return GNSS_RX_FAIL(GNSS_RX_LENGTH);
        if (!isfinite(R4(aux+8)) || !isfinite(R8(aux+12)))
            return GNSS_RX_FAIL(GNSS_RX_VALUE);
        break;
      case ID_RXSETUP:
        if (aux_len < 260) return GNSS_RX_FAIL(GNSS_RX_LENGTH);
        break;
      case ID_GEOPRNMASK:
        if (aux_len < 9 || U1(aux+8) > 51 || U1(aux+8) > aux_len-9)
            return GNSS_RX_FAIL(GNSS_RX_LENGTH);
        break;
      case ID_GEOIGPMASK:
        if (aux_len < 11 || U1(aux+10) > 201 || U1(aux+10) > aux_len-11)
            return GNSS_RX_FAIL(GNSS_RX_LENGTH);
        break;
      case ID_GEOFASTCORR:
      case ID_GEOMT00:
      case ID_GEOLONGTERMCOR:
      case ID_GEOIONODELAY:
        if (aux_len < 12) return GNSS_RX_FAIL(GNSS_RX_LENGTH);
        if (type == ID_GEOLONGTERMCOR) {
            aux_count = U1(aux+7); aux_stride = U1(aux+8);
        } else if (type == ID_GEOIONODELAY) {
            aux_count = U1(aux+9); aux_stride = U1(aux+10);
        } else {
            aux_count = U1(aux+10); aux_stride = U1(aux+11);
        }
        if (aux_stride < (type == ID_GEOLONGTERMCOR ? 40 : 8) ||
            aux_count > (aux_len-12)/aux_stride)
            return GNSS_RX_FAIL(GNSS_RX_LENGTH);
        for (int ai=0; ai<aux_count; ++ai) {
            const uint8_t *row = aux+12+ai*aux_stride;
            if (type == ID_GEOLONGTERMCOR && (row[1] < 1 || row[1] > 51))
                return GNSS_RX_FAIL(GNSS_RX_VALUE);
            if (type == ID_GEOIONODELAY && (row[0] < 1 || row[0] > MAXNIGP))
                return GNSS_RX_FAIL(GNSS_RX_VALUE);
        }
        break;
    }
    uint32_t tow = U4(raw->buff+8);]=])
    # S1 subblock indexes are one-based; velocity axes are not row indexes.
    gnss_receiver_replace(1 "j = U1(p+12+i*sbLength+1);" "j = U1(p+12+i*sbLength)-1;")
    gnss_receiver_replace(1 "if (j >= raw->nav.sbssat.nsat) break;"
                            "if (j < 0 || j >= raw->nav.sbssat.nsat) continue;")
    gnss_receiver_replace(1 "j = U1(p+12+i*sbLength);" "j = U1(p+12+i*sbLength)-1;")
    gnss_receiver_replace(1 "igp[n++].lon" "igp[n].lon")
    gnss_receiver_replace(1 "lcorr.dvel[i] = R4(p+12+i*sbLength+16);" "lcorr.dvel[0] = R4(p+12+i*sbLength+16);")
    gnss_receiver_replace(1 "lcorr.dvel[i] = R4(p+12+i*sbLength+20);" "lcorr.dvel[1] = R4(p+12+i*sbLength+20);")
    gnss_receiver_replace(1 "lcorr.dvel[i] = R4(p+12+i*sbLength+24);" "lcorr.dvel[2] = R4(p+12+i*sbLength+24);")
    # S1 QZSNav pp323-324: same explicit flag and IODE consistency as GPSNav.
    gnss_receiver_replace(1 "    /* byte 21 L2 P data flag */"
                            "    eph.flag = U1(raw->buff + 21); /* L2 P data flag */")
    gnss_receiver_replace(1 "    /* byte 25: IODE from frame 3 */"
                            "    if (eph.iode != U1(raw->buff + 25)) return GNSS_RX_FAIL(GNSS_RX_NAV_INCONSISTENT); /* IODE3 */")
    gnss_receiver_replace(1 [=[    if (sat == 0) {
        trace(2, "SBF decode_qzssnav svid]=] [=[    if (sat == 0 || satsys(sat, NULL) != SYS_QZS) {
        trace(2, "SBF decode_qzssnav svid]=])
    # A reference-epoch slip is an event, not a persistent delta-epoch flag.
    # RxTools delta decoding retains validity/half-cycle state only. Preserve
    # half-cycle and any other state bits, but do not repeat the old slip.
    gnss_receiver_replace(1 "= master_reference->LLI[masterFreqIndex];"
                            "= master_reference->LLI[masterFreqIndex] & ~LLI_SLIP;")
    gnss_receiver_replace(1 "= masterReference->LLI[masterFreqIndex];"
                            "= masterReference->LLI[masterFreqIndex] & ~LLI_SLIP;")
    gnss_receiver_replace(1 "= slaveReference->LLI[slaveRefFreqIdx];"
                            "= slaveReference->LLI[slaveRefFreqIdx] & ~LLI_SLIP;")
    # mosaic-X5 FW4.15 Guide 4.2.6 BDSNav (4081), p312: PRN field uses
    # SVID numbering in 4.1.9, including 223..245 for C41..C63. The pinned
    # decoder subtracts 140 and additionally rejects everything above C32.
    gnss_receiver_replace(1 [=[    uint8_t prn = U1(raw->buff + 14) - 140;
    int sat = satno(SYS_CMP, prn);

    if (sat == 0) return -1;

    if (!((prn >= 1) && (prn <= 32))) {]=] [=[    int prn = 0;
    int sat = svid2sat(U1(raw->buff + 14));

    if (!sat || satsys(sat, &prn) != SYS_CMP) return -1;

    if (prn < MINPRNCMP || prn > MAXPRNCMP) {]=])
    gnss_receiver_replace(1 "    data->sat = (uint8_t)sat;" "    data->sat = (uint8_t)sat;\n    data->freq = 255;")
    gnss_receiver_replace(1 "        raw->obuf.data[n].freq = fcn+7;" "        /* Set FCN after init_obsd below. */")
    gnss_receiver_replace(1 "        sys = satsys(sat, NULL);" [=[        sys = satsys(sat, NULL);
        raw->obuf.data[n].freq = sys==SYS_GLO&&sig>=8&&sig<=11&&fcn>=-7&&fcn<=6 ? fcn+7 : 255;]=])
    gnss_receiver_replace(3 "raw->obuf.data[n].freq = glofnc+7;" "raw->obuf.data[n].freq = glofnc>=-7&&glofnc<=6 ? glofnc+7 : 255;")
    gnss_receiver_replace(1 "raw->obuf.data[n].freq = glofnc+8;" "raw->obuf.data[n].freq = glofnc>=-7&&glofnc<=6 ? glofnc+7 : 255;")
    # SBF GPS/BDS CNAV DELTA_N_DOT is in semicircles/s^2, like DELTA_N.
    # Normalize the backend and ROS contract to rad/s^2.
    gnss_receiver_replace(1 "    eph.ndot   = R4(p +  40);" "    eph.ndot   = R4(p +  40) * PI;")
    gnss_receiver_replace(1 "    eph.ndot   = R4(p +  32);" "    eph.ndot   = R4(p +  32) * PI;")
    # Navigation semantics/layout fixes; mosaic-X5 v4.15.0 pp.299-301,314-319.
    gnss_receiver_replace(1 [=[    if (raw->len < 172) {]=] [=[    if (sbf_id != ID_GPSCNAV || (U2(raw->buff+4)>>13) != 0 || raw->len != 172) {]=])
    gnss_receiver_replace(2 [=[    eph.iodc = tops;  // IODC/IODE is not present in CNAV, use t_op instead
    eph.iode = tops;]=] [=[    eph.iodc = 0; /* No IOD in GPS CNAV; prediction time is not represented in eph_t. */
    eph.iode = 0;
    if (tops >= 604800 || eph.toes >= 604800 || tocs >= 604800) return -1;]=])
    gnss_receiver_replace(1 [=[static int decode_cmpcnav2(raw_t *raw, uint32_t sbf_id)
{
    uint8_t *p = raw->buff+8;  /* points at TOW location */
    eph_t eph = {0};
    uint32_t tocs, tops;
    uint8_t prn, sat;

    trace(4, "SBF decode_cmpcnav2: len=%d\n", raw->len);

    if (raw->len < 164) {
        trace(2, "SBF decode_cmpcnav2 frame length error: len=%d\n", raw->len);
        return -1;
    }

    prn = U1(p+6);
    sat = satno(SYS_CMP, prn);

    if (sat == 0) return -1;

    if (!(prn>=1 && prn<=37)){
        trace(2, "SBF decode_cmpcnav2 prn error: sat=%d\n", prn);
        return -1;
    }

    eph.week   = U2(p +   4);
    eph.code   = 0;
    eph.flag   = U1(p +   7);
    eph.toes   = U4(p +   8);
    eph.A      = R8(p +  12);
    eph.Adot   = R8(p +  20);
    eph.deln   = R4(p +  28) * PI;
    eph.ndot   = R4(p +  32) * PI;
    eph.M0     = R8(p +  36) * PI;
    eph.e      = R8(p +  44);
    eph.omg    = R8(p +  52) * PI;
    eph.OMG0   = R8(p +  60) * PI;
    eph.OMGd   = R8(p +  68) * PI;
    eph.i0     = R8(p +  76) * PI;
    eph.idot   = R4(p +  84) * PI;
    eph.cis    = R4(p +  88);
    eph.cic    = R4(p +  92);
    eph.crs    = R4(p +  96);
    eph.crc    = R4(p + 100);
    eph.cus    = R4(p + 104);
    eph.cuc    = R4(p + 108);
    tocs       = U4(p + 112);
    eph.f2     = R4(p + 116);
    eph.f1     = R4(p + 120);
    eph.f0     = R8(p + 124);
    tops       = U4(p + 132);
    eph.sva    = U1(p + 139); /* to be checked */
    eph.svh    = U1(p + 140);

    if (sbf_id == ID_BDSCNAV1) {
        if (R4(p + 144) != -2.e10)
            eph.tgd[4] = R4(p + 144);  /* ISC_B1Cd */
        if (R4(p + 148) != -2.e10)
            eph.tgd[2] = R4(p + 148);  /* T_GDB1Cp */
        if (R4(p + 152) != -2.e10)
            eph.tgd[3] = R4(p + 152);  /* T_GDB2ap */
    } else if (sbf_id == ID_BDSCNAV2) {
        if (R4(p + 144) != -2.e10)
            eph.tgd[5] = R4(p + 144);  /* ISC_B2ad */
        if (R4(p + 148) != -2.e10)
            eph.tgd[3] = R4(p + 148);  /* T_GDB2ap */
        if (R4(p + 152) != -2.e10)
            eph.tgd[2] = R4(p + 152);  /* T_GDB1Cp */
    } else if (sbf_id == ID_BDSCNAV3) {
        if (R4(p + 144) != -2.e10)
            eph.tgd[1] = R4(p + 144);  /* T_GDB2bI */
    }

    eph.fit    = 0;

    eph.iodc = 0; /* No IOD in GPS CNAV; prediction time is not represented in eph_t. */
    eph.iode = 0;
    if (tops >= 604800 || eph.toes >= 604800 || tocs >= 604800) return -1;

    eph.toe = gpst2time(eph.week, eph.toes);
    eph.toc = gpst2time(eph.week, tocs);
    eph.ttr = raw->time;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF BDS Decoded C-Navigation 2 Data (PRN=%d, IODE=%d, IODC=%d, TOES=%6.0f )", prn, eph.iode, eph.iodc, eph.toes);
    }

    if (!strstr(raw->opt, "-EPHALL")) {
        if ((eph.iode == raw->nav.eph[sat-1].iode) &&
            (eph.iodc == raw->nav.eph[sat-1].iodc)) return 0;
    }

    eph.sat = sat;
    raw->nav.eph[sat-1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;

    return 2;
}


]=] [=[static int decode_cmpcnav2(raw_t *raw, uint32_t sbf_id)
{
    uint8_t *p = raw->buff+8;  /* points at TOW location */
    eph_t eph = {0};
    uint32_t tocs, tops;
    uint8_t prn, sat;

    trace(4, "SBF decode_cmpcnav2: len=%d\n", raw->len);

    if ((U2(raw->buff+4)>>13) != 0 || raw->len != (sbf_id == ID_BDSCNAV3 ? 152 : 160)) {
        trace(2, "SBF decode_cmpcnav2 frame length error: len=%d\n", raw->len);
        return -1;
    }

    prn = U1(p+6);
    sat = satno(SYS_CMP, prn);

    if (sat == 0) return -1;

    if (!(prn>=1 && prn<=MAXPRNCMP)){
        trace(2, "SBF decode_cmpcnav2 prn error: sat=%d\n", prn);
        return -1;
    }

    eph.week   = U2(p +   4);
    eph.code   = 0;
    eph.flag   = U1(p +   7);
    eph.toes   = U4(p +   8);
    eph.A      = R8(p +  12);
    eph.Adot   = R8(p +  20);
    eph.deln   = R4(p +  28) * PI;
    eph.ndot   = R4(p +  32) * PI;
    eph.M0     = R8(p +  36) * PI;
    eph.e      = R8(p +  44);
    eph.omg    = R8(p +  52) * PI;
    eph.OMG0   = R8(p +  60) * PI;
    eph.OMGd   = R4(p +  68) * PI;
    eph.i0     = R8(p +  72) * PI;
    eph.idot   = R4(p +  80) * PI;
    eph.cis    = R4(p +  84);
    eph.cic    = R4(p +  88);
    eph.crs    = R4(p +  92);
    eph.crc    = R4(p +  96);
    eph.cus    = R4(p + 100);
    eph.cuc    = R4(p + 104);
    tocs       = U4(p + 108);
    eph.f2     = R4(p + 112);
    eph.f1     = R4(p + 116);
    eph.f0     = R8(p + 120);
    tops       = U4(p + 128);
    eph.sva    = -1; /* SISAI components live in ROS metadata, not legacy URA. */
    eph.svh    = U1(p + 136);

    if (sbf_id == ID_BDSCNAV1) {
        if (R4(p + 140) != -2.e10)
            eph.tgd[4] = R4(p + 140);  /* ISC_B1Cd */
        if (R4(p + 144) != -2.e10)
            eph.tgd[2] = R4(p + 144);  /* T_GDB1Cp */
        if (R4(p + 148) != -2.e10)
            eph.tgd[3] = R4(p + 148);  /* T_GDB2ap */
    } else if (sbf_id == ID_BDSCNAV2) {
        if (R4(p + 140) != -2.e10)
            eph.tgd[5] = R4(p + 140);  /* ISC_B2ad */
        if (R4(p + 144) != -2.e10)
            eph.tgd[3] = R4(p + 144);  /* T_GDB2ap */
        if (R4(p + 148) != -2.e10)
            eph.tgd[2] = R4(p + 148);  /* T_GDB1Cp */
    } else if (sbf_id == ID_BDSCNAV3) {
        if (R4(p + 140) != -2.e10)
            eph.tgd[1] = R4(p + 140);  /* T_GDB2bI */
    }

    eph.fit    = 0;

    /* B-CNAV3 reserves these three bytes; only B-CNAV1/2 carry IOD here. */
    eph.iodc = sbf_id == ID_BDSCNAV3 ? 0 : U2(p + 138);
    eph.iode = sbf_id == ID_BDSCNAV3 ? 0 : U1(p + 137);
    if (tops >= 604800 || eph.toes >= 604800 || tocs >= 604800) return -1;

    /* Header is GPST; navigation reference times are BDT. Resolve the week
       against reception in BDT, including the first 14 seconds of a GPS week. */
    {
        double now_bdt = time2bdt(gpst2bdt(raw->time), &eph.week);
        int toe_week = eph.week, toc_week = eph.week;
        if (eph.toes < now_bdt - 302400) ++toe_week;
        if (eph.toes > now_bdt + 302400) --toe_week;
        if (tocs < now_bdt - 302400) ++toc_week;
        if (tocs > now_bdt + 302400) --toc_week;
        eph.week = toe_week;
        eph.toe = bdt2gpst(bdt2time(toe_week, eph.toes));
        eph.toc = bdt2gpst(bdt2time(toc_week, tocs));
    }
    eph.ttr = raw->time;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF BDS Decoded C-Navigation 2 Data (PRN=%d, IODE=%d, IODC=%d, TOES=%6.0f )", prn, eph.iode, eph.iodc, eph.toes);
    }

    if (!strstr(raw->opt, "-EPHALL")) {
        if ((eph.iode == raw->nav.eph[sat-1].iode) &&
            (eph.iodc == raw->nav.eph[sat-1].iodc)) return 0;
    }

    eph.sat = sat;
    raw->nav.eph[sat-1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;

    return 2;
}


]=])
    gnss_receiver_replace(1 [=[    eph.sva = sva == 255 ? 0 : sva;]=] [=[    eph.sva = sva; /* 255 = unknown, not zero-metre accuracy */]=])
    gnss_receiver_replace(1 [=[    if (raw->len < 96) {]=] [=[    if (raw->len < ((U2(raw->buff+4)>>13) ? 100 : 96)) {]=])
    gnss_receiver_replace(1 [=[    eph.iode   = U2(p +  78);]=] [=[    if (U2(p+78) >= 1440 || U2(p+78)%15) return -1;
    eph.iode   = U2(p + 78)/15; /* SBF tb is minutes; backend tb is 15-minute index */]=])
    gnss_receiver_replace(1 [=[    eph.sva    = U2(p +  86);]=] [=[    /* F_T is hundredths of a metre, not an accuracy index. The original physical value
       is not in the ROS schema; -1 explicitly means index unavailable. */
    eph.sva = -1;
    {
        const unsigned ft[15] = {100,200,250,400,500,700,1000,1200,1400,1600,3200,6400,12800,25600,51200};
        int i, p1 = U1(p+74);
        for (i=0;i<15;++i) if (U2(p+86)==ft[i]) eph.sva=i;
        if (p1 != 0 && p1 != 30 && p1 != 45 && p1 != 60) return -1;
        if (U1(p+75)>1 || U1(p+80)>3 || U1(p+81)>3 || U1(p+82)>1 || U1(p+83)>1) return -1;
        p1 = p1 ? p1/15-1 : 0;
        eph.flags = (U1(p+80)<<7)|(U1(p+83)<<6)|(U1(p+75)<<4)|(p1<<2)|U1(p+81);
        eph.svh |= U1(p+82)<<3;
        if ((U2(raw->buff+4)>>13) >= 1 && U1(p+88) <= 1)
            eph.svh |= 4|(U1(p+88)<<1);
    }
    if (toes >= 604800) return -1;]=])
    gnss_receiver_replace(1 [=[    return n > 0 ? 1 : 0;
}

/* decode SBF measurements]=] [=[    raw->obuf.n = 0;
    return n > 0 ? 1 : 0;
}

/* Package-owned completion API: no fabricated EndOfMeas bytes or time change. */
extern int gnss_flush_sbf_observations(raw_t *raw) { return flushobuf(raw); }

/* decode SBF measurements]=])
    gnss_receiver_replace(1 [=[    int ant_sel = 0; /* antenna selection (0:main) */

    if (timediff(raw->time, sbf->current_time) > 0) {]=] [=[    int ant_sel = 0; /* antenna selection (0:main) */
    int doppler1_valid;
    /* Validate the entire layout before changing the pending epoch. */
    if (raw->len < 20 || (U1(p+3)&0x80)) return -1;
    {
        unsigned offset=20, row, count=U1(p), a=U1(p+1), b=U1(p+2);
        if (a < 20) return -1;
        for (row=0; row<count; ++row) {
            unsigned children;
            if (offset+a > (unsigned)raw->len) return -1;
            children=U1(raw->buff+offset+19);
            if (children && b < 12) return -1;
            offset+=a+b*children;
            if (offset > (unsigned)raw->len) return -1;
        }
    }

    if (timediff(raw->time, sbf->current_time) > 0) {]=])
    gnss_receiver_replace(1 "    if (timediff(raw->time, sbf->current_time) > 0) {" "    if (timediff(raw->time, sbf->current_time) != 0) {")
    gnss_receiver_replace(1 "        init_obsd(raw->time, sat, raw->obuf.data+n);" [=[        init_obsd(raw->time, sat, raw->obuf.data+n);
        raw->obuf.data[n].P[idx]=raw->obuf.data[n].L[idx]=NAN;
        raw->obuf.data[n].D[idx]=raw->obuf.data[n].SNR[idx]=NAN;]=])
    # S1 ObsInfo half-cycle bit describes the signal even when phase/lock is DNU.
    # Keep this flag; only a valid phase/lock sample may update slip history.
    gnss_receiver_replace(1 "        lock = U2(p+16);" [=[        raw->obuf.data[n].LLI[idx] = (info & 4) ? LLI_HALFC : 0;
        lock = U2(p+16);]=])
    gnss_receiver_replace(1 "            if (lock != 255) {" [=[            raw->obuf.data[n].LLI[idx] = (info & 4) ? LLI_HALFC : 0;
            if (lock != 255) {]=])
    gnss_receiver_replace(1 "        if (I4(p+8) != -2147483648) {" [=[        doppler1_valid = I4(p+8) != (-2147483647-1);
        if (doppler1_valid) {]=])
    gnss_receiver_replace(1 "            P2 = 0.0;" [=[            raw->obuf.data[n].P[idx]=raw->obuf.data[n].L[idx]=NAN;
            raw->obuf.data[n].D[idx]=raw->obuf.data[n].SNR[idx]=NAN;
            P2 = 0.0;]=])
    gnss_receiver_replace(1 "            if (P2!=0.0 && freq2>0.0 && (I1(p+4)!=-128 || U2(p+8)!=0)) {" "            if (P2!=0.0 && freq2>0.0 && lock!=255 && (I1(p+4)!=-128 || U2(p+8)!=0)) {")
    gnss_receiver_replace(1 "            if (D1!=0.0 && freq1>0.0 && freq2>0.0 && (getbits(p+3, 0, 5)!=-16 || U2(p+10)!=0)) {" "            if (doppler1_valid && freq1>0.0 && freq2>0.0 && (getbits(p+3, 0, 5)!=-16 || U2(p+10)!=0)) {")
    gnss_receiver_replace(1 "    if (tow != 4294967295u && week != 65535u) {" "    if (tow < 604800000u && week != 65535u) {")
    gnss_receiver_replace(1 [=[    int sig, n, idx, sat, ret = 0;
    int ant_sel = 0; /* antenna selection (0:main) */]=] [=[    int sig, n, idx, sat, ret = 0;
    int ant_sel = 0; /* antenna selection (0:main) */
    if (raw->len < 20 || U1(p+7) < 10 ||
        20u + U1(p+6)*(unsigned)U1(p+7) > (unsigned)raw->len) return -1;]=])
    # S1 pp268-269: MeasExtra N is modulo 256, not the full row count.
    # Infer complete rows from the validated payload length; widen the loop
    # counter too so a 256-row block cannot silently omit CN0 refinements.
    gnss_receiver_replace(1 "    uint8_t n_chn, sbLen, i, misc;"
      "    unsigned n_chn, i;\n    uint8_t sbLen, misc;")
    gnss_receiver_replace(1 "    sbLen = U1(p+7);"
      "    sbLen = U1(p+7);\n    n_chn += 256u * (((unsigned)(raw->len - 20) / sbLen - n_chn) / 256u);")
    gnss_receiver_replace(1 [=[        if (sig == 31)
            sig = (U1(p_chan + 15) >> 3) + 32;]=] [=[        if (sig == 31) {
            if (sbLen < 16) continue;
            sig = (U1(p_chan + 15) >> 3) + 32;
        }]=])
    gnss_receiver_replace(1 "        if (n < 0) {" "        if (n < 0 || n >= raw->obuf.n) {")
    # A delayed marker from a flushed epoch must not prematurely flush a newer one.
    gnss_receiver_replace(1 [=[        case ID_MEASEPOCH_END:
            if (raw->outtype) {]=] [=[        case ID_MEASEPOCH_END:
            if (raw->obuf.n == 0 || timediff(raw->time,raw->obuf.data[0].time)!=0) return 0;
            if (raw->outtype) {]=])
  endif()
  include("${CMAKE_CURRENT_LIST_DIR}/rtklib_i03.cmake")
  include("${CMAKE_CURRENT_LIST_DIR}/rtklib_diagnostics.cmake")
  set(output "${CMAKE_CURRENT_BINARY_DIR}/rtklib_patched/${receiver}.c")
  set(previous "")
  if(EXISTS "${output}")
    file(READ "${output}" previous)
  endif()
  if(NOT previous STREQUAL _receiver_source)
    file(WRITE "${output}" "${_receiver_source}")
  endif()
  list(REMOVE_ITEM _core_sources "${input}")
  list(APPEND _core_sources "${output}")
endforeach()

# RTCM 1020 GLONASS metadata: M follows NT (11 bits); ln is bit 3 of
# the backend's extended health, distinct from Cn_a in bit 2.
foreach(receiver rtcm3 rtcm3e)
  set(input "${RTKLIB_ROOT}/src/${receiver}.c")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${input}")
  file(READ "${input}" _receiver_source)
  if(receiver STREQUAL "rtcm3")
    gnss_receiver_replace(1 "    ver    =getbitu(rtcm->buff,i,3); i+=3;"
      "    ver    =getbitu(rtcm->buff,i,3); i+=3;\n    if (ver!=1) return GNSS_RX_FAIL(GNSS_RX_VERSION);")
    # A 64-bit MSM satellite mask can name unsupported GLONASS slots. Neither
    # the 32-entry FCN cache nor geph may be indexed until satno() accepted it.
    gnss_receiver_replace(1 "if (ex&&ex[i]<=13) {" "if (sat&&ex&&ex[i]<=13) {")
    gnss_receiver_replace(1 "else if (rtcm->nav.glo_fcn[prn-1]>0) {"
      "else if (sat&&rtcm->nav.glo_fcn[prn-1]>0) {")
    gnss_receiver_replace(1 "    obs->data[i].sat=sat;" "    obs->data[i].sat=sat;\n    obs->data[i].freq=255;")
    gnss_receiver_replace(2 "        pr1=pr1*0.02+amb*PRUNIT_GLO;" [=[        rtcm->obs.data[index].freq=fcn<=13 ? fcn : 255;
        pr1=pr1*0.02+amb*PRUNIT_GLO;]=])
    gnss_receiver_replace(2 [=[        if (!rtcm->nav.glo_fcn[prn-1]) {
            rtcm->nav.glo_fcn[prn-1]=fcn-7+8; /* fcn+8 */
        }]=] [=[        rtcm->nav.glo_fcn[prn-1]=fcn<=13 ? fcn+1 : 0;]=])
    gnss_receiver_replace(1 [=[                if (!rtcm->nav.glo_fcn[prn-1]) {
                    rtcm->nav.glo_fcn[prn-1]=fcn+8; /* fcn+8 */
                }]=] [=[                rtcm->nav.glo_fcn[prn-1]=fcn+8; /* FCN+8 */]=])
    gnss_receiver_replace(1 "                freq=fcn<-7?0.0:code2freq(sys,code[k],fcn);" [=[                rtcm->obs.data[index].freq=sys==SYS_GLO&&fcn>=-7&&fcn<=6 ? fcn+7 : 255;
                freq=fcn<-7?0.0:code2freq(sys,code[k],fcn);]=])
    gnss_receiver_replace(1 "    int M      =getbitu(rtcm->buff,i, 2);"
                           "    i+=11; /* NT */\n    int M      =getbitu(rtcm->buff,i, 2);")
    include("${CMAKE_CURRENT_LIST_DIR}/rtklib_rtcm_observations.cmake")
    include("${CMAKE_CURRENT_LIST_DIR}/rtklib_rtcm_auxiliary.cmake")
  else()
    gnss_receiver_replace(1 "    int ln = (geph->svh >> 2) & 1;"
                           "    int ln = (geph->svh >> 3) & 1;")
  endif()
  include("${CMAKE_CURRENT_LIST_DIR}/rtklib_i03.cmake")
  include("${CMAKE_CURRENT_LIST_DIR}/rtklib_diagnostics.cmake")
  set(output "${CMAKE_CURRENT_BINARY_DIR}/rtklib_patched/${receiver}.c")
  if(EXISTS "${output}")
    file(READ "${output}" previous)
  else()
    set(previous "")
  endif()
  if(NOT previous STREQUAL _receiver_source)
    file(WRITE "${output}" "${_receiver_source}")
  endif()
  list(REMOVE_ITEM _core_sources "${input}")
  list(APPEND _core_sources "${output}")
endforeach()

include("${CMAKE_CURRENT_LIST_DIR}/rtklib_stream.cmake")

include("${CMAKE_CURRENT_LIST_DIR}/rtklib_raw_navigation.cmake")
