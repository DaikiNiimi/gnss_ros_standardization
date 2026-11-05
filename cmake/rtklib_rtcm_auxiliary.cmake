# Count validation and bounded auxiliary strings; applied only to generated RTCM C.
gnss_receiver_replace(3 [==[des[32]]==] [==[des[MAXANT]]==])
gnss_receiver_replace(3 [==[for (j=0;j<n&&j<31;j++) {
            des[j]=(char)getbitu(rtcm->buff,i,8); i+=8;]==] [==[for (j=0;j<n;j++) {
            if (j<MAXANT-1) des[j]=(char)getbitu(rtcm->buff,i,8);
            i+=8;]==])
gnss_receiver_replace(2 [==[sno[32]]==] [==[sno[MAXANT]]==])
gnss_receiver_replace(2 [==[for (j=0;j<m&&j<31;j++) {
            sno[j]=(char)getbitu(rtcm->buff,i,8); i+=8;]==] [==[for (j=0;j<m;j++) {
            if (j<MAXANT-1) sno[j]=(char)getbitu(rtcm->buff,i,8);
            i+=8;]==])
gnss_receiver_replace(1 [==[rec[32]]==] [==[rec[MAXANT]]==])
gnss_receiver_replace(1 [==[for (j=0;j<n1&&j<31;j++) {
            rec[j]=(char)getbitu(rtcm->buff,i,8); i+=8;]==] [==[for (j=0;j<n1;j++) {
            if (j<MAXANT-1) rec[j]=(char)getbitu(rtcm->buff,i,8);
            i+=8;]==])
gnss_receiver_replace(1 [==[ver[32]]==] [==[ver[MAXANT]]==])
gnss_receiver_replace(1 [==[for (j=0;j<n2&&j<31;j++) {
            ver[j]=(char)getbitu(rtcm->buff,i,8); i+=8;]==] [==[for (j=0;j<n2;j++) {
            if (j<MAXANT-1) ver[j]=(char)getbitu(rtcm->buff,i,8);
            i+=8;]==])
gnss_receiver_replace(1 [==[rsn[32]]==] [==[rsn[MAXANT]]==])
gnss_receiver_replace(1 [==[for (j=0;j<n3&&j<31;j++) {
            rsn[j]=(char)getbitu(rtcm->buff,i,8); i+=8;]==] [==[for (j=0;j<n3;j++) {
            if (j<MAXANT-1) rsn[j]=(char)getbitu(rtcm->buff,i,8);
            i+=8;]==])
gnss_receiver_replace(2 [==[strncpy(rtcm->sta.antdes,des,n); rtcm->sta.antdes[n]='\0';]==] [==[snprintf(rtcm->sta.antdes,sizeof(rtcm->sta.antdes),"%s",des);]==])
gnss_receiver_replace(1 [==[strncpy(rtcm->sta.antsno,sno,m); rtcm->sta.antsno[m]='\0';]==] [==[snprintf(rtcm->sta.antsno,sizeof(rtcm->sta.antsno),"%s",sno);]==])
gnss_receiver_replace(1 [==[strncpy(rtcm->sta.antdes, des,n ); rtcm->sta.antdes [n] ='\0';]==] [==[snprintf(rtcm->sta.antdes,sizeof(rtcm->sta.antdes),"%s",des);]==])
gnss_receiver_replace(1 [==[strncpy(rtcm->sta.antsno, sno,m ); rtcm->sta.antsno [m] ='\0';]==] [==[snprintf(rtcm->sta.antsno,sizeof(rtcm->sta.antsno),"%s",sno);]==])
gnss_receiver_replace(1 [==[strncpy(rtcm->sta.rectype,rec,n1); rtcm->sta.rectype[n1]='\0';]==] [==[snprintf(rtcm->sta.rectype,sizeof(rtcm->sta.rectype),"%s",rec);]==])
gnss_receiver_replace(1 [==[strncpy(rtcm->sta.recver, ver,n2); rtcm->sta.recver [n2]='\0';]==] [==[snprintf(rtcm->sta.recver,sizeof(rtcm->sta.recver),"%s",ver);]==])
gnss_receiver_replace(1 [==[strncpy(rtcm->sta.recsno, rsn,n3); rtcm->sta.recsno [n3]='\0';]==] [==[snprintf(rtcm->sta.recsno,sizeof(rtcm->sta.recsno),"%s",rsn);]==])
gnss_receiver_replace(1 [==[if (i+20>=rtcm->len*8)]==] [==[if (i+20>rtcm->len*8)]==])
gnss_receiver_replace(1 [==[    mask =getbitu(rtcm->buff,i, 4); i+= 4;]==] [==[    mask =getbitu(rtcm->buff,i, 4); i+= 4;
    { int count=0; for (j=0;j<4;j++) count+=(mask>>j)&1;
      if (i+16*count>rtcm->len*8) return GNSS_RX_FAIL(GNSS_RX_LENGTH); }]==])
gnss_receiver_replace(1 [==[    for (j=0;j<nsat&&i+121+np+ni+nj<=rtcm->len*8;j++) {]==] [==[    if (i+nsat*(121+np+ni+nj)>rtcm->len*8) return GNSS_RX_FAIL(GNSS_RX_LENGTH);
    for (j=0;j<nsat&&i+121+np+ni+nj<=rtcm->len*8;j++) {]==])
gnss_receiver_replace(1 [==[    for (j=0;j<nsat&&i+70+np<=rtcm->len*8;j++) {]==] [==[    if (i+nsat*(70+np)>rtcm->len*8) return GNSS_RX_FAIL(GNSS_RX_LENGTH);
    for (j=0;j<nsat&&i+70+np<=rtcm->len*8;j++) {]==])
gnss_receiver_replace(1 [==[    for (j=0;j<nsat&&i+191+np+ni+nj<=rtcm->len*8;j++) {]==] [==[    if (i+nsat*(191+np+ni+nj)>rtcm->len*8) return GNSS_RX_FAIL(GNSS_RX_LENGTH);
    for (j=0;j<nsat&&i+191+np+ni+nj<=rtcm->len*8;j++) {]==])
gnss_receiver_replace(1 [==[    for (j=0;j<nsat&&i+6+np<=rtcm->len*8;j++) {]==] [==[    if (i+nsat*(6+np)>rtcm->len*8) return GNSS_RX_FAIL(GNSS_RX_LENGTH);
    for (j=0;j<nsat&&i+6+np<=rtcm->len*8;j++) {]==])
gnss_receiver_replace(1 [==[    for (j=0;j<nsat&&i+22+np<=rtcm->len*8;j++) {]==] [==[    if (i+nsat*(22+np)>rtcm->len*8) return GNSS_RX_FAIL(GNSS_RX_LENGTH);
    for (j=0;j<nsat&&i+22+np<=rtcm->len*8;j++) {]==])
gnss_receiver_replace(1 [==[    for (j=0;j<nsat&&i+5+np<=rtcm->len*8;j++) {]==] [==[    { int scan=i, satellite, biases;
      for (satellite=0;satellite<nsat;satellite++) {
        if (scan+np+5>rtcm->len*8) return GNSS_RX_FAIL(GNSS_RX_LENGTH);
        biases=getbitu(rtcm->buff,scan+np,5);
        scan+=np+5;
        if (scan+biases*19>rtcm->len*8) return GNSS_RX_FAIL(GNSS_RX_LENGTH);
        scan+=biases*19;
      }
    }
    for (j=0;j<nsat&&i+5+np<=rtcm->len*8;j++) {]==])
gnss_receiver_replace(1 [==[    for (j=0;j<nsat&&i+5+17+np<=rtcm->len*8;j++) {]==] [==[    { int scan=i, satellite, biases;
      for (satellite=0;satellite<nsat;satellite++) {
        if (scan+np+22>rtcm->len*8) return GNSS_RX_FAIL(GNSS_RX_LENGTH);
        biases=getbitu(rtcm->buff,scan+np,5);
        scan+=np+22;
        if (scan+biases*(subtype==0?49:32)>rtcm->len*8) return GNSS_RX_FAIL(GNSS_RX_LENGTH);
        scan+=biases*(subtype==0?49:32);
      }
    }
    for (j=0;j<nsat&&i+5+17+np<=rtcm->len*8;j++) {]==])
gnss_receiver_replace(1 [==[    int ret=0,type=getbitu(rtcm->buff,24,12),week;]==] [==[    int ret=0,type=getbitu(rtcm->buff,24,12),week;
    const gtime_t accepted_time=rtcm->time;]==])
gnss_receiver_replace(1 [==[    if (ret>=0) {]==] [==[    if (ret<0) rtcm->time=accepted_time;
    if (ret>=0) {]==])

# SSR epoch constraints precede nearest-week/day adjustment.
gnss_receiver_replace(1 "        tod=getbitu(rtcm->buff,i,17); i+=17;"
  "        tod=getbitu(rtcm->buff,i,17); i+=17;\n        if (tod>=86400.0) return GNSS_RX_FAIL(GNSS_RX_TIME);")
gnss_receiver_replace(2 "        tow=getbitu(rtcm->buff,i,20); i+=20;"
  "        tow=getbitu(rtcm->buff,i,20); i+=20;\n        if (tow>=604800.0) return GNSS_RX_FAIL(GNSS_RX_TIME);")
gnss_receiver_replace(3 "    i=decode_ssr_epoch(rtcm,sys,subtype);"
  "    if ((i=decode_ssr_epoch(rtcm,sys,subtype))<0) return i;")
