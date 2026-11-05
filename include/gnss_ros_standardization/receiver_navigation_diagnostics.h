#ifndef GNSS_RECEIVER_NAVIGATION_DIAGNOSTICS_H
#define GNSS_RECEIVER_NAVIGATION_DIAGNOSTICS_H
/* Included only by instrumented RTKLIB C sources, after rtklib.h. Diagnostic
 * reads mirror the pinned rcvraw.c assembly layout, never mutate raw buffers. */
#include "gnss_ros_standardization/receiver_diagnostics.h"
_Static_assert(MAXSAT<GNSS_DIAGNOSTIC_SATELLITES,"Review diagnostic satellite bound");
static unsigned gnss_nav_missing(const raw_t *raw, int sat, unsigned family)
{
    unsigned missing=0;
    const uint8_t *b=raw->subfrm[sat-1];
    for(unsigned part=1;part<=10;++part) {
        unsigned actual=part, required=0;
        switch(family) {
          case GNSS_NAV_GPS: required=part<=3; if(required) actual=getbitu(b+(part-1)*30,43,3); break;
          case GNSS_NAV_INAV: required=part<=6; if(required) actual=getbitu(b+part*16,0,6); break;
          case GNSS_NAV_FNAV: required=part<=4; if(required) actual=getbitu(b+128+(part-1)*31,0,6); break;
          case GNSS_NAV_BDS1: required=part<=3; if(required) actual=getbitu(b+(part-1)*38,15,3); break;
          case GNSS_NAV_BDS2: required=part!=2; if(required) actual=getbitu(b+(part-1)*38,42,4); break;
          case GNSS_NAV_GLO: required=part<=(raw->format==STRFMT_SEPT ? 5u : 4u); if(required) actual=getbitu(b+(part-1)*10,1,4); break;
          case GNSS_NAV_IRN: required=part<=2; if(required && !(gnss_receiver_irn_fragment(sat,2)&(1u<<(part-1)))) actual=0; break;
        }
        if(required && actual!=part) missing|=1u<<(part-1);
    }
    return missing;
}
static int gnss_nav_wait(const raw_t *raw, int sat, unsigned family, int failed)
{
    unsigned missing=gnss_nav_missing(raw,sat,family);
    /* All headers present does not imply IOD/time/parity consistency. */
    unsigned state=failed && !missing ? 3 : 2;
    gnss_receiver_navigation(sat,family,state,missing);
    GNSS_RX_EVENT(state==3 ? GNSS_RX_NAV_INCONSISTENT : GNSS_RX_NAV_WAIT,1);
    return 0;
}
#endif
