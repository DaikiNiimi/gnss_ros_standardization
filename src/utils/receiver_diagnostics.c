#include "gnss_ros_standardization/receiver_diagnostics.h"
#include <string.h>
static _Thread_local gnss_receiver_diagnostics *current_sink;

gnss_receiver_diagnostics *gnss_receiver_diagnostics_swap(gnss_receiver_diagnostics *sink)
{
    gnss_receiver_diagnostics *previous=current_sink;
    current_sink=sink;
    return previous;
}
void gnss_receiver_event_record(enum gnss_receiver_event event, uint64_t count,
                                const char *function, unsigned line)
{
    if (!current_sink || event<0 || event>=GNSS_RX_EVENT_COUNT || !count) return;
    uint64_t *value=&current_sink->counts[event];
    *value=UINT64_MAX-*value<count ? UINT64_MAX : *value+count;
    current_sink->functions[event]=function;
    current_sink->lines[event]=line;
}
int gnss_receiver_negative(const char *function, unsigned line)
{
    /* These helpers use -1 for a slot/mapping lookup, not a rejected payload.
     * Their callers have explicit exclusion hooks with the correct units. */
    if (strcmp(function,"obsindex") && strcmp(function,"sig_idx") &&
        strcmp(function,"meas2_sig2idx") && strcmp(function,"meas3_sig2idx") &&
        strcmp(function,"decode_track_stat") && strcmp(function,"checkpri"))
        gnss_receiver_event_record(GNSS_RX_VALIDATION,1,function,line);
    return -1;
}

void gnss_receiver_navigation(unsigned sat, unsigned family, unsigned state, unsigned missing)
{
    if (!current_sink || !sat || sat>=GNSS_DIAGNOSTIC_SATELLITES || family>=GNSS_NAV_FAMILIES) return;
    current_sink->navigation[sat][family]=(uint8_t)state;
    current_sink->missing[sat][family]=(uint16_t)missing;
}
unsigned gnss_receiver_irn_fragment(unsigned sat, unsigned fragment)
{
    if (!current_sink || !sat || sat>=GNSS_DIAGNOSTIC_SATELLITES) return 0;
    if (fragment<2) current_sink->irn_seen[sat]|=1u<<fragment;
    return current_sink->irn_seen[sat];
}
