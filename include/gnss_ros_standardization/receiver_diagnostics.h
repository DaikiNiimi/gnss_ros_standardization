#ifndef GNSS_RECEIVER_DIAGNOSTICS_H
#define GNSS_RECEIVER_DIAGNOSTICS_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Events have different units. They must never be added into a loss total.
 * No change to raw_t, rtcm_t, obsd_t or the ROS interfaces is required. */
enum gnss_receiver_event {
    GNSS_RX_SATELLITE, GNSS_RX_SIGNAL, GNSS_RX_SLOT,
    GNSS_RX_CAPACITY_ROWS, GNSS_RX_CAPACITY_ATTEMPTS,
    GNSS_RX_QUALITY, GNSS_RX_SELECTION,
    GNSS_RX_VALIDATION, GNSS_RX_LENGTH, GNSS_RX_TIME, GNSS_RX_VERSION,
    GNSS_RX_REFERENCE, GNSS_RX_NAV_NO_OUTPUT, GNSS_RX_NAV_WAIT,
    GNSS_RX_NAV_UNSUPPORTED, GNSS_RX_IMU_WAIT, GNSS_RX_IMU_REJECT,
    GNSS_RX_FRONTEND_REJECT,
    GNSS_RX_ARRAY, GNSS_RX_FIELD, GNSS_RX_CHECKSUM, GNSS_RX_STATION,
    GNSS_RX_PAYLOAD_UNSUPPORTED, GNSS_RX_NAV_DUPLICATE, GNSS_RX_NAV_QUALITY,
    GNSS_RX_NAV_PAGE_IGNORED, GNSS_RX_NAV_INCONSISTENT, GNSS_RX_NAV_AUXILIARY,
    GNSS_RX_NAV_SATELLITE, GNSS_RX_VALUE, GNSS_RX_IMU_SCALE,
    GNSS_RX_ASSEMBLY_REPLACED, GNSS_RX_DUPLICATE_INPUT, GNSS_RX_EVENT_COUNT
};
enum gnss_navigation_family { GNSS_NAV_GPS, GNSS_NAV_INAV, GNSS_NAV_FNAV,
    GNSS_NAV_BDS1, GNSS_NAV_BDS2, GNSS_NAV_GLO, GNSS_NAV_IRN, GNSS_NAV_FAMILIES };
/* Satellite IDs in the pinned RTKLIB fit in uint8_t. Index zero is unused. */
#define GNSS_DIAGNOSTIC_SATELLITES 256
typedef struct {
    uint64_t counts[GNSS_RX_EVENT_COUNT];
    const char *functions[GNSS_RX_EVENT_COUNT];
    unsigned lines[GNSS_RX_EVENT_COUNT];
    /* 0:not observed, 1:ready, 2:waiting, 3:complete headers but rejected. */
    uint8_t navigation[GNSS_DIAGNOSTIC_SATELLITES][GNSS_NAV_FAMILIES];
    uint16_t missing[GNSS_DIAGNOSTIC_SATELLITES][GNSS_NAV_FAMILIES];
    uint8_t irn_seen[GNSS_DIAGNOSTIC_SATELLITES];
} gnss_receiver_diagnostics;
/* A thread-local, scoped sink: callers without a sink retain original behavior.
 * The caller owns the sink and must restore it before its lifetime ends. */
gnss_receiver_diagnostics *gnss_receiver_diagnostics_swap(gnss_receiver_diagnostics *sink);
void gnss_receiver_event_record(enum gnss_receiver_event event, uint64_t count,
                                const char *function, unsigned line);
int gnss_receiver_negative(const char *function, unsigned line);
void gnss_receiver_navigation(unsigned sat, unsigned family, unsigned state, unsigned missing);
unsigned gnss_receiver_irn_fragment(unsigned sat, unsigned fragment);
#define GNSS_RX_REJECT(kind) (GNSS_RX_EVENT(kind, 1), false)
#define GNSS_RX_EVENT(kind, count) gnss_receiver_event_record(kind, count, __func__, __LINE__)
#define GNSS_RX_FAIL(kind) (GNSS_RX_EVENT(kind, 1), -1)
#ifdef __cplusplus
}
namespace gnss_ros_standardization::wire {
class ReceiverDiagnosticScope {
 public:
  explicit ReceiverDiagnosticScope(gnss_receiver_diagnostics& sink)
      : previous_(gnss_receiver_diagnostics_swap(&sink)) {}
  ~ReceiverDiagnosticScope() { gnss_receiver_diagnostics_swap(previous_); }
  ReceiverDiagnosticScope(const ReceiverDiagnosticScope&)=delete;
  ReceiverDiagnosticScope& operator=(const ReceiverDiagnosticScope&)=delete;
 private:
  gnss_receiver_diagnostics* previous_;
};
}
#endif
#endif
