# Surface POSIX serial hangup/fatal read errors to strstat; idle EAGAIN/zero
# reads remain connected. Reopen policy belongs to the ROS session wrapper.
set(receiver stream)
set(input "${RTKLIB_ROOT}/src/stream.c")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${input}")
file(READ "${input}" _receiver_source)
gnss_receiver_replace(1 "static int readserial(serial_t *serial, uint8_t *buff, int n, char *msg)" [=[#ifndef WIN32
#include <poll.h>
#endif
static int readserial(serial_t *serial, uint8_t *buff, int n, char *msg)]=])
gnss_receiver_replace(1 "    if ((nr=read(serial->dev,buff,n))<0) return 0;" [=[    struct pollfd descriptor={serial->dev,POLLIN,0};
    int hung_up=poll(&descriptor,1,0)>0 && (descriptor.revents&(POLLHUP|POLLERR|POLLNVAL));
    nr=read(serial->dev,buff,n);
    if (nr<0) {
        if (errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR) {
            serial->error=1;
            sprintf(msg,"serial read error (%d)",errno);
        }
        return 0;
    }
    if (nr==0 && hung_up) {
        serial->error=1;
        sprintf(msg,"serial disconnected");
    }]=])
# Bound caster-provided status lines and detect the actually usable buffer
# capacity (one byte is reserved for the terminating NUL). Without the latter,
# a full unrecognized response leaves the stream permanently waiting.
gnss_receiver_replace(1 "        strcpy(msg,p);" "        snprintf(msg,MAXSTRMSG,\"%s\",p);")
gnss_receiver_replace(2 "ntrip->nb>=NTRIP_MAXRSP" "ntrip->nb>=NTRIP_MAXRSP-1")
set(output "${CMAKE_CURRENT_BINARY_DIR}/rtklib_patched/stream.c")
set(previous "")
if(EXISTS "${output}")
  file(READ "${output}" previous)
endif()
if(NOT previous STREQUAL _receiver_source)
  file(WRITE "${output}" "${_receiver_source}")
endif()
list(REMOVE_ITEM _core_sources "${input}")
list(APPEND _core_sources "${output}")
