#ifndef AZUREIOTHUB_H
#define AZUREIOTHUB_H

namespace azureiothub {
    uint8_t init();
    uint8_t post_message(const char* payload, size_t& messageTrackingId);
    uint8_t do_work();
}

#endif
