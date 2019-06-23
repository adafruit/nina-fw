#ifndef AZUREIOTHUB_H
#define AZUREIOTHUB_H

namespace azureiothub {
    uint8_t init(const char* token);
    uint8_t post_message(const char* payload, size_t& messageTrackingId);
}

#endif
