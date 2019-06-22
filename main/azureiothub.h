#ifndef AZUREIOTHUB_H
#define AZUREIOTHUB_H

namespace azureiot {
    uint8_t init();
    uint8_t post_message(const char* payload);
    uint8_t do_work();
}

#endif
