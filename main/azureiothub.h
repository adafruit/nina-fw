#ifndef AZUREIOTHUB_H
#define AZUREIOTHUB_H

namespace azureiot {
    int init(void);
    int post_message(const char* payload);
}

#endif
