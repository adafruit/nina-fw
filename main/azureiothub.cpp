// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <stdio.h>
#include <stdlib.h>

#include "iothub_client.h"
#include "iothub_device_client_ll.h"
#include "iothub_client_options.h"
#include "iothub_message.h"
#include "azure_c_shared_utility/threadapi.h"
#include "azure_c_shared_utility/crt_abstractions.h"
#include "azure_c_shared_utility/platform.h"
#include "azure_c_shared_utility/shared_util_options.h"
#include "iothubtransportmqtt.h"
#include "iothub_client_options.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "azureiothub.h"


#ifdef MBED_BUILD_TIMESTAMP
    #define SET_TRUSTED_CERT_IN_SAMPLES
#endif // MBED_BUILD_TIMESTAMP

#ifdef SET_TRUSTED_CERT_IN_SAMPLES
    #include "certs.h"
#endif // SET_TRUSTED_CERT_IN_SAMPLES

namespace azureiothub {

/*String containing Hostname, Device Id & Device Key in the format:                         */
/*  "HostName=<host_name>;DeviceId=<device_id>;SharedAccessKey=<device_key>"                */
/*  "HostName=<host_name>;DeviceId=<device_id>;SharedAccessSignature=<device_sas_token>"    */
static IOTHUB_CLIENT_LL_HANDLE iotHubClientHandle = NULL;
static int iotHubReceiveContext = 0;
static size_t nextMessageTrackingId = 0;

typedef struct EVENT_INSTANCE_TAG
{
    IOTHUB_MESSAGE_HANDLE messageHandle;
    size_t messageTrackingId;  // For tracking the messages within the user callback.
} EVENT_INSTANCE;

static IOTHUBMESSAGE_DISPOSITION_RESULT ReceiveMessageCallback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback)
{
    int* counter = (int*)userContextCallback;
    const char* buffer;
    size_t size;
    const char* messageId;
    const char* correlationId;

    // Message properties
    if ((messageId = IoTHubMessage_GetMessageId(message)) == NULL)
    {
        messageId = "<null>";
    }

    if ((correlationId = IoTHubMessage_GetCorrelationId(message)) == NULL)
    {
        correlationId = "<null>";
    }

    // Message content
    if (IoTHubMessage_GetByteArray(message, (const unsigned char**)&buffer, &size) != IOTHUB_MESSAGE_OK)
    {
        (void)printf("unable to retrieve the message data\r\n");
        return IOTHUBMESSAGE_REJECTED;
    }

    /* Some device specific action code goes here... */
    (*counter)++;
    return IOTHUBMESSAGE_ACCEPTED;
}

static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback)
{
    EVENT_INSTANCE* eventInstance = (EVENT_INSTANCE*)userContextCallback;
    size_t id = eventInstance->messageTrackingId;

/*
    IOTHUB_CLIENT_CONFIRMATION_OK
    IOTHUB_CLIENT_CONFIRMATION_BECAUSE_DESTROY
    IOTHUB_CLIENT_CONFIRMATION_MESSAGE_TIMEOUT
    IOTHUB_CLIENT_CONFIRMATION_ERROR
 */
    if (result == IOTHUB_CLIENT_CONFIRMATION_OK) {
        (void)printf("Confirmation received for message tracking id = %d with result = %s\r\n", (int)id, ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));
        /* Some device specific action code goes here... */
        //callbackCounter++;
    }
    IoTHubMessage_Destroy(eventInstance->messageHandle);
}

void connection_status_callback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* userContextCallback)
{
    (void)printf("\n\nConnection Status result:%s, Connection Status reason: %s\n\n", ENUM_TO_STRING(IOTHUB_CLIENT_CONNECTION_STATUS, result),
                 ENUM_TO_STRING(IOTHUB_CLIENT_CONNECTION_STATUS_REASON, reason));
}

uint8_t init(const char* connectionString)
{
    if (iotHubReceiveContext != 0) {
        // already initialized
        return 0;
    }

#ifdef CONFIG_IOTHUB_CONNECTION_STRING
    if (NULL == connectionString) {
        connectionString = CONFIG_IOTHUB_CONNECTION_STRING;
    }    
#endif
    if (NULL == connectionString) {
        (void)printf("missing configuration string\r\n");
        return 1;
    }

    int res = platform_init();
    if (res != 0)
    {
        (void)printf("Failed to initialize the platform %d.\r\n", res);
        return 2;
    }

    if ((iotHubClientHandle = IoTHubClient_LL_CreateFromConnectionString(connectionString, MQTT_Protocol)) == NULL)
    {
        (void)printf("ERROR: iotHubClientHandle is NULL!\r\n");
        return 3;
    }
    
    bool traceOn = true;
    IoTHubClient_LL_SetOption(iotHubClientHandle, OPTION_LOG_TRACE, &traceOn);

    IoTHubClient_LL_SetConnectionStatusCallback(iotHubClientHandle, connection_status_callback, NULL);
    // Setting the Trusted Certificate.  This is only necessary on system with without
    // built in certificate stores.
#ifdef SET_TRUSTED_CERT_IN_SAMPLES
    IoTHubDeviceClient_LL_SetOption(iotHubClientHandle, OPTION_TRUSTED_CERT, certificates);
#endif // SET_TRUSTED_CERT_IN_SAMPLES

    /* Setting Message call back, so we can receive Commands. */
    if (IoTHubClient_LL_SetMessageCallback(iotHubClientHandle, ReceiveMessageCallback, &iotHubReceiveContext) != IOTHUB_CLIENT_OK)
    {
        (void)printf("ERROR: IoTHubClient_LL_SetMessageCallback..........FAILED!\r\n");
        return 4;
    }

    (void)printf("IoTHubClient_LL_SetMessageCallback...successful.\r\n");

    return 0;
}

uint8_t do_work() {
    if (iotHubReceiveContext == 0) {
        // not initialized yet
        return 0;
    }

    IoTHubClient_LL_DoWork(iotHubClientHandle);
    return 0;
}

uint8_t post_message(const char* msgText, size_t& messageTrackingId) 
{
    EVENT_INSTANCE message;
    messageTrackingId = 0;
    message.messageHandle = IoTHubMessage_CreateFromByteArray((const unsigned char*)msgText, strlen(msgText));
    if (message.messageHandle == NULL)
    {
        (void)printf("ERROR: iotHubMessageHandle is NULL!\r\n");
        return 4;
    }

    message.messageTrackingId = ++nextMessageTrackingId;
    if (IoTHubClient_LL_SendEventAsync(iotHubClientHandle, message.messageHandle, SendConfirmationCallback, &message) != IOTHUB_CLIENT_OK)
    {
        (void)printf("ERROR: IoTHubClient_LL_SendEventAsync..........FAILED!\r\n");
        return 5;
    }
    (void)printf("IoTHubClient_LL_SendEventAsync accepted message [%d] for transmission to IoT Hub.\r\n", message.messageTrackingId);

    messageTrackingId = message.messageTrackingId;
    return 0;
}

}