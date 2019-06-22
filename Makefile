PROJECT_NAME := nina-fw

EXTRA_COMPONENT_DIRS := $(PWD)/arduino
EXTRA_COMPONENT_DIRS += $(PWD)/../esp-azure
EXTRA_COMPONENT_DIRS += ${PWD}/../esp-azure/port")

ifeq ($(RELEASE),1)
CFLAGS += -DNDEBUG -DCONFIG_FREERTOS_ASSERT_DISABLE -Os -DLOG_LOCAL_LEVEL=0
CPPFLAGS += -DNDEBUG -Os
endif

ifeq ($(UNO_WIFI_REV2),1)
CFLAGS += -DUNO_WIFI_REV2
CPPFLAGS += -DUNO_WIFI_REV2
endif

include $(IDF_PATH)/make/project.mk

firmware: all
	python combine.py

.PHONY: firmware
