PROJECT_NAME := ble_app_cscs_s110_uart_pca10028

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

TEMPLATE_PATH = ../../../../../../components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc"
AS       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as"
AR       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar" -r
LD       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld"
NM       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm"
OBJDUMP  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump"
OBJCOPY  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy"
SIZE    		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size"

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
../../../../../../components/serialization/application/codecs/s110/serializers/app_ble_gap_sec_keys.c \
../../../../../../components/serialization/application/codecs/s110/serializers/app_ble_user_mem.c \
../../../../../../components/serialization/application/transport/app_mailbox.c \
../../../../../../components/serialization/application/codecs/s110/middleware/app_mw_ble.c \
../../../../../../components/serialization/application/codecs/s110/middleware/app_mw_ble_gap.c \
../../../../../../components/serialization/application/codecs/s110/middleware/app_mw_ble_gattc.c \
../../../../../../components/serialization/application/codecs/s110/middleware/app_mw_ble_gatts.c \
../../../../../../components/serialization/application/codecs/s110/middleware/app_mw_ble_l2cap.c \
../../../../../../components/serialization/application/codecs/s110/middleware/app_mw_nrf_soc.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_enable.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_event.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_evt_tx_complete.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_evt_user_mem_release.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_evt_user_mem_request.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_address_get.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_address_set.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_adv_data_set.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_adv_start.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_adv_stop.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_appearance_get.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_appearance_set.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_auth_key_reply.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_authenticate.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_conn_param_update.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_conn_sec_get.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_connect.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_connect_cancel.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_device_name_get.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_device_name_set.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_disconnect.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_encrypt.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_evt_adv_report.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_evt_auth_key_request.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_evt_auth_status.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_evt_conn_param_update.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_evt_conn_param_update_request.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_evt_conn_sec_update.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_evt_connected.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_evt_disconnected.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_evt_passkey_display.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_evt_rssi_changed.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_evt_scan_req_report.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_evt_sec_info_request.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_evt_sec_params_request.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_evt_sec_request.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_evt_timeout.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_ppcp_get.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_ppcp_set.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_rssi_get.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_rssi_start.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_rssi_stop.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_scan_start.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_scan_stop.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_sec_info_reply.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_sec_params_reply.c \
../../../../../../components/serialization/common/struct_ser/s110/ble_gap_struct_serialization.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gap_tx_power_set.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_char_value_by_uuid_read.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_char_values_read.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_characteristics_discover.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_descriptors_discover.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_evt_char_disc_rsp.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_evt_char_val_by_uuid_read_rsp.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_evt_char_vals_read_rsp.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_evt_desc_disc_rsp.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_evt_hvx.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_evt_prim_srvc_disc_rsp.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_evt_read_rsp.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_evt_rel_disc_rsp.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_evt_timeout.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_evt_write_rsp.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_hv_confirm.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_primary_services_discover.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_read.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_relationships_discover.c \
../../../../../../components/serialization/common/struct_ser/s110/ble_gattc_struct_serialization.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gattc_write.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gatts_characteristic_add.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gatts_descriptor_add.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gatts_evt_hvc.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gatts_evt_rw_authorize_request.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gatts_evt_sc_confirm.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gatts_evt_sys_attr_missing.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gatts_evt_timeout.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gatts_evt_write.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gatts_hvx.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gatts_include_add.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gatts_rw_authorize_reply.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gatts_service_add.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gatts_service_changed.c \
../../../../../../components/serialization/common/struct_ser/s110/ble_gatts_struct_serialization.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gatts_sys_attr_get.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gatts_sys_attr_set.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gatts_value_get.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_gatts_value_set.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_l2cap_cid_register.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_l2cap_cid_unregister.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_l2cap_evt_rx.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_l2cap_tx.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_opt_get.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_opt_set.c \
../../../../../../components/serialization/common/ble_serialization.c \
../../../../../../components/serialization/common/struct_ser/s110/ble_struct_serialization.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_tx_buffer_count_get.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_user_mem_reply.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_uuid_decode.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_uuid_encode.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_uuid_vs_add.c \
../../../../../../components/serialization/application/codecs/s110/serializers/ble_version_get.c \
../../../../../../components/serialization/common/cond_field_serialization.c \
../../../../../../components/serialization/application/codecs/s110/serializers/power_system_off.c \
../../../../../../components/serialization/application/hal/ser_app_hal_nrf51.c \
../../../../../../components/serialization/application/hal/ser_app_power_system_off.c \
../../../../../../components/serialization/common/transport/ser_hal_transport.c \
../../../../../../components/serialization/common/transport/ser_phy/ser_phy_nrf51_uart.c \
../../../../../../components/serialization/application/transport/ser_sd_transport.c \
../../../../../../components/serialization/application/transport/ser_softdevice_handler.c \
../../../../../../components/serialization/application/codecs/s110/serializers/temp_get.c \
../../../../../../components/libraries/button/app_button.c \
../../../../../../components/libraries/util/app_error.c \
../../../../../../components/libraries/scheduler/app_scheduler.c \
../../../../../../components/libraries/timer/app_timer.c \
../../../../../../components/libraries/util/app_util_platform.c \
../../../../../../components/libraries/util/nrf_assert.c \
../../../../../../components/libraries/sensorsim/sensorsim.c \
../../../../../../components/drivers_nrf/uart/app_uart.c \
../../../../../../components/drivers_nrf/ble_flash/ble_flash.c \
../../../../../../components/drivers_nrf/hal/nrf_delay.c \
../../../../../../components/drivers_nrf/common/nrf_drv_common.c \
../../../../../../components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
../../../../../../components/drivers_nrf/nrf_soc_nosd/nrf_soc.c \
../../../../../../components/drivers_nrf/pstorage/pstorage_nosd.c \
../../../../../bsp/bsp.c \
../../../../../bsp/bsp_btn_ble.c \
../../../main.c \
../../../../../../components/ble/common/ble_advdata.c \
../../../../../../components/ble/ble_advertising/ble_advertising.c \
../../../../../../components/ble/ble_services/ble_bas/ble_bas.c \
../../../../../../components/ble/common/ble_conn_params.c \
../../../../../../components/ble/ble_services/ble_cscs/ble_cscs.c \
../../../../../../components/ble/ble_services/ble_dis/ble_dis.c \
../../../../../../components/ble/ble_services/ble_cscs/ble_sc_ctrlpt.c \
../../../../../../components/ble/common/ble_srv_common.c \
../../../../../../components/ble/device_manager/device_manager_peripheral.c \
../../../../../../components/toolchain/system_nrf51.c \
../../../../../../components/softdevice/common/softdevice_handler/softdevice_handler.c \

#assembly files common to all targets
ASM_SOURCE_FILES  = ../../../../../../components/toolchain/gcc/gcc_startup_nrf51.s

#includes common to all targets
INC_PATHS  = -I../../../config
INC_PATHS += -I../../../../../../components/serialization/common/transport/ser_phy/config
INC_PATHS += -I../../../../../../components/drivers_nrf/config
INC_PATHS += -I../../../../../bsp
INC_PATHS += -I../../../../../../components/serialization/common
INC_PATHS += -I../../../../../../components/libraries/util
INC_PATHS += -I../../../../../../components/drivers_nrf/pstorage
INC_PATHS += -I../../../../../../components/drivers_nrf/uart
INC_PATHS += -I../../../../../../components/ble/common
INC_PATHS += -I../../../../../../components/libraries/sensorsim
INC_PATHS += -I../../../../../../components/libraries/button
INC_PATHS += -I../../../../../../components/ble/device_manager
INC_PATHS += -I../../../../../../components/ble/ble_services/ble_cscs
INC_PATHS += -I../../../../../../components/ble/ble_services/ble_dis
INC_PATHS += -I../../../../../../components/device
INC_PATHS += -I../../../../../../components/ble/ble_services/ble_bas
INC_PATHS += -I../../../../../../components/serialization/application/codecs/s110/serializers
INC_PATHS += -I../../../../../../components/drivers_nrf/ble_flash
INC_PATHS += -I../../../../../../components/libraries/timer
INC_PATHS += -I../../../../../../components/serialization/common/transport/ser_phy
INC_PATHS += -I../../../../../../components/softdevice/s110/headers
INC_PATHS += -I../../../../../../components/drivers_nrf/gpiote
INC_PATHS += -I../../../../../../components/libraries/scheduler
INC_PATHS += -I../../../../../../components/softdevice/common/softdevice_handler
INC_PATHS += -I../../../../../../components/drivers_nrf/hal
INC_PATHS += -I../../../../../../components/toolchain/gcc
INC_PATHS += -I../../../../../../components/toolchain
INC_PATHS += -I../../../../../../components/drivers_nrf/common
INC_PATHS += -I../../../../../../components/ble/ble_advertising
INC_PATHS += -I../../../../../../components/serialization/common/transport
INC_PATHS += -I../../../../../../components/libraries/trace
INC_PATHS += -I../../../../../../components/serialization/application/hal
INC_PATHS += -I../../../../../../components/serialization/common/struct_ser/s110
INC_PATHS += -I../../../../../../components/serialization/application/transport

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DBOARD_PCA10028
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DNRF51
CFLAGS += -DS110
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSVCALL_AS_NORMAL_FUNCTION
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -O3
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DBOARD_PCA10028
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DNRF51
ASMFLAGS += -DS110
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSVCALL_AS_NORMAL_FUNCTION
#default target - first one defined
default: clean nrf51422_xxac_s110

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51422_xxac_s110 

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf51422_xxac_s110


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf51422_xxac_s110: OUTPUT_FILENAME := nrf51422_xxac_s110
nrf51422_xxac_s110: LINKER_SCRIPT=ble_app_cscs_gcc_nrf51.ld
nrf51422_xxac_s110: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<


# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ""
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ""

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash: $(MAKECMDGOALS)
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.hex
	nrfjprog --reset --program $(OUTPUT_BINARY_DIRECTORY)/$<.hex

## Flash softdevice