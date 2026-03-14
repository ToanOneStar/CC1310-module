################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/Users/Dell/Downloads/ti-cgt-arm_20.2.7.LTS

GEN_OPTS__FLAG := --cmd_file="E:/datn/CC1310-module/tirtos_builds_CC1310_LAUNCHXL_release_ccs/Debug/configPkg/compiler.opt" 
GEN_CMDS__FLAG := -l"E:/datn/CC1310-module/tirtos_builds_CC1310_LAUNCHXL_release_ccs/Debug/configPkg/linker.cmd" 

ORDERED_OBJS += \
"./Application/main_tirtos.obj" \
"./Application/uartecho.obj" \
"./Drivers/src/IIC.obj" \
"./Drivers/src/MPU_6050.obj" \
"./Middleware/MPU6050/mpu6050.obj" \
"./Middleware/MPU6050/mpu6050_angles.obj" \
"./Startup/CC1310_LAUNCHXL.obj" \
"./Startup/CC1310_LAUNCHXL_fxns.obj" \
"./Startup/ccfg.obj" \
"../Startup/CC1310_LAUNCHXL_TIRTOS.cmd" \
-l"ti/display/lib/display.aem3" \
-l"ti/grlib/lib/ccs/m3/grlib.a" \
-l"third_party/spiffs/lib/ccs/m3/spiffs_cc26xx.a" \
-l"ti/drivers/rf/lib/rf_multiMode_cc13x0.aem3" \
-l"ti/drivers/lib/drivers_cc13x0.aem3" \
-l"ti/dpl/lib/dpl_cc13x0.aem3" \
$(GEN_CMDS__FLAG) \
-l"ti/devices/cc13x0/driverlib/bin/ccs/driverlib.lib" \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include Application/subdir_vars.mk
-include Drivers/src/subdir_vars.mk
-include Middleware/MPU6050/subdir_vars.mk
-include Startup/subdir_vars.mk
-include Application/subdir_rules.mk
-include Drivers/src/subdir_rules.mk
-include Middleware/MPU6050/subdir_rules.mk
-include Startup/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
uartecho.out 

EXE_OUTPUTS__QUOTED += \
"uartecho.out" 


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "uartecho.out"

dependents:
	-cd E:/datn/CC1310-module/tirtos_builds_CC1310_LAUNCHXL_release_ccs/Debug && $(MAKE) all

# Tool invocations
uartecho.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS) E:/datn/CC1310-module/tirtos_builds_CC1310_LAUNCHXL_release_ccs/Debug/tirtos_builds_CC1310_LAUNCHXL_release_ccs.lib
	@echo 'Building target: "$@"'
	@echo 'Invoking: Arm Linker'
	"C:/Users/Dell/Downloads/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --define=DeviceFamily_CC13X0 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on -z -m"uartecho.map" -i"C:/TI/simplelink_cc13x0_sdk_4_20_02_07/source" -i"C:/TI/simplelink_cc13x0_sdk_4_20_02_07/kernel/tirtos/packages" -i"C:/Users/Dell/Downloads/ti-cgt-arm_20.2.7.LTS/lib" --diag_wrap=off --display_error_number --warn_sections --xml_link_info="uartecho_linkInfo.xml" --rom_model -o "uartecho.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "Application\main_tirtos.obj" "Application\uartecho.obj" "Drivers\src\IIC.obj" "Drivers\src\MPU_6050.obj" "Middleware\MPU6050\mpu6050.obj" "Middleware\MPU6050\mpu6050_angles.obj" "Startup\CC1310_LAUNCHXL.obj" "Startup\CC1310_LAUNCHXL_fxns.obj" "Startup\ccfg.obj" 
	-$(RM) "Application\main_tirtos.d" "Application\uartecho.d" "Drivers\src\IIC.d" "Drivers\src\MPU_6050.d" "Middleware\MPU6050\mpu6050.d" "Middleware\MPU6050\mpu6050_angles.d" "Startup\CC1310_LAUNCHXL.d" "Startup\CC1310_LAUNCHXL_fxns.d" "Startup\ccfg.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:
E:/datn/CC1310-module/tirtos_builds_CC1310_LAUNCHXL_release_ccs/Debug/tirtos_builds_CC1310_LAUNCHXL_release_ccs.lib:

-include ../makefile.targets

