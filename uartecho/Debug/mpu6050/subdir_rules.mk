################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
mpu6050/%.obj: ../mpu6050/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/Users/Dell/Downloads/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/Dell/workspace_ccstheia/uartecho" --include_path="C:/TI/simplelink_cc13x0_sdk_4_20_02_07/source/ti/posix/ccs" --include_path="C:/Users/Dell/Downloads/ti-cgt-arm_20.2.7.LTS/include" --define=DeviceFamily_CC13X0 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="mpu6050/$(basename $(<F)).d_raw" --obj_directory="mpu6050" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


