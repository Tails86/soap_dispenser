################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: MSP430 Compiler'
	"D:/ti/ccs1011/ccs/tools/compiler/ti-cgt-msp430_20.2.1.LTS/bin/cl430" -vmsp -O3 --opt_for_speed=4 --use_hw_mpy=none --include_path="D:/ti/ccs1011/ccs/ccs_base/msp430/include" --include_path="D:/Users/jmsmi/OneDrive/Documents/Code Composer Studio/workspace_v10/Soap Dispenser" --include_path="D:/ti/ccs1011/ccs/tools/compiler/ti-cgt-msp430_20.2.1.LTS/include" --advice:power=all --define=__MSP430G2452__ -g --printf_support=minimal --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: MSP430 Compiler'
	"D:/ti/ccs1011/ccs/tools/compiler/ti-cgt-msp430_20.2.1.LTS/bin/cl430" -vmsp -O3 --opt_for_speed=4 --use_hw_mpy=none --include_path="D:/ti/ccs1011/ccs/ccs_base/msp430/include" --include_path="D:/Users/jmsmi/OneDrive/Documents/Code Composer Studio/workspace_v10/Soap Dispenser" --include_path="D:/ti/ccs1011/ccs/tools/compiler/ti-cgt-msp430_20.2.1.LTS/include" --advice:power=all --define=__MSP430G2452__ -g --printf_support=minimal --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


