################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32g474retx.s 

OBJS += \
./Core/Startup/startup_stm32g474retx.o 

S_DEPS += \
./Core/Startup/startup_stm32g474retx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/College 2.2/Studio/Studio_repository/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/BasicMathFunctions" -I"C:/College 2.2/Studio/Studio_repository/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/BayesFunctions" -I"C:/College 2.2/Studio/Studio_repository/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/CommonTables" -I"C:/College 2.2/Studio/Studio_repository/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/ComplexMathFunctions" -I"C:/College 2.2/Studio/Studio_repository/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/ControllerFunctions" -I"C:/College 2.2/Studio/Studio_repository/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/DistanceFunctions" -I"C:/College 2.2/Studio/Studio_repository/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/FastMathFunctions" -I"C:/College 2.2/Studio/Studio_repository/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/FilteringFunctions" -I"C:/College 2.2/Studio/Studio_repository/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/InterpolationFunctions" -I"C:/College 2.2/Studio/Studio_repository/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/MatrixFunctions" -I"C:/College 2.2/Studio/Studio_repository/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/QuaternionMathFunctions" -I"C:/College 2.2/Studio/Studio_repository/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/StatisticsFunctions" -I"C:/College 2.2/Studio/Studio_repository/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/SupportFunctions" -I"C:/College 2.2/Studio/Studio_repository/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/SVMFunctions" -I"C:/College 2.2/Studio/Studio_repository/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/TransformFunctions" -I"C:/College 2.2/Studio/Studio_repository/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup

