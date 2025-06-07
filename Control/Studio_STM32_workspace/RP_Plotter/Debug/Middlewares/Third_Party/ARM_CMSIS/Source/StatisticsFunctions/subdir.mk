################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctions.c \
../Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctionsF16.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctions.o \
./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctionsF16.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctions.d \
./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctionsF16.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/%.o Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/%.su Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/%.cyclo: ../Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/%.c Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"C:/Users/Last/Desktop/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/BasicMathFunctions" -I"C:/Users/Last/Desktop/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/BayesFunctions" -I"C:/Users/Last/Desktop/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/CommonTables" -I"C:/Users/Last/Desktop/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/ComplexMathFunctions" -I"C:/Users/Last/Desktop/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/ControllerFunctions" -I"C:/Users/Last/Desktop/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/DistanceFunctions" -I"C:/Users/Last/Desktop/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/FastMathFunctions" -I"C:/Users/Last/Desktop/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/FilteringFunctions" -I"C:/Users/Last/Desktop/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/InterpolationFunctions" -I"C:/Users/Last/Desktop/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/MatrixFunctions" -I"C:/Users/Last/Desktop/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/QuaternionMathFunctions" -I"C:/Users/Last/Desktop/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/StatisticsFunctions" -I"C:/Users/Last/Desktop/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/SupportFunctions" -I"C:/Users/Last/Desktop/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/SVMFunctions" -I"C:/Users/Last/Desktop/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/TransformFunctions" -I"C:/Users/Last/Desktop/RP-Plotter-G.9/Control/Studio_STM32_workspace/RP_Plotter/Source/WindowFunctions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-StatisticsFunctions

clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-StatisticsFunctions:
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctions.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctions.d ./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctions.o ./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctions.su ./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctionsF16.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctionsF16.d ./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctionsF16.o ./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctionsF16.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-StatisticsFunctions

