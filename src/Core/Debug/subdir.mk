################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core.cpp \
../EventManager.cpp \
../Main.cpp \
../ScriptSystem.cpp \
../ZazenGameObject.cpp \
../ZazenGameObjectFactory.cpp \
../ZazenSubSystemFactory.cpp 

OBJS += \
./Core.o \
./EventManager.o \
./Main.o \
./ScriptSystem.o \
./ZazenGameObject.o \
./ZazenGameObjectFactory.o \
./ZazenSubSystemFactory.o 

CPP_DEPS += \
./Core.d \
./EventManager.d \
./Main.d \
./ScriptSystem.d \
./ZazenGameObject.d \
./ZazenGameObjectFactory.d \
./ZazenSubSystemFactory.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include/lua5.1 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


