################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../IFaces/XML/tinystr.cpp \
../IFaces/XML/tinyxml.cpp \
../IFaces/XML/tinyxmlerror.cpp \
../IFaces/XML/tinyxmlparser.cpp 

OBJS += \
./IFaces/XML/tinystr.o \
./IFaces/XML/tinyxml.o \
./IFaces/XML/tinyxmlerror.o \
./IFaces/XML/tinyxmlparser.o 

CPP_DEPS += \
./IFaces/XML/tinystr.d \
./IFaces/XML/tinyxml.d \
./IFaces/XML/tinyxmlerror.d \
./IFaces/XML/tinyxmlparser.d 


# Each subdirectory must supply rules for building sources it contributes
IFaces/XML/%.o: ../IFaces/XML/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include/lua5.1 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


