# modbus-rtu-stm32

# This is an implementation of modbus rtu protocol with stm32 microcontrollers

 The idea of this project is the implementation of modbus protocol in industrial serial communication (RS485) using 
 microcontrollers of the manufacturer STMicroelectronics (STM32F103C8) or any other. 
  
 The library is used to implement both units, slave and master. 

# src directory

1. The library used for implementation of modbus protocol (this is the last version and is copied from ModbusMaster project).
2. The STM32 project where the unit is configured as a slave (ModbusSlave).
3. The STM32 project where the unit is configured as a master (ModbusMaster).
  
# Issues

 In the library are implemented four function codes: read coils (1), read registers (3), write multiple coils (15) and write multiple registers(16).
 Implementation using the library of modbus slave unit works just fine.
 Implementation of modbus master using the library is not yet complited:
 
1. It is a problem with the implementation of the write multiple coils implemented from the master.
2. The response timeout of the master is not implemented (work to be complited).
        
        
# Implementation of the slave

1. Configure one UART module, two timers and two ports as digital outputs.
2. Declare a variable of the type structure MBHandler and two arrays, one of the type uint8_t and the other 
   type of uint16_t to represent the modbus data model of the coils and registers.
3. Call InitMBSlave function and then StartMBSlave.
4. Implement callback functions of the UART and TIMER module.
   (at this time the unit can be seen as modbus slave unit)
      
# Implementation of the master

1. Configure one UART module, two timers and two ports as digital outputs.
2. Declare a variable of the type structure MBHandler.
3. Call InitMBMaster function.
4. Implement callback functions of the UART and TIMER module (at this time the unit 
   can be seen as modbus master unit and by calling functions in the 
   library you can read or write data in slave unit).
   
# Hardware & tools

1. Two or more STM32 microcontrollers.
2. UART-RS485 converters.
3. Twisted pair of wires used as a serial bus.
3. Temperature sensors, LEDs, potenciometers etc for testing.
4. STM32CubeIDE for the development of the library and two samle projects. 
        
