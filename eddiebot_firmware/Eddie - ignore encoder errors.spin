Con

  _CLKMODE = XTAL1 + PLL16X
  _CLKFREQ = 80_000_000

  ' Settings
  BUFFER_LENGTH = 255           ' Input Buffer Length must fit within input/output 'Index' ranges (currently a byte)
  VERSION       = 112           ' Added 100 for this modified version (Tang) 

  ' Positions one wheel must make for a 360° rotation
  POSITIONS_PER_ROTATION = 186

  ' Position Sensor Constants
  #0 ' Identifiers   
  ALL_ENCODERS
  LEFT_ENCODER                                                                                                     
  RIGHT_ENCODER
  #1 ' Commands
  QUERY_POSITION
  QUERY_SPEED
  CHECK_FOR_ARRIVAL
  TRAVEL_NUMBER_OF_POSITIONS
  CLEAR_POSITION
  SET_ORIENTATION_AS_REVERSED
  SET_TRANSMIT_DELAY
  SET_SPEED_MAXIMUM
  SET_SPEED_RAMP_RATE

  ' Motor names
  #0
  LEFT_MOTOR
  RIGHT_MOTOR

  ' Motor control modes
  #0
  POWER
  VELOCITY
  STOPPING
  POSITION
  'ARC_POSITION

  ' Pin assignments
  ' Ping))) sensors
  PING_0        = 0
  PING_1        = 1
  PING_2        = 2
  PING_3        = 3
  ' Encoders
  ENCODERS_PIN  = 10
  ' Solid-state Relays
  SSR_A         = 16
  SSR_B         = 17
  SSR_C         = 18
  ' Motors
  MOTOR_L_B     = 19
  MOTOR_L_PWM   = 20
  MOTOR_L_A     = 21
  MOTOR_R_B     = 22
  MOTOR_R_PWM   = 23
  MOTOR_R_A     = 24
  ' ADC
  ADC_CS        = 25
  ADC_DIO       = 26
  ADC_CLK       = 27
  ' I2C
  SCL           = 28
  SDA           = 29
  ' RS-232 TTL
  TX            = 30 
  RX            = 31 

  ' Master GPIO mask (Only high pins can be set as outputs)
  OUTPUTABLE    = %00000000_00000111_11111111_11111111
  PINGABLE      = %00000000_00000000_11111111_11111111
  INITIAL_GPIO  = |< SSR_A | |< SSR_B | |< SSR_C
  INITIAL_PING  = |< PING_0 | |< PING_1 | |< PING_2 | |< PING_3

  ' Terminal Settings
  BAUDMODE      = %0000
  BAUDRATE      = 115_200
  TIMEOUT       = 10

  ' ASCII commands
  NUL           = $00           ' Null character
  SOH           = $01           ' Start of Header
  STX           = $02           ' Start of Text
  ETX           = $03           ' End of Text
  EOT           = $04           ' End of Transmission
  ENQ           = $05           ' Enquiry
  ACK           = $06           ' Acknowledgment
  BEL           = $07           ' Bell
  BS            = $08           ' Backspace
  HT            = $09           ' Horizontal Tab
  LF            = $0A           ' Line feed
  VT            = $0B           ' Vertical Tab
  FF            = $0C           ' Form feed
  CR            = $0D           ' Carriage return
  SO            = $0E           ' Shift Out
  SI            = $0F           ' Shift In
  DLE           = $10           ' Data Link Escape
  DC1           = $11           ' Device Control 1 (i.e. XON)
  DC2           = $12           ' Device Control 2
  DC3           = $13           ' Device Control 3 (i.e. XOFF)
  DC4           = $14           ' Device Control 4
  NAK           = $15           ' Negative Acknowledgment
  SYN           = $16           ' Synchronous idle
  ETB           = $17           ' End of Transmission Block
  CAN           = $18           ' Cancel
  EM            = $19           ' End of Medium
  SB            = $1A           ' Substitute
  ESC           = $1B           ' Escape
  FS            = $1C           ' File Separator
  GS            = $1D           ' Group Separator
  RS            = $1E           ' Record Separator
  US            = $1F           ' Unit Separator
  DEL           = $7F           ' Delete

  ' EEPROM constants
  I2C_ACK       = 0
  I2C_NACK      = 1
  DEVICE_CODE   = %0110 << 4
  PAGE_SIZE     = 128

  ' Other
  DIVISOR       = 50
  DEADZONE      = 1
  PROMPT_SUM    = $0

  ' Tang  
  ' Kinect pan servo parameters
  Pin_ServoPan  = 4
  Pan_FullLeft  = 600

           
Obj                             
                                
  Term          : "FullDuplexSerial"
  Encoders      : "FullDuplexSerial"
  ADC           : "MCP3208.spin"                        ' Works with MCP3008, but two LSBs may be incorrect
  Ping          : "ReadPulseWidths.spin"
  Motors        : "Eddie Motor Driver"
  Servos        : "PWM_32_v4"                           ' Added for Kinect Pan servo (Tang)


Var

  long Error                                            ' Error message pointer for verbose responses                        
  long MotorSpeed[2]                                    ' Current motor speeds reported from position controllers  
  long Stack[50]                                        ' Stack for cog running position control system


Dat

GPIOMask      long      INITIAL_GPIO                    ' Pins currently used as GPIO pins
PingMask      long      INITIAL_PING                    ' Pins currently running PING))) Sensors
PingResults   long      INITIAL_PING, 0[31]             ' Results from cog recording PING))) sensor output pulses
MotorPosition long      0[2]                            ' Physical position of the motor as returned by encoder
MotPosOffset  long      0[2]                            ' Offset between Physical and internal motor position
MotorPower    long      0[2]                            ' Current motor power level
MidPosition   long      0[2]                            ' Position the PD loop is attempting to hold
MidPosAcc     long      0[2]                            '  fractional part
MidVelocity   long      0[2]                            ' Current position change velocity
MidVelAcc     long      0[2]                            '  fractional part
SetPosition   long      0[2]                            ' Position that MidPosition is approaching
SetVelocity   long      0[2]                            ' Velocity that MidVelocity is approaching
SetPower      long      0[2]                            ' Power level that MotorPower is approaching
MaxPowAccel   long      Motors#MAX_ON_TIME / 16         ' Maximum allowed motor power acceleration
MaxPosAccel   long      200                             ' Maximum allowed positional acceleration
Decel         long      0[2]                            ' Deceleration rate
Kp            long      Motors#MAX_ON_TIME / 16         ' Proportional error feedback gain
StillCnt      byte      0[2]                            ' Number of iterations in a row that the motor hasn't moved noticeably
Mode          byte      0                               ' Current mode of the control system

InputBuffer   byte      0[BUFFER_LENGTH]                ' Command input buffer
OutputBuffer  byte      0[BUFFER_LENGTH]                ' Response output buffer
InputIndex    byte      0                               ' Next empty position in the command input buffer
ParseIndex    byte      0                               ' Last processed position in the command input buffer 
OutputIndex   byte      0                               ' Next empty position in the response output buffer
Verbose       byte      0                               ' Verbosity level (Currently nonzero = verbose)
PDRunning     byte      0                               ' State of position control cog


Pub Main

  Term.Start(RX, TX, BAUDMODE, BAUDRATE)                ' Start a UART for the command port
  ADC.Start(ADC_DIO, ADC_CLK, ADC_CS, $FF)              ' Continuously run ADC conversions
  Ping.Start(@PingResults)                              ' Continuously read pulse widths on PING))) pins 
  cognew(PDLoop, @Stack)                                ' Run the position controller in another core

  ' Tang  
  ' Initialize PWM for Kinect pan servo
  Servos.Start
  SetPanPosition(90)

  repeat                                                ' Main loop (repeats forever)
    InputBuffer[InputIndex++] := Term.Rx                ' Read a byte from the command UART 
    if InputIndex == constant(BUFFER_LENGTH)            ' Check for a full buffer
      OutputStr(@Nack)                                  ' Ready error response
      if Verbose                                        ' If in verbose mode, add a description
        OutputStr(@Overflow)
      repeat                                            ' Ignore all inputs other than NUL or CR (terminating a command) 
        case Term.Rx                                    ' Send the correct error response for the transmission mode
          NUL :                                         '   Checksum mode
            SendChecksumResponse
            quit
          CR :                                          '   Plain text mode
            SendResponse
            quit
    else                                                ' If there isn't a buffer overflow...                              
      case InputBuffer[InputIndex - 1]                  ' Parse the character
        NUL :                                           ' End command in checksum mode:
          if InputIndex > 1                             '   Only parse buffer if it has content
            ifnot Error := \ChecksumParse               '   Run the parser and trap and report errors
              OutputIndex~                              '    Handle errors, if they occurred
              OutputStr(@Nack)
              if Verbose
                OutputStr(Error)
            SendChecksumResponse                        '   Send a response if no error
          else                                          '   For an empty buffer, clear the pointer
            InputIndex~                                 '   to start receiving a new command

        BS :                                            ' Process backspaces
          if --InputIndex                               ' Ignore the BS character itself
            --InputIndex                                ' Ignore previous character if exists

        CR:                                             ' End command in plaintext mode:
          if InputIndex > 1                             '   Only parse buffer if it has content
            if Error := \Parse                          '   Run the parser and trap and report errors
              OutputIndex~                              '    Handle errors, if they occurred
              OutputStr(@Nack)
              if Verbose
                OutputStr(Error)
            SendResponse                                '   Send a response if no error
          else                                          '   For an empty buffer, clear the pointer
            InputIndex~                                 '   to start receiving a new command

        SOH..BEL, LF..FF, SO..US, 127..255 :            ' Ignore invalid characters


PRI SetPanPosition(Horz) | width, units
{{
  Tang  
  Sets the servo position of the Kinect.
  Pos - 1 to 180 degrees
}}
  width := (Horz * 10) + Pan_FullLeft
  Servos.Servo(Pin_ServoPan, width)

PRI SetServoPosition(Pin,Horz) | width, units
{{
  Tang  
  Sets the servo position.
  Pos - 1 to 180 degrees
}}
  width := (Horz * 10) + Pan_FullLeft
  Servos.Servo(Pin, width)     
    

Pri Parse | Index, Difference, Parameter[3]             '' Parse the command in the input buffer
  
  InputBuffer[InputIndex - 1]~                          ' Ensure buffer is NUL terminated (may have been CR terminated)

  repeat ParseIndex from 0 to InputIndex - 2            ' Find the end of the command in the input buffer             
    case InputBuffer[ParseIndex]                         
      "a".."z" :                                        ' Set all command characters to uppercase
        InputBuffer[ParseIndex] -= constant("a" - "A")
      HT, " " :                                         ' Determine command length by finding the first whitespace character
        InputBuffer[ParseIndex]~                        ' Null terminate the command
        quit                                            ' ParseIndex now points to the null at the end of the command string (before the first parameter, if present)

  ' Parameter[n] := ParseHex(NextParameter) caches the parameters to check for their existence before running the command
  ' CheckLastParameter checks for too many parameters
  ' The Output...() methods write responses to the output buffer
  ' Check command against the following strings:
  if     strcomp(@InputBuffer, string("HWVER"))         ' Command: Respond with hardware version number
    CheckLastParameter
    ReadEEPROM(@Parameter, @Parameter + 1, 65534)       ' Read the hardware version from upper EEPROM
    OutputHex(Parameter, 4)
  elseif strcomp(@InputBuffer, string("VER"))           ' Command: Respond with version number
    CheckLastParameter
    OutputHex(VERSION, 4)
  elseif strcomp(@InputBuffer, string("OUT"))           ' Command: Set GPIO pins in mask as outputs
    Parameter := ParseHex(NextParameter)
    CheckLastParameter
    dira |= (Parameter & GPIOMask)                      '   Change only GPIO pins in mask to outputs
  elseif strcomp(@InputBuffer, string("IN"))            ' Command: Set GPIO pins in mask as inputs
    Parameter := ParseHex(NextParameter)
    CheckLastParameter
    dira &= !(Parameter & GPIOMask)                     '   Change only GPIO pins in mask to inputs
  elseif strcomp(@InputBuffer, string("HIGH"))          ' Command: Set GPIO pins in mask high
    Parameter := ParseHex(NextParameter)
    CheckLastParameter
    outa |= Parameter & GPIOMask                        '   Change only GPIO pins in mask high
  elseif strcomp(@InputBuffer, string("LOW"))           ' Command: Set GPIO pins in mask low
    Parameter := ParseHex(NextParameter)
    CheckLastParameter
    outa &= !(Parameter & GPIOMask)                     '   Change only GPIO pins in mask low
  elseif strcomp(@InputBuffer, string("OUTS"))          ' Command: Set GPIO pins in mask as outputs
    CheckLastParameter
    OutputHex(dira & GPIOMask, 8)
  elseif strcomp(@InputBuffer, string("INS"))           ' Command: return list of GPIO inputs
    CheckLastParameter
    OutputHex(!dira & GPIOMask, 8)
  elseif strcomp(@InputBuffer, string("HIGHS"))         ' Command: return list of high GPIO pins (may be inputs)
    CheckLastParameter
    OutputHex(outa & GPIOMask, 8)
  elseif strcomp(@InputBuffer, string("LOWS"))          ' Command: return list of low GPIO pins (may be inputs)
    CheckLastParameter
    OutputHex(!outa & GPIOMask, 8)
  elseif strcomp(@InputBuffer, string("READ"))          ' Command: return state of GPIO pins
    CheckLastParameter
    OutputHex(ina & GPIOMask, 8)
  elseif strcomp(@InputBuffer, string("ADC"))           ' Command: Respond with voltages from ADC
    CheckLastParameter
    OutputHex(ADC.In(0), 3)                             ' Show output from first ADC
    repeat Index from 1 to 7                            '  For the remaining 7
      OutputChr(" ")                                    '  Transmit a space character
      OutputHex(ADC.In(Index), 3)                       '  Then the output from the ADC
  elseif strcomp(@InputBuffer, string("PING"))          ' Command: Respond with status of active PING))) sensors
    CheckLastParameter
    ProcessPings                                        ' Processed in a separate method
  elseif strcomp(@InputBuffer, string("SGP"))           ' Command: Set pins as GPIO pins
    Parameter := ParseHex(NextParameter) & OUTPUTABLE   '  Ignore pins that shouldn't be set as GPIO pins
    CheckLastParameter
    if Parameter & PingMask                             '  If this affects any active PING))) sensors,
      ResetPingDriver(PingMask &= !Parameter)           '    remove them from the list of active PING))) sensors
    GPIOMask := Parameter                               '  Set the pins as GPIO pins
  elseif strcomp(@InputBuffer, string("SPNG"))          ' Command: Set pins as PING))) sensor pins
    Parameter := ParseHex(NextParameter) & PINGABLE     '   Ignore pins that shouldn't be set as PING))) sensor pins
    CheckLastParameter
    GPIOMask &= !Parameter                              '   Remove active PING))) sensor pins from GPIO mask
    outa &= !Parameter                                  '   Set active PING))) sensor pins as low inputs
    dira &= !Parameter                                                                   
    ResetPingDriver(Parameter)                          '   Restart the ReadPulseWidths object, with new PING))) sensor mask
  elseif strcomp(@InputBuffer, string("TRVL"))          ' Command: Travel a specified distance at a specified speed
    Parameter[0] := ParseHex(NextParameter)                                                                     
    Parameter[1] := ParseHex(NextParameter)
    CheckLastParameter
    ifnot PDRunning                                     ' Return an error if the position controller cog isn't running
      abort @EncoderError
    case Parameter[1]                                   ' Only accept a valid speed
      1..127:                                   
        InterpolateMidVariables
        SetVelocity[LEFT_MOTOR] := SetVelocity[RIGHT_MOTOR] := Parameter[1]
        SetPosition[LEFT_MOTOR] := MidPosition[LEFT_MOTOR] + ~~Parameter[0] ' Add travel distance to current position
        SetPosition[RIGHT_MOTOR] := MidPosition[RIGHT_MOTOR] + ~~Parameter[0]
        StillCnt~
        Mode := POSITION
      other:
        abort @InvalidParameter
  elseif strcomp(@InputBuffer, string("GOSPD"))         ' Command: Set the motors to a specified power
    Parameter[0] := ParseHex(NextParameter)
    Parameter[1] := ParseHex(NextParameter)
    CheckLastParameter
    ifnot PDRunning                                     '   Requires use of the PD controller
      abort @EncoderError                               '   Abort if it isn't running
    InterpolateMidVariables
    SetVelocity[LEFT_MOTOR] := ~~Parameter[0]           ' Then set the motors' powers
    SetVelocity[RIGHT_MOTOR] := ~~Parameter[1]
    StillCnt~
    Mode := VELOCITY
  else                                                  ' Only 16 elseifs, nest within an else for more
    if     strcomp(@InputBuffer, string("GO"))          ' Command: Set the motors to a specified power
      Parameter[0] := ParseHex(NextParameter)           '   Read both parameters before processing them
      Parameter[1] := ParseHex(NextParameter)           '   To prevent changes with a "Too few parameters" error
      CheckLastParameter
      ifnot PDRunning                                   '   Requires use of the PD controller
        abort @EncoderError                             '   Abort if it isn't running
      Mode := POWER
      SetPower[LEFT_MOTOR] := ~Parameter[0] * Motors#MAX_ON_TIME / 127' Then set the motors' powers
      SetPower[RIGHT_MOTOR] := ~Parameter[1] * Motors#MAX_ON_TIME / 127
    elseif strcomp(@InputBuffer, string("STOP"))        ' Command: Slow to a stop over a specified distance
      Parameter := ParseHex(NextParameter)
      CheckLastParameter
      ifnot PDRunning                                   '   If the motor controller isn't running
        outa[24..19]~~                                  '     Forcefully disable all motors (cannot be undone, firmware must be reset)
        abort @EncoderError
      if Parameter
        InterpolateMidVariables      
        case SetPower[LEFT_MOTOR]
          -1..negx:
            SetPosition[LEFT_MOTOR] := MotorPosition[LEFT_MOTOR] - Parameter
          1..posx:
            SetPosition[LEFT_MOTOR] := MotorPosition[LEFT_MOTOR] + Parameter
          other:
            SetPosition[LEFT_MOTOR] := MotorPosition[LEFT_MOTOR]
        case SetPower[RIGHT_MOTOR]
          -1..negx:
            SetPosition[RIGHT_MOTOR] := MotorPosition[RIGHT_MOTOR] - Parameter
          1..posx:
            SetPosition[RIGHT_MOTOR] := MotorPosition[RIGHT_MOTOR] + Parameter
          other:
            SetPosition[RIGHT_MOTOR] := MotorPosition[RIGHT_MOTOR]
        Decel[LEFT_MOTOR] := ||(-25 * MidVelocity[LEFT_MOTOR] * MidVelocity[LEFT_MOTOR] / (MidVelocity[LEFT_MOTOR] - DIVISOR * ||(SetPosition[LEFT_MOTOR] - MotorPosition[LEFT_MOTOR])))  
        Decel[RIGHT_MOTOR] := ||(-25 * MidVelocity[RIGHT_MOTOR] * MidVelocity[RIGHT_MOTOR] / (MidVelocity[RIGHT_MOTOR] - DIVISOR * ||(SetPosition[RIGHT_MOTOR] - MotorPosition[RIGHT_MOTOR])))  
        StillCnt~
        Mode := STOPPING
      else                                              '   For a zero stopping distance:
        waitcnt(constant(_clkfreq / DIVISOR) + cnt)          '     Prevent race condition with PDLoop
        SetPower[LEFT_MOTOR]~                           '     Disable power to both motors
        SetPower[RIGHT_MOTOR]~
        StillCnt~
        Mode := POWER                                   '     Set the motors to power mode
    elseif strcomp(@InputBuffer, string("TURN"))        ' Command: Turn a specified number of degrees around a circle of a specified radius
      ' Read both parameters before processing them (and adjust encoder positions per revolution / 2 wheels)
      Parameter[0] := ParseHex(NextParameter) 
      Parameter[1] := ParseHex(NextParameter)           '   To prevent changes with a "Too few parameters" error
      CheckLastParameter
      ifnot PDRunning
        abort @EncoderError
      case Parameter[1]
        1..127:                                   
          InterpolateMidVariables
          SetVelocity[LEFT_MOTOR] := SetVelocity[RIGHT_MOTOR] := Parameter[1]
          SetPosition[LEFT_MOTOR] := MidPosition[LEFT_MOTOR] + ~~Parameter[0] * (POSITIONS_PER_ROTATION / 2) / 360' Add travel distance to current position
          SetPosition[RIGHT_MOTOR] := MidPosition[RIGHT_MOTOR] - ~~Parameter[0] * (POSITIONS_PER_ROTATION / 2) / 360
          StillCnt~
          Mode := POSITION
        other:
          abort @InvalidParameter
{
    elseif strcomp(@InputBuffer, string("ARC"))         ' Command: Turn a specified number of degrees around a circle of a specified radius
      Parameter[0] := ParseHex(NextParameter)           '   Read both parameters before processing them
      Parameter[1] := ParseHex(NextParameter)           '   To prevent changes with a "Too few parameters" error
      CheckLastParameter
}
    elseif strcomp(@InputBuffer, string("ACC"))
      Parameter := ParseHex(NextParameter)
      CheckLastParameter
      case Parameter
        0..511:                                   
          MaxPosAccel := Parameter
        other:
          abort @InvalidParameter
    elseif     strcomp(@InputBuffer, string("SPD"))     ' Command: Respond with the current motor speeds
      CheckLastParameter
      ifnot PDRunning
        abort @EncoderError
      OutputHex(MotorSpeed[LEFT_MOTOR], 4)
      OutputChr(" ")
      OutputHex(MotorSpeed[RIGHT_MOTOR], 4)
    elseif strcomp(@InputBuffer, string("HEAD"))        ' Command: Respond with the current heading, relative to last reset
      CheckLastParameter
      ifnot PDRunning
        abort @EncoderError
      OutputHex((((MotorPosition[LEFT_MOTOR] + MotPosOffset[LEFT_MOTOR]) - (MotorPosition[RIGHT_MOTOR] + MotPosOffset[RIGHT_MOTOR])) // POSITIONS_PER_ROTATION) * 360 / POSITIONS_PER_ROTATION, 3)
    elseif strcomp(@InputBuffer, string("DIST"))        ' Command: Respond with the accumulative distance each motor has traveled, relative to last reset
      CheckLastParameter
      ifnot PDRunning
        abort @EncoderError
      OutputHex(MotorPosition[LEFT_MOTOR] + MotPosOffset[LEFT_MOTOR], 8)
      OutputChr(" ")
      OutputHex(MotorPosition[RIGHT_MOTOR] + MotPosOffset[RIGHT_MOTOR], 8)
    elseif strcomp(@InputBuffer, string("RST"))         ' Command: Reset heading and accumulated distance
      CheckLastParameter
      MotPosOffset[LEFT_MOTOR] := -MotorPosition[LEFT_MOTOR]
      MotPosOffset[RIGHT_MOTOR] := -MotorPosition[RIGHT_MOTOR]
    elseif strcomp(@InputBuffer, string("BLNK"))        ' Command: Blink the specified pin at a specified rate
      Parameter[0] := ParseHex(NextParameter)           ' Pin number
      Parameter[1] := ParseHex(NextParameter)           ' Frequency
      CheckLastParameter
      Parameter[2] := |< Parameter[0]                   ' Create a pin mask from the pin number
      if Parameter[2] & OUTPUTABLE                      ' Only use this command on a pin that can be an output
        if Parameter[1]                                 
          if Parameter[2] & PingMask                    ' If the pin used to be a PING))) sensor pin
            ResetPingDriver(PingMask &= !Parameter[0])  ' Restart the PING))) sensor processor with the pin removed from the mask
          GPIOMask &= !Parameter[2]                     ' Flag the pin as a GPIO pin
          outa[Parameter[0]]~                           ' The I/O pin must be set as a low input
          dira[Parameter[0]]~~
          ctra := constant(%00101 << 26) + Parameter[0] ' Set counter A to toggle the I/O pin
          frqa := constant($8000_0000 / _clkfreq / 10) * Parameter[1] ' Calculate the toggle period            
        else
          ctra~                                         ' A frequency of 0 turns of the counter
    elseif strcomp(@InputBuffer, string("VERB"))        ' Command: Set verbose mode
      Parameter := ParseHex(NextParameter)
      CheckLastParameter
      case Parameter
        0..1:
          Verbose := Parameter
        other:
          abort @InvalidParameter

    elseif strcomp(@InputBuffer, string("KPOS"))          ' Command: Set Kinect Pan servo (Tang)
      Parameter := ParseHex(NextParameter)
      CheckLastParameter
      SetPanPosition(Parameter)
      
    elseif strcomp(@InputBuffer, string("SV"))         ' Command: Set Servos (Tang) 
      Parameter[0] := ParseHex(NextParameter)             ' I/O Pin
      Parameter[1] := ParseHex(NextParameter)             ' Parameter 1 - 180
      CheckLastParameter
      SetServoPosition(Parameter[0], Parameter[1]) 

    else
      abort @InvalidCommand

  return 0


Pri InterpolateMidVariables | Difference
'' Sets MidVelocity and MidPosition variables to values that would create the current SetPower at the current MotorPosition  

  if Difference := SetPower[LEFT_MOTOR] / Kp            ' Determine left MidPosition to MotorPosition offset
    if Difference => DEADZONE                           ' Adjust for the deadzone   
      Difference -= DEADZONE 
    elseif Difference =< DEADZONE
      Difference += DEADZONE
    MidPosition[LEFT_MOTOR] := MotorPosition[LEFT_MOTOR] + Difference ' Add it back to the current Motor Position
  else 
    MidPosition[LEFT_MOTOR] := MotorPosition[LEFT_MOTOR]
  MidPosAcc[LEFT_MOTOR]~                                ' Clear the fractional part

  if Difference := SetPower[RIGHT_MOTOR] / Kp           ' Determine right MidPosition to MotorPosition offset
    if Difference => DEADZONE                                   ' Remove the deadzone
      Difference -= DEADZONE
    elseif Difference =< DEADZONE
      Difference += DEADZONE             
    MidPosition[RIGHT_MOTOR] := MotorPosition[RIGHT_MOTOR] + Difference ' Add it back to the current Motor Position
  else
    MidPosition[RIGHT_MOTOR] := MotorPosition[RIGHT_MOTOR]
  MidPosAcc[RIGHT_MOTOR]~                               ' Clear the fractional part

  ' Set the left MidVelocity to the current motor speed
  MidVelocity[LEFT_MOTOR] := MotorSpeed[LEFT_MOTOR]
  MidVelAcc[LEFT_MOTOR]~                                ' Clear the fractional part

  ' Set the right MidVelocity to the current motor speed
  MidVelocity[RIGHT_MOTOR] := MotorSpeed[RIGHT_MOTOR]
  MidVelAcc[LEFT_MOTOR]~                                ' Clear the fractional part


Pri PDLoop | NextCNT, ToSignExtend                      '' Measure, set, and maintain wheel position

  ' Open a serial connection to the position sensors
  Encoders.Start(ENCODERS_PIN, ENCODERS_PIN, %1100, 19_200)                     
  waitcnt(constant(_clkfreq / 2) + cnt)                 ' Give the driver time to start
  repeat 3                                              ' Clear the position sensors' input buffers
    Encoders.Tx(constant(CLEAR_POSITION << 3 | ALL_ENCODERS))
  ' Disable the transmit delay
  Encoders.Tx(constant(SET_TRANSMIT_DELAY << 3 | ALL_ENCODERS))
  Encoders.Tx(0)

  Encoders.Tx(constant(QUERY_SPEED << 3) | LEFT_ENCODER) ' Request left motor speed
  ifnot !Encoders.RxTime(TIMEOUT)                       ' Don't start the motor controller if a position sensor isn't working
    cogstop(cogid)
  ToSignExtend := Encoders.RxTime(TIMEOUT)              ' Read left motor speed
  MotorSpeed[LEFT_MOTOR] := ~ToSignExtend               ' Work around inability to sign the result of an expression

  Encoders.Tx(constant(QUERY_SPEED << 3) | RIGHT_ENCODER) ' Request right motor speed
  ifnot !Encoders.RxTime(TIMEOUT)                       ' Don't start the motor controller if a position sensor isn't working
    cogstop(cogid)
  ToSignExtend := Encoders.RxTime(TIMEOUT)              ' Read right motor speed
  MotorSpeed[RIGHT_MOTOR] := -~ToSignExtend             ' Work around inability to sign the result of an expression

  Motors.Start                                          ' Start the motor controller

  Encoders.Tx(constant(QUERY_POSITION << 3) | LEFT_ENCODER) ' Request distance traveled for left wheel
  ToSignExtend := Encoders.RxTime(TIMEOUT) << 8 | Encoders.RxTime(TIMEOUT)
  SetPosition[LEFT_MOTOR] := MotorPosition[LEFT_MOTOR] := ~~ToSignExtend

  Encoders.Tx(constant(QUERY_POSITION << 3) | RIGHT_ENCODER) ' Request distance traveled for right wheel
  ToSignExtend := Encoders.RxTime(TIMEOUT) << 8 | Encoders.RxTime(TIMEOUT)
  SetPosition[RIGHT_MOTOR] := MotorPosition[RIGHT_MOTOR] := -~~ToSignExtend

  PDRunning~~                                           ' Let the command parser know that the motor controller is running

  NextCNT := cnt                                        ' Set up a timed loop
  repeat
    Motors.Right(PDIteration(RIGHT_MOTOR))              ' Service right wheel and motor
    Motors.Left(PDIteration(LEFT_MOTOR))                ' Service left wheel and motor
    waitcnt(NextCNT += constant(_clkfreq / DIVISOR))    ' Loop DIIVSOR times per second

  
Pri PDIteration(Side) | Difference[2], ToSignExtend, Limit, RawPosition
'' Read the wheel's speed and position, and set its power

  Encoders.Tx(constant(QUERY_SPEED << 3) | Side + 1)    ' Request motor speed
  ifnot !Encoders.RxTime(TIMEOUT)                       ' Ignore high byte and abort on error
    PDRunning~
    'Motors.Stop
    cogstop(cogid)
  ToSignExtend := Encoders.RxTime(TIMEOUT)
  MotorSpeed[Side] := ~ToSignExtend * (2 - Side << 2)   ' Multiply by two and reverse sign for right motor

  Encoders.Tx(constant(QUERY_POSITION << 3) | Side + 1) ' Request distance traveled
  ToSignExtend := (RawPosition := (Encoders.RxTime(TIMEOUT) << 8 | Encoders.RxTime(TIMEOUT))) * (1 - Side << 1) & $FFFF - MotorPosition[Side]
  MotorPosition[Side] += ~~ToSignExtend

  if ||MotorSpeed[Side] =< 2 or RawPosition == 00       ' Keep track of how long the motor hasn't been moving
    StillCnt[Side]++
    StillCnt[Side] <#= 127
  else
    StillCnt[Side]~ 

  ' Keep the mid point from traveling too far away from the current motor position
      
  Difference := SetPower[Side] / Kp                     ' Interpolate MidPosition to MotorPosition offset, based on current power

  if Difference => DEADZONE                             ' Adjust for the deadzone   
    Difference -= DEADZONE
  elseif Difference =< DEADZONE
    Difference += DEADZONE
  else
    Difference~
{  
  ' Check to see if offset is higher than it should be for the current power level (When motor is moving)
  if Difference > DEADZONE * 2 + 1 and ||(MidPosition[Side] - MotorPosition[Side]) > ||Difference * 11 / 10
    ' If so, bring the set point closer to the physical position
    MidPosition[Side] := MotorPosition[Side] - (MotorPosition[Side] - MidPosition[Side])
    if StillCnt => constant(DIVISOR / 2)                ' If the offset was high and the motor isn't moving
      PDRunning~                                        ' Then something is wrong - stop the motor driver
      Motors.Stop
      cogstop(cogid)
}        
  case Mode                                             ' Set motor power based on current control method
    POWER:                                              ' Run the motors at a set power level
      ' MotorPower approaches SetPower as limited by MaxPowAccel
      return MotorPower[Side] += -MaxPowAccel #> (SetPower[Side] - MotorPower[Side]) <# MaxPowAccel
      StillCnt~

    VELOCITY:                                           ' Maintain the motors at a set velocity
      ' MidVelocity / DIVISOR approaches SetVelocity as limited by MaxPosAccel
      MidVelAcc[Side] += -MaxPosAccel #> (SetVelocity[Side] - MidVelocity[Side]) * DIVISOR - MidVelAcc[Side] <# MaxPosAccel
      MidVelocity[Side] += MidVelAcc[Side] / DIVISOR
      MidVelAcc[Side] //= DIVISOR

      ' MidPosition / DIVISOR increases by MidVelocity / DIVISOR
      MidPosAcc[Side] += MidVelocity[Side]                             
      MidPosition[Side] += MidPosAcc[Side] / DIVISOR
      MidPosAcc[Side] //= DIVISOR

      ' Measure motors physical distance from the set point
      Difference := MidPosition[Side] - MotorPosition[Side]

      if Difference => DEADZONE                         ' Adjust for the deadzone   
        Difference -= DEADZONE
      elseif Difference =< DEADZONE
        Difference += DEADZONE
      else
        Difference~

      ' SetPower is proportional to the motors physical distance from the set point, limited by Motors#MAX_ON_TIME
      SetPower[Side] := -Motors#MAX_ON_TIME #> Difference * Kp <# Motors#MAX_ON_TIME

      ' MotorPower approaches SetPower as limited by MaxPowAccel
      return MotorPower[Side] += -MaxPowAccel #> (SetPower[Side] - MotorPower[Side]) <# MaxPowAccel

    STOPPING:                                           ' Slow to a stop at a set position
      ' MidPosition approaches SetPosition as limited by the deceleration curve
      Limit := ^^(constant(8 * DIVISOR * DIVISOR) * (SetPosition[Side] - MidPosition[Side])/ Decel ) * Decel / 2
      MidPosAcc[Side] += -Limit #> (SetPosition[Side] - MidPosition[Side]) * constant(DIVISOR * DIVISOR) + MidPosAcc[Side] <# Limit
      MidPosition[Side] += MidPosAcc[Side] / constant(DIVISOR * DIVISOR)
      MidPosAcc[Side] //= constant(DIVISOR * DIVISOR)

      ' Measure motors physical distance from the set point
      Difference := MidPosition[Side] - MotorPosition[Side]

      if Difference => DEADZONE                         ' Adjust for the deadzone   
        Difference -= DEADZONE
      elseif Difference =< DEADZONE
        Difference += DEADZONE
      else
        Difference~

      ' SetPower is proportional to the motors physical distance from the set point, limited by Motors#MAX_ON_TIME
      SetPower[Side] := -Motors#MAX_ON_TIME #> Difference * Kp <# Motors#MAX_ON_TIME

      ' MotorPower approaches SetPower as limited by MaxPowAccel
      return MotorPower[Side] += -MaxPowAccel #> (SetPower[Side] - MotorPower[Side]) <# MaxPowAccel
   
'   ARC_POSITION:                                       ' Turn along an arc

    POSITION:                                           ' Travel to a set position
      ' MidVelocity approaches SetVelocity as limited by MaxPosAccel / DIVISOR
      MidVelAcc[Side] += -MaxPosAccel #> (SetVelocity[Side] - MidVelocity[Side]) * DIVISOR - MidVelAcc[Side] <# MaxPosAccel
      MidVelocity[Side] += MidVelAcc[Side] / DIVISOR
      MidVelAcc[Side] //= DIVISOR

      ' MidPosition approaches SetPosition as limited by the deceleration curve
      Limit := MidVelocity[Side] * DIVISOR + MidVelAcc[Side] <# ^^(constant(8 * DIVISOR * DIVISOR) * (SetPosition[Side] - MidPosition[Side])/ MaxPosAccel) * MaxPosAccel / 2
      MidPosAcc[Side] += -Limit #> (SetPosition[Side] - MidPosition[Side]) * constant(DIVISOR * DIVISOR) + MidPosAcc[Side] <# Limit
      MidPosition[Side] += MidPosAcc[Side] / constant(DIVISOR * DIVISOR)
      MidPosAcc[Side] //= constant(DIVISOR * DIVISOR)

      ' Measure motors physical distance from the set point
      Difference := MidPosition[Side] - MotorPosition[Side]

      if Difference => DEADZONE                         ' Adjust for the deadzone   
        Difference -= DEADZONE
      elseif Difference =< DEADZONE
        Difference += DEADZONE
      else
        Difference~

      ' SetPower is proportional to the motors physical distance from the set point, limited by Motors#MAX_ON_TIME
      SetPower[Side] := -Motors#MAX_ON_TIME #> Difference * Kp <# Motors#MAX_ON_TIME

      ' MotorPower approaches SetPower as limited by MaxPowAccel
      return MotorPower[Side] += -MaxPowAccel #> (SetPower[Side] - MotorPower[Side]) <# MaxPowAccel

    other:                                              ' Invalid state
      return MotorPower[Side]~                          ' Stop the motor


Pri ProcessPings | PingPins, Index, Response            '' Output distance readings from all active PING))) sensors
' PingPins is a working variable used to count the number of PING)))sensors that are currently active  

  if PingPins := PingMask                               ' Copy mask of affected PING))) sensors into PingPins and only proceed if some are active
    outa |= PingMask                                    ' Set the outputs high for all active PING))) sensors
    dira |= PingMask                                    ' Set directions to outputs 
    outa &= !PingMask                                   ' Set the outputs low       
    dira &= !PingMask                                   ' Set directions to inputs
    waitcnt(constant(_clkfreq / 40) + cnt)
     
    Index~
    repeat
      if PingPins & %1
        Response := PingResults[Index] >> 9
        if Response < 18 or Response > 2900
          'abort @PINGNotResponding                     ' Send error if any PING))) sensors don't respond
          Response~
        OutputHex(Response, 3)
        OutputChr(" ")
        Index += 2
      PingPins >>= 1
    while PingPins
    OutputBuffer[OutputIndex--]~                        ' Remove last space from output buffer


Pri ResetPingDriver(NewMask)                            '' Restart the PING))) driver with  anew set of pins

    PingMask := NewMask & PINGABLE                      ' Only use allowed pins
    GPIOMask &= !PingMask                               ' Remove used pins from GPIO set
    Ping.Stop                                           ' Stop the PING))) driver
    PingResults := PingMask                             ' Set the new set of pins
    Ping.Start(@PingResults)                            ' Start the PING))) driver


Pri ChecksumParse | Index, Accumulator, RemoteChecksum  '' Verify checksum and parse input command

  Accumulator~
  repeat Index from 0 to InputIndex - 1
    Accumulator += InputBuffer[Index]
  RemoteChecksum := Term.Rx << 8
  RemoteChecksum |= Term.Rx
  if Accumulator == RemoteChecksum
    \Parse
  else
    abort @BadChecksum


Pri NextParameter                                       '' Condition the next input parameter and return its pointer

  repeat until ++ParseIndex => InputIndex               ' Ignore whitespace
    case InputBuffer[ParseIndex]                        ' First character is always whitespace
      0, HT, " ":
      other:
        quit                                            ' ParseIndex points to first non-whitespace character
  if ParseIndex => InputIndex                           ' If at the end of the buffer (or passed it, just in case)
    abort @TooFewParameters                             '  then there are no more parameters
  result := @InputBuffer[ParseIndex]                    ' When responding, point to the next parameter,

  repeat ParseIndex from ParseIndex to InputIndex - 1   ' But first...
    case InputBuffer[ParseIndex]                         
      NUL, HT, " " :                                    '  Null terminate the parameter           
        InputBuffer[ParseIndex]~
        quit
      "a".."z" :                                        '  Set all command characters to uppercase
        InputBuffer[ParseIndex] -= constant("a" - "A")


Pri CheckLastParameter                                  '' Abort if there are any unparsed input parameters

  repeat until (InputBuffer[ParseIndex] <> 0 and InputBuffer[ParseIndex] <> " " and InputBuffer[ParseIndex] <> HT) or ParseIndex == InputIndex - 1
    ++ParseIndex

  ifnot ParseIndex == InputIndex - 1                    '   Ensure that there are no further arguments           
    abort @TooManyParameters                            '   To prevent changes with a "Too many parameters" error


Pri ParseHex(Pointer) | Character                       '' Interpret an ASCII hexadecimal string

  repeat 8
    case Character := byte[Pointer++]
      NUL :
        Pointer--
        return result
      "0".."9" :
        result := result << 4 + Character - "0"
      "A".."F" :
        result := result << 4 + Character - constant("A" - 10)
      "a".."f" :
        result := result << 4 + Character - constant("a" - 10)
      other :
        abort @InvalidParameter

  if byte[Pointer]                                      ' Make sure there are no remaining characters after parsing first eight.                        
    abort @InvalidParameter


Pri OutputChr(Char)                                     '' Add a character to the output buffer

  if OutputIndex < BUFFER_LENGTH - 3
    OutputBuffer[OutputIndex++] := Char
    OutputBuffer[OutputIndex]~

  
Pri OutputStr(Pointer)                                  '' Concatenate a string to the end of the output buffer

  if strsize(Pointer) + 1 =< BUFFER_LENGTH - OutputIndex                        ' Check for overflow
    bytemove(@OutputBuffer + OutputIndex, Pointer, strsize(Pointer) + 1)        ' Copy the string to the buffer, including the terminator
    OutputIndex += strsize(Pointer)                                             ' Increment the output buffer index by the length of the sting, not including the terminator


Pri OutputDec(value) | i, x                             '' Create a decimal string and concatenate the end of the output buffer

'' Print a decimal number

  x := value == NEGX                                    ' Check for max negative
  if value < 0
    value := ||(value+x)                                ' If negative, make positive; adjust for max negative
    OutputChr("-")                                      ' and output sign

  i := 1_000_000_000                                    ' Initialize divisor

  repeat 10                                             ' Loop for 10 digits
    if value => i                                                               
      OutputChr(value / i + "0" + x*(i == 1))           ' If non-zero digit, output digit; adjust for max negative
      value //= i                                       ' and digit from value
      result~~                                          ' flag non-zero found
    elseif result or i == 1
      OutputChr("0")                                    ' If zero digit (or only digit) output it
    i /= 10                                             ' Update divisor


Pri OutputHex(value, digits)                            '' Create a hexadecimal string and concatenate the end of the output buffer

'' Print a hexadecimal number

  if OutputIndex < BUFFER_LENGTH - digits - 2
    value <<= (8 - digits) << 2
    repeat digits    
      OutputBuffer[OutputIndex++] := lookupz((value <-= 4) & $F : "0".."9", "A".."F")
    OutputBuffer[OutputIndex]~


Pri SendResponse                                        '' Transmit the string in the output buffer and clear the buffer

  Term.Str(@OutputBuffer)                               ' Transmit the buffer contents
  Term.Str(@Prompt)                                     ' Transmit the prompt
  InputIndex~                                           ' Clear the buffers
  OutputBuffer~
  OutputIndex~


Pri SendChecksumResponse | Index, Accumulator           '' Transmit the string in the output buffer, and its checksum, then clear the buffer

  Accumulator := PROMPT_SUM                             ' Generate the checksum
  repeat Index from 0 to strsize(@OutputBuffer)
    Accumulator += OutputBuffer[Index]

  Term.Str(@OutputBuffer)                               ' Transmit the buffer contents
  Term.Str(@CheckSumPrompt)                             ' Transmit the prompt
  Term.Tx(0)                                            ' Null must be transmitted separately (cannot be part of CheckSumPrompt string)
  Term.Tx(Accumulator >> 8)                             ' Transmit the Checksum
  Term.Tx(Accumulator)

  InputIndex~                                           ' Clear the buffers
  OutputBuffer~
  OutputIndex~


Pri ReadEEPROM(startAddr, endAddr, eeStart) | addr

  ''Copy from EEPROM beginning at eeStart address to startAddr..endAddr in main RAM.
  
  SetAddr(eeStart)                                      ' Set EEPROM's address pointer 
  i2cstart
  SendByte(%10100001)                                   ' EEPROM I2C address + read operation
  if startAddr == endAddr
    addr := startAddr
  else
    repeat addr from startAddr to endAddr - 1           ' Main RAM index startAddr to endAddr
      byte[addr] := GetByte                             ' GetByte byte from EEPROM & copy to RAM 
      SendAck(I2C_ACK)                                  ' Acknowledge byte received
  byte[addr] := GetByte                                 ' GetByte byte from EEPROM & copy to RAM 
  SendAck(I2C_NACK)
  i2cstop                                               ' Stop sequential read

  
Pri WriteEEPROM(startAddr, endAddr, eeStart) | addr, page, eeAddr

  ''Copy startAddr..endAddr from main RAM to EEPROM beginning at eeStart address.

  addr := startAddr                                     ' Initialize main RAM index
  eeAddr := eeStart                                     ' Initialize EEPROM index
  repeat
    page := addr +PAGE_SIZE -eeAddr // PAGE_SIZE <# endaddr +1 ' Find next EEPROM page boundary
    SetAddr(eeAddr)                                     ' Give EEPROM starting address
    repeat                                              ' Bytes -> EEPROM until page boundary
      SendByte(byte[addr++])
    until addr == page
    i2cstop                                             ' From 24LC256's page buffer -> EEPROM
    eeaddr := addr - startAddr + eeStart                ' Next EEPROM starting address
  until addr > endAddr                                  ' Quit when RAM index > end address


Pri SetAddr(addr) : ackbit

  'Sets EEPROM internal address pointer.

  ' Poll until acknowledge.  This is especially important if the 24LC256 is copying from buffer to EEPROM.
  ackbit~~                                              ' Make acknowledge 1
  repeat                                                ' Send/check acknowledge loop
    i2cstart                                            ' Send I2C start condition
    ackbit := SendByte(%10100000)                       ' Write command with EEPROM's address
  while ackbit                                          ' Repeat while acknowledge is not 0

  SendByte(addr >> 8)                                   ' Send address high byte
  SendByte(addr)                                        ' Send address low byte


Pri I2cStart

  ' I2C start condition.  SDA transitions from high to low while the clock is high.
  ' SCL does not have the pullup resistor called for in the I2C protocol, so it has to be
  ' set high. (It can't just be set to inSendByte because the resistor won't pull it up.)

  dira[SCL]~                                            ' SCL pin outSendByte-high
  dira[SDA]~                                            ' Let pulled up SDA pin go high
  dira[SDA]~~                                           ' SDA -> outSendByte for SendByte method


Pri I2cStop

  ' Send I2C stop condition.  SCL must be high as SDA transitions from low to high.
  ' See note in i2cStart about SCL line.

  dira[SDA]~~
  dira[SCL]~                                            ' SCL -> high
  dira[SDA]~                                            ' SDA -> inSendByte GetBytes pulled up

   
Pri SendAck(ackbit)

  ' Transmit an acknowledgment bit (ackbit).

  dira[SDA] := !ackbit                                  ' Set SDA output state to ackbit
  dira[SDA]~~                                           ' Make sure SDA is an output
  dira[SCL]~                                            ' Send a pulse on SCL
  dira[SCL]~~
  dira[SDA]~                                            ' Let go of SDA


Pri GetAck : ackbit

  ' GetByte and return acknowledge bit transmitted by EEPROM after it receives a byte.
  ' 0 = I2C_ACK, 1 = I2C_NACK.

  dira[SDA]~                                            ' SDA -> SendByte so 24LC256 controls
  dira[SCL]~                                            ' Start a pulse on SCL
  ackbit := ina[SDA]                                    ' GetByte the SDA state from 24LC256
  dira[SCL]~~                                           ' Finish SCL pulse
  dira[SDA]~~                                           ' SDA -> outSendByte, master controls

  
Pri SendByte(b) : ackbit | i

  ' Shift a byte to EEPROM, MSB first.  Return if EEPROM acknowledged.  Returns
  ' acknowledge bit.  0 = I2C_ACK, 1 = I2C_NACK.

  b ><= 8                                               ' Reverse bits for shifting MSB right
  dira[SCL]~~                                           ' SCL low, SDA can change
  repeat 8                                              ' 8 reps sends 8 bits
    dira[SDA] := !b                                     ' Lowest bit sets state of SDA
    dira[SCL]~                                          ' Pulse the SCL line
    dira[SCL]~~
    b >>= 1                                             ' Shift b right for next bit
  ackbit := GetAck                                      ' Call GetByteAck and return EEPROM's Ack


Pri GetByte : value

  ' Shift in a byte, MSB first.  

  value~                                                ' Clear value
  dira[SDA]~                                            ' SDA input so 24LC256 can control
  repeat 8                                              ' Repeat shift in eight times
    dira[SCL]~                                          ' Start an SCL pulse
    value <-= 1                                         ' Shift the value left
    value |= ina[SDA]                                   ' Add the next most significant bit
    dira[SCL]~~                                         ' Finish the SCL pulse


Dat

Prompt                  byte CR, 0
CheckSumPrompt
NullString              byte 0
Nack                    byte "ERROR", 0
Overflow                byte " - Overflow", 0
BadChecksum             byte " - Bad checksum", 0
InvalidCommand          byte " - Invalid command", 0
InvalidParameter        byte " - Invalid parameter", 0
TooFewParameters        byte " - Too few parameters", 0
TooManyParameters       byte " - Too many parameters", 0
EncoderError            byte " - Encoder error", 0
NonCommand              byte " - Nothing happens", 0  
