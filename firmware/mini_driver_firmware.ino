/* Copyright (c) 2014, Dawn Robotics Ltd
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.

3. Neither the name of the Dawn Robotics Ltd nor the names of its contributors 
may be used to endorse or promote products derived from this software without 
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include <Servo.h>
#include "Encoder.h"

//------------------------------------------------------------------------------
const uint8_t VERSION_MAJOR = 0;
const uint8_t VERSION_MINOR = 38;
const uint16_t FIRMWARE_ID = 0xACED;

const uint16_t MAX_MSG_SIZE = 32;
const uint16_t MSG_START_BYTES = 0xFFFF;
const uint16_t MSG_ID_POS = 2;
const uint16_t MSG_SIZE_POS = 3;
const uint16_t MSG_HEADER_SIZE = 2 + 1 + 1; // Start bytes + Id + size

const uint8_t COMMAND_ID_GET_FIRMWARE_INFO = 1;
const uint8_t COMMAND_ID_SET_OUTPUTS = 2;
const uint8_t COMMAND_ID_SET_PAN_SERVO_LIMITS = 3;
const uint8_t COMMAND_ID_SET_TILT_SERVO_LIMITS = 4;
const uint8_t COMMAND_ID_SET_SENSOR_CONFIGURATION = 5;

const uint8_t RESPONSE_ID_FIRMWARE_INFO = 1;
const uint8_t RESPONSE_ID_INVALID_COMMAND = 2;
const uint8_t RESPONSE_ID_INVALID_CHECK_SUM = 3;
const uint8_t RESPONSE_ID_BATTERY_READING = 4;    // Not used any more
const uint8_t RESPONSE_ID_SENSOR_READINGS = 5;

const int LEFT_MOTOR_DIR_PIN = 7;
const int LEFT_MOTOR_PWM_PIN = 9;
const int RIGHT_MOTOR_DIR_PIN = 8;
const int RIGHT_MOTOR_PWM_PIN = 10;
const int PAN_SERVO_PIN = 11;
const int TILT_SERVO_PIN = 6;
const int BATTERY_VOLTAGE_PIN = A7;

const int LEFT_ENCODER_PIN_1 = 2;
const int LEFT_ENCODER_PIN_2 = 4;
const int RIGHT_ENCODER_PIN_1 = 3;
const int RIGHT_ENCODER_PIN_2 = 5;

const uint8_t RENC_SINGLE_BITMASK = 0x02;
const uint8_t LENC_SINGLE_BITMASK = 0x01;

const int ULTRASONIC_PIN = 12;

const int MOTOR_PWM_80HZ_OCR2 = 62;     // Fast PWM used for fast motor speeds
const int MOTOR_PWM_20HZ_OCR2 = 255;    // Slow PWM used for slow motor speeds
const int MOTOR_PWM_80HZ_DUTY_CYCLE = 100;
const int MOTOR_PWM_20HZ_DUTY_CYCLE = 20;
                                            
const uint32_t SENSOR_READ_INTERVAL_MS = 10;
const uint32_t ULTRASONIC_READ_INTERVAL_MS = 500;

const uint32_t NO_ULTRASONIC_SENSOR_PRESENT = 1000;  // Value returned if it looks like no sensor is attached
const uint32_t MAX_ULTRASONIC_RANGE_CM = 400;
const uint32_t ULTRASONIC_US_PER_CM = 58;
const uint32_t MAX_ULTRASONIC_TIMEOUT_US = MAX_ULTRASONIC_RANGE_CM*ULTRASONIC_US_PER_CM;
const int ABSOLUTE_MIN_PWM = 400;
const int ABSOLUTE_MAX_PWM = 2600;

const uint32_t NUM_TICKS_PER_MOTOR_WAVE = 100;
const unsigned long MOTOR_COMMAND_TIMEOUT_MS = 2000;

//------------------------------------------------------------------------------
// This class is provided because the Arduino Servo library maps minimum and 
// maximum bounds to a single byte for some reason.
class ServoLimits
{
    // MIN_PULSE_WIDTH and MAX_PULSE_WIDTH come from Servo.h
    public: ServoLimits( int minPWM=MIN_PULSE_WIDTH, int maxPWM=MAX_PULSE_WIDTH )
    {
        setLimits( minPWM, maxPWM );
    }
    
    public: void setLimits( int minPWM, int maxPWM )
    {
        mMinPWM = constrain( minPWM, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM );
        mMaxPWM = constrain( maxPWM, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM );
    }
    
    public: int convertAngleToPWM( int angle )
    {
        angle = constrain( angle, 0, 180 );
        return map( angle, 0, 180, mMinPWM, mMaxPWM );
    }
    
    public: int getMinPWM() const { return mMinPWM; }
    public: int getMaxPWM() const { return mMaxPWM; }
    
    private: int mMinPWM;
    private: int mMaxPWM;
};

//------------------------------------------------------------------------------
enum eMotorDirection
{
    eMD_Forwards,
    eMD_Backwards
};

enum eMessageState
{
    eMS_WaitingForMessage,
    eMS_ReceivingMessage
};

eMessageState gMessageState = eMS_WaitingForMessage;
uint8_t gMsgBuffer[ MAX_MSG_SIZE ];
uint16_t gNumMsgBytesReceived = 0;

Servo gPanServo;
ServoLimits gPanServoLimits;
Servo gTiltServo;
ServoLimits gTiltServoLimits;

Encoder gLeftEncoder( LEFT_ENCODER_PIN_1, LEFT_ENCODER_PIN_2 );
Encoder gRightEncoder( RIGHT_ENCODER_PIN_1, RIGHT_ENCODER_PIN_2 );

uint8_t gLeftMotorDutyCycle = 0;
uint8_t gRightMotorDutyCycle = 0;
eMotorDirection gLeftMotorDirection = eMD_Forwards;
eMotorDirection gRightMotorDirection = eMD_Forwards;
uint8_t gPanServoAngle = 90;
uint8_t gTiltServoAngle = 90;
unsigned long gLastCommandTime = 0;
unsigned long gLastSensorReadTimeMS = 0;
unsigned long gLastUltrasonicReadTimeMS = 0;

volatile uint8_t gCurMotorWaveTick = 0;

int gLastUltrasonicReadingCM = NO_ULTRASONIC_SENSOR_PRESENT;
unsigned long gUltrasonicPulseStartTimeUS = 0;
bool gbTakingUltrasonicSensorReading = false;

// This configuration byte determines which pins are used as digital inputs
// A bit set to 1 indicates that the pin is used as a digital input
// A bit set to 0 indicates that the pin is used for its alternative function if there is one
// The alternative function for the analog pins is an analogRead. The alternative function for
// D12 (the ULTRASONIC_PIN) is to measure range with an ultrasonic sensor
//
// The bit order is
//
// pin     A5 | A4 | A3 | A2 | A1 | A0 | D13 | D12
// bit      7                                    0
//
// This bit order is also used when returning the digital readings in sendSensorReadingsMessage
uint8_t gSensorConfigurationA = 0;    // By default no pins are configured as digital inputs

// This configuration byte determines whether single output or quadrature encoders are used.
// The bits are interpreted as follows
//
// bit      R | R | R | R | R | R | RENC_SINGLE | LENC_SINGLE
//
// Bits marked 'R' are reserved and should be set to 0
//
// If RENC_SINGLE is set to 1 then the right encoder is single output. If RENC_SINGLE is set
// to 0 then the right encoder is a quadrature encoder
//
// If LENC_SINGLE is set to 1 then the left encoder is single output. If LENC_SINGLE is set
// to 0 then the left encoder is a quadrature encoder
uint8_t gSensorConfigurationB = 0;    // By default both encoders are quadrature encoders

//------------------------------------------------------------------------------
int measureDistanceUltrasonic( int pin );
uint8_t getMessageId() { return gMsgBuffer[ MSG_ID_POS ]; }
uint8_t getMessageSize() { return gMsgBuffer[ MSG_SIZE_POS ]; }
void receiveMessages();
void processMessage();
void sendFirmwareInfoResponse();
void sendInvalidCommandResponse();
void sendInvalidChecksumResponse();
void sendSensorReadingsMessage( int batteryReading, uint8_t digitalReadings, 
    int analogReadings[ 6 ], int gLastUltrasonicReadingCM,
    int32_t leftEncoderReading, int32_t rightEncoderReading );
uint8_t calculateCheckSum( const uint8_t* pData, uint8_t msgSize );

//------------------------------------------------------------------------------
void setup()
{
    // Set up timer 2 in CTC mode with a prescaler of clk/32, so with the chip 
    // running at 16MHz this gives 500,000 clock ticks per second. By varying
    // the value of OCR2 we will generate an interrupt from roughly 8000 times a
    // second to 2000 times a second which gives a motor PWM frequency of 80Hz
    // to 20Hz. Lower PWM frequencies give choppier movement
    noInterrupts();
    TCCR2 = (1 << 3) | 3;      // Activate timer in CTC mode with a prescaler of clk/32
    OCR2 = 0xFF;
    
    TIMSK |= (1 << OCIE2);     // Activate timer 2 output compare interrupt
    
    interrupts();
    
    gPanServo.attach( PAN_SERVO_PIN );
    gTiltServo.attach( TILT_SERVO_PIN );
    
    pinMode( LEFT_MOTOR_DIR_PIN, OUTPUT );
    pinMode( LEFT_MOTOR_PWM_PIN, OUTPUT );
    pinMode( RIGHT_MOTOR_DIR_PIN, OUTPUT );
    pinMode( RIGHT_MOTOR_PWM_PIN, OUTPUT );
    
    // Make all sensor pins inputs
    pinMode( 12, INPUT );
    pinMode( 13, INPUT );
    pinMode( A0, INPUT );
    pinMode( A1, INPUT );
    pinMode( A2, INPUT );
    pinMode( A3, INPUT );
    pinMode( A4, INPUT );
    pinMode( A5, INPUT );

    Serial.begin( 57600 );
}

//------------------------------------------------------------------------------
void loop()
{    
    // Read any commands from the serial connection
    receiveMessages();
    
    // Turn off the motors if we haven't received a command for a while
    unsigned long curTime = millis();
    
    if ( curTime - gLastCommandTime > MOTOR_COMMAND_TIMEOUT_MS )
    {
        gLeftMotorDutyCycle = 0;
        gRightMotorDutyCycle = 0;
        gLeftMotorDirection = eMD_Forwards;
        gRightMotorDirection = eMD_Forwards;
    }
    
    // Update the frequency of the motor PWM
    uint8_t lowestDutyCycle = ( gLeftMotorDutyCycle < gRightMotorDutyCycle 
        ? gLeftMotorDutyCycle : gRightMotorDutyCycle );
    
    if ( lowestDutyCycle <= MOTOR_PWM_20HZ_DUTY_CYCLE )
    {
        OCR2 = MOTOR_PWM_20HZ_OCR2;
    }
    else if ( lowestDutyCycle >= MOTOR_PWM_80HZ_DUTY_CYCLE )
    {
        OCR2 = MOTOR_PWM_80HZ_OCR2;
    }
    else
    {
        // Linear interpolation from 20Hz to 80Hz
        long int maxOcrChange = MOTOR_PWM_80HZ_OCR2 - MOTOR_PWM_20HZ_OCR2;
        
        int ocrDiff = (int)(maxOcrChange
            *(long int)((int)lowestDutyCycle - (int)MOTOR_PWM_20HZ_DUTY_CYCLE)
            /(long int)((int)MOTOR_PWM_80HZ_DUTY_CYCLE - (int)MOTOR_PWM_20HZ_DUTY_CYCLE));
        OCR2 = (uint8_t)((int)MOTOR_PWM_20HZ_OCR2 + ocrDiff);
    }
    
    // Update the motor directions
    digitalWrite( LEFT_MOTOR_DIR_PIN, ( eMD_Forwards == gLeftMotorDirection ? HIGH : LOW ) );
    digitalWrite( RIGHT_MOTOR_DIR_PIN, ( eMD_Forwards == gRightMotorDirection ? HIGH : LOW ) );
    gLeftEncoder.setGoingForward( eMD_Forwards == gLeftMotorDirection );
    gRightEncoder.setGoingForward( eMD_Forwards == gRightMotorDirection );
    
    // Update the servo angles
    gPanServo.writeMicroseconds( gPanServoLimits.convertAngleToPWM( gPanServoAngle ) );
    gTiltServo.writeMicroseconds( gTiltServoLimits.convertAngleToPWM( gTiltServoAngle ) );
    
    // Periodically kick off an ultrasonic sensor read
    if ( curTime - gLastUltrasonicReadTimeMS >= SENSOR_READ_INTERVAL_MS
       && !gbTakingUltrasonicSensorReading )
    {
        // Start to measure the range
        pinMode( ULTRASONIC_PIN, OUTPUT );
        digitalWrite( ULTRASONIC_PIN, LOW );
        delayMicroseconds( 2 );
        digitalWrite( ULTRASONIC_PIN, HIGH );
        delayMicroseconds( 5 );
        digitalWrite( ULTRASONIC_PIN, LOW );
    
        unsigned long startWaitTimeUS = micros();
    
        // Wait for the response pulse to start
        pinMode( ULTRASONIC_PIN, INPUT );
        bool bPulseStarted = false; 
        while ( !bPulseStarted
           && micros() - startWaitTimeUS < 500 )
        {
            bPulseStarted = digitalRead( ULTRASONIC_PIN ) == HIGH;
        }
        
        if ( bPulseStarted )
        {
            gUltrasonicPulseStartTimeUS = micros();
            gbTakingUltrasonicSensorReading = true;
        }
        else
        {
            // Looks like no sensor is attached. Wait a bit before trying again
            gLastUltrasonicReadingCM = NO_ULTRASONIC_SENSOR_PRESENT;
            gLastUltrasonicReadTimeMS = curTime;
        }
    }
    
    // Check the status of an ongoing ultrasonic read
    if ( gbTakingUltrasonicSensorReading
        && digitalRead( ULTRASONIC_PIN ) == LOW )
    {
        // Pulse has ended
        unsigned long durationUS = micros() - gUltrasonicPulseStartTimeUS;
        
        gLastUltrasonicReadingCM = durationUS/ULTRASONIC_US_PER_CM;
        if ( gLastUltrasonicReadingCM > MAX_ULTRASONIC_RANGE_CM )
        {
            gLastUltrasonicReadingCM = MAX_ULTRASONIC_RANGE_CM;
        }
        
        gLastUltrasonicReadTimeMS = curTime;
        gbTakingUltrasonicSensorReading = false;
    }
    
    // Periodically read from the sensors and send the readings back
    if ( curTime - gLastSensorReadTimeMS >= SENSOR_READ_INTERVAL_MS )
    {
        // Read in the battery voltage
        int batteryReading = analogRead( BATTERY_VOLTAGE_PIN );
        
        uint8_t digitalReadings = 0;
        int analogReadings[ 6 ] = { 0 };
        
        // Check each of the 8 sensor pins in turn. Either use them for a digital read, or
        // for their alternative function.
        if ( gSensorConfigurationA & (1 << 0) )
        {
            digitalReadings |= (( digitalRead( 12 ) == HIGH ? 1 : 0 ) << 0);
        }
        else
        {
            // Ultrasonic reads are being carried out on this pin
        }
        
        if ( gSensorConfigurationA & (1 << 1) )
        {
            digitalReadings |= (( digitalRead( 13 ) == HIGH ? 1 : 0 ) << 1);
        }
        else
        {
            // No alternative function for D13
        }
        
        for ( int i = 0; i < 6; i++ )
        {
            if ( gSensorConfigurationA & (1 << (2 + i)) )
            {
                digitalReadings |= (( digitalRead( A0 + i ) == HIGH ? 1 : 0 ) << (2 + i));
            }
            else
            {
                analogReadings[ i ] = analogRead( A0 + i );
            }
        }
        
        sendSensorReadingsMessage( batteryReading, digitalReadings, 
            analogReadings, gLastUltrasonicReadingCM, gLeftEncoder.read(), gRightEncoder.read() );
        
        gLastSensorReadTimeMS = curTime;
    }
}

//------------------------------------------------------------------------------
void receiveMessages()
{
    bool bMessageReceived = false;
    int numBytesAvailable = Serial.available();
    
    while ( numBytesAvailable > 0 && !bMessageReceived )
    {
        switch ( gMessageState )
        {
            case eMS_WaitingForMessage:
            {
                int numBytesToRead = max( min( numBytesAvailable, MSG_HEADER_SIZE - gNumMsgBytesReceived ), 0 );
                int numBytesRead = Serial.readBytes( (char*)&gMsgBuffer[ gNumMsgBytesReceived ], numBytesToRead );
                gNumMsgBytesReceived += numBytesRead;
                                
                if ( MSG_HEADER_SIZE == gNumMsgBytesReceived )
                {
                    if ( MSG_START_BYTES == *((uint16_t*)gMsgBuffer) )
                    {
                        // We have a message header
                        gMessageState = eMS_ReceivingMessage;
                    }
                    else
                    {
                        // Discard the first byte as it is not part of a message
                        gMsgBuffer[ 0 ] = gMsgBuffer[ 1 ];
                        gMsgBuffer[ 1 ] = gMsgBuffer[ 2 ];
                        gMsgBuffer[ 2 ] = gMsgBuffer[ 3 ];
                        gNumMsgBytesReceived = 3;
                    }
                }
                
                break;
            }
            case eMS_ReceivingMessage:
            {
                int numBytesToRead = max( min( numBytesAvailable, getMessageSize() - gNumMsgBytesReceived ), 0 );
                int numBytesRead = Serial.readBytes( (char*)&gMsgBuffer[ gNumMsgBytesReceived ], numBytesToRead );
                gNumMsgBytesReceived += numBytesRead;
                
                if ( getMessageSize() == gNumMsgBytesReceived )
                {
                    processMessage();
                    bMessageReceived = true;
                    
                    // Prepare for next message
                    gNumMsgBytesReceived = 0;
                    gMessageState = eMS_WaitingForMessage;
                }
                
                break;
            }
            default:
            {
                // We should never get here, but just in case, return to eMS_WaitingForMessage
                gNumMsgBytesReceived = 0;
                gMessageState = eMS_WaitingForMessage;
            }
        }
        
        numBytesAvailable = Serial.available();
    }
}

//------------------------------------------------------------------------------
void processMessage()
{
    // Check the checksum of the message
    uint8_t calculatedCheckSum = calculateCheckSum( gMsgBuffer, getMessageSize() );
    
    if ( calculatedCheckSum != gMsgBuffer[ getMessageSize() - 1 ] )
    {
        sendInvalidCheckSumResponse();
    }
    
    // Handle the command Id
    bool bCommandHandled = false;
    switch ( getMessageId() )
    {
        case COMMAND_ID_GET_FIRMWARE_INFO:
        {
            sendFirmwareInfoResponse();
            bCommandHandled = true;
            
            break;
        }
        case COMMAND_ID_SET_OUTPUTS:
        {
            if ( getMessageSize() == 9 )
            {
                int leftMotorSpeed = (int)gMsgBuffer[ 4 ] - 128;
                int rightMotorSpeed = (int)gMsgBuffer[ 5 ] - 128;
                gPanServoAngle = constrain( gMsgBuffer[ 6 ], 0, 180 );
                gTiltServoAngle = constrain( gMsgBuffer[ 7 ], 0, 180 );
            
                gLeftMotorDirection = ( leftMotorSpeed >= 0 ? eMD_Forwards : eMD_Backwards );
                gRightMotorDirection = ( rightMotorSpeed >= 0 ? eMD_Forwards : eMD_Backwards );
                gLeftMotorDutyCycle = constrain( abs( leftMotorSpeed ), 0, 100 );
                gRightMotorDutyCycle = constrain( abs( rightMotorSpeed ), 0, 100 );
                gLastCommandTime = millis();
            
                bCommandHandled = true;
            }
        
            break;
        }
        case COMMAND_ID_SET_PAN_SERVO_LIMITS:
        case COMMAND_ID_SET_TILT_SERVO_LIMITS:
        {
            if ( getMessageSize() == 9 )
            {
                uint16_t servoMin = gMsgBuffer[ 4 ] << 8 | gMsgBuffer[ 5 ];
                uint16_t servoMax = gMsgBuffer[ 6 ] << 8 | gMsgBuffer[ 7 ];
                
                if ( getMessageId() == COMMAND_ID_SET_PAN_SERVO_LIMITS )
                {                   
                    gPanServoLimits.setLimits( servoMin, servoMax );
                }
                else
                {
                    gTiltServoLimits.setLimits( servoMin, servoMax );
                }
                
                bCommandHandled = true;
            }
            break;
        }
        case COMMAND_ID_SET_SENSOR_CONFIGURATION:
        {
            if ( getMessageSize() == 7 )
            {
                gSensorConfigurationA = gMsgBuffer[ 4 ];
                gSensorConfigurationB = gMsgBuffer[ 5 ];
                
                gLeftEncoder.setSingleOutput( gSensorConfigurationB & LENC_SINGLE_BITMASK );
                gRightEncoder.setSingleOutput( gSensorConfigurationB & RENC_SINGLE_BITMASK );
                
                bCommandHandled = true;
            }
            break;
        }
    }
    
    if ( !bCommandHandled )
    {
        sendInvalidCommandResponse();
    }
}

//------------------------------------------------------------------------------
void sendFirmwareInfoResponse()
{
    uint8_t msgData[] = 
    {
        0xFF, 0xFF, RESPONSE_ID_FIRMWARE_INFO, 0,   // Header
        FIRMWARE_ID >> 8,  FIRMWARE_ID & 0xFF, 
        VERSION_MAJOR, VERSION_MINOR, 0
    };
    
    const uint8_t MSG_SIZE = sizeof( msgData );
    msgData[ 3 ] = MSG_SIZE;
    msgData[ MSG_SIZE - 1 ] = calculateCheckSum( msgData, MSG_SIZE );
    
    Serial.write( msgData, MSG_SIZE );
}

//------------------------------------------------------------------------------
void sendInvalidCommandResponse()
{
    uint8_t msgData[] = 
    {
        0xFF, 0xFF, RESPONSE_ID_INVALID_COMMAND, 0, 0
    };
    
    const uint8_t MSG_SIZE = sizeof( msgData );
    msgData[ 3 ] = MSG_SIZE;
    msgData[ MSG_SIZE - 1 ] = calculateCheckSum( msgData, MSG_SIZE );
    
    Serial.write( msgData, MSG_SIZE );
}

//------------------------------------------------------------------------------
void sendInvalidCheckSumResponse()
{
    uint8_t msgData[] = 
    {
        0xFF, 0xFF, RESPONSE_ID_INVALID_CHECK_SUM, 0, 0
    };
    
    const uint8_t MSG_SIZE = sizeof( msgData );
    msgData[ 3 ] = MSG_SIZE;
    msgData[ MSG_SIZE - 1 ] = calculateCheckSum( msgData, MSG_SIZE );
    
    Serial.write( msgData, MSG_SIZE );
}

//------------------------------------------------------------------------------
void sendSensorReadingsMessage( int batteryReading, uint8_t digitalReadings, 
    int analogReadings[ 6 ], int gLastUltrasonicReadingCM,
    int32_t leftEncoderReading, int32_t rightEncoderReading )
{    
    uint8_t msgData[] = 
    {
        0xFF, 0xFF, RESPONSE_ID_SENSOR_READINGS, 0, // Header
        gSensorConfigurationA, gSensorConfigurationB,
        (batteryReading >> 8) & 0xFF, batteryReading & 0xFF,
        digitalReadings,
        (analogReadings[ 0 ] >> 8) & 0xFF, analogReadings[ 0 ] & 0xFF,
        (analogReadings[ 1 ] >> 8) & 0xFF, analogReadings[ 1 ] & 0xFF,
        (analogReadings[ 2 ] >> 8) & 0xFF, analogReadings[ 2 ] & 0xFF,
        (analogReadings[ 3 ] >> 8) & 0xFF, analogReadings[ 3 ] & 0xFF,
        (analogReadings[ 4 ] >> 8) & 0xFF, analogReadings[ 4 ] & 0xFF,
        (analogReadings[ 5 ] >> 8) & 0xFF, analogReadings[ 5 ] & 0xFF,
        (gLastUltrasonicReadingCM >> 8) & 0xFF, gLastUltrasonicReadingCM & 0xFF,
        (leftEncoderReading >> 24) & 0xFF, (leftEncoderReading >> 16) & 0xFF, (leftEncoderReading >> 8) & 0xFF, leftEncoderReading & 0xFF,
        (rightEncoderReading >> 24) & 0xFF, (rightEncoderReading >> 16) & 0xFF, (rightEncoderReading >> 8) & 0xFF, rightEncoderReading & 0xFF,
        0
    };
    
    const uint8_t MSG_SIZE = sizeof( msgData );
    msgData[ 3 ] = MSG_SIZE;
    msgData[ MSG_SIZE - 1 ] = calculateCheckSum( msgData, MSG_SIZE );
    
    Serial.write( msgData, MSG_SIZE );
}

//------------------------------------------------------------------------------
uint8_t calculateCheckSum( const uint8_t* pData, uint8_t msgSize )
{
    uint32_t sum = 0;
    
    // Use all of the data apart from the message start bytes and the byte
    // that will store the checksum
    for ( uint8_t i = 2; i < msgSize - 1; i++ )
    {
        sum += pData[ i ];
    }
    
    return (uint8_t)(~sum);
}

//------------------------------------------------------------------------------
ISR( TIMER2_COMP_vect )        // Interrupt service routine for motor PWM
{
    gCurMotorWaveTick++;
    
    digitalWrite( LEFT_MOTOR_PWM_PIN, ( gCurMotorWaveTick > gLeftMotorDutyCycle ? LOW : HIGH ) );
    digitalWrite( RIGHT_MOTOR_PWM_PIN, ( gCurMotorWaveTick > gRightMotorDutyCycle ? LOW : HIGH ) );
    
    if ( gCurMotorWaveTick >= 100 )
    {
        gCurMotorWaveTick = 0;
    }
}

