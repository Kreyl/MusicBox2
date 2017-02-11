/*
 * SteppingMotor.cpp
 *
 *  Created on: 05 февр. 2017 г.
 *      Author: Elessar
 */

#include "SteppingMotor.h"

void StepperTmrCallback(void *p) {
    reinterpret_cast<SteppingMotor_t*>(p)->StepperTmrCallbacHandler();
}


void SteppingMotor_t::SetSpeed(int32_t Speed, const StepMode_t Mode = smLikeBefore) {     // [об/мин * 0,1]
//    Start();

    if (Speed > 0) Rotation = srClockwise;
    else if (Speed < 0) { Rotation = srReverse; Speed *= -1; }

    MotorMode = Mode;
    if (Speed != 0){
        switch(MotorMode) {
            case smFullStep:
                Steps_CNT = 3;
                StepInterval = (uint32_t)(1000*60*StepAngle*10)/(360 * GearRatio * Speed);
                break;
            case smHalftep:
                Steps_CNT = 7;
                StepInterval = (uint32_t)(1000*60*StepAngle*5)/(360 * GearRatio * Speed);
                break;
            default: break;
        }         // обороты в минуту /10 -> интервал между шагами [mS]
    }
    if (StepInterval == 0) StepInterval = 1;
    Uart.Printf("\r Step Interval %u mS", StepInterval);
}

void SteppingMotor_t::Task() {
//    uint16_t temp = PortGetValue(IMotor.PGpio);
//    temp = temp | (FullStepOperation[StepIndex] << PinA1);
//    PortSetValue(IMotor.PGpio, temp);
//    Uart.PrintfI("\r Motor Step");
    if (MotorMode == smFullStep) {
        switch(StepIndex) {
            case 0: // 0b1010
                PinSet  (IMotor.PGpio, IMotor.PinA1);
                PinClear(IMotor.PGpio, IMotor.PinA2);
                PinSet  (IMotor.PGpio, IMotor.PinB1);
                PinClear(IMotor.PGpio, IMotor.PinB2);
                break;
            case 1: // 0b0110
                PinClear(IMotor.PGpio, IMotor.PinA1);
                PinSet  (IMotor.PGpio, IMotor.PinA2);
                PinSet  (IMotor.PGpio, IMotor.PinB1);
                PinClear(IMotor.PGpio, IMotor.PinB2);
                break;
            case 2: // 0b0101
                PinClear(IMotor.PGpio, IMotor.PinA1);
                PinSet  (IMotor.PGpio, IMotor.PinA2);
                PinClear(IMotor.PGpio, IMotor.PinB1);
                PinSet  (IMotor.PGpio, IMotor.PinB2);
                break;
            case 3: // 0b1001
                PinSet  (IMotor.PGpio, IMotor.PinA1);
                PinClear(IMotor.PGpio, IMotor.PinA2);
                PinClear(IMotor.PGpio, IMotor.PinB1);
                PinSet  (IMotor.PGpio, IMotor.PinB2);
                break;
        }
    } else if (MotorMode == smHalftep) {
        switch(StepIndex) {
            case 0: // 0b1010
                PinSet  (IMotor.PGpio, IMotor.PinA1);
                PinClear(IMotor.PGpio, IMotor.PinA2);
                PinSet  (IMotor.PGpio, IMotor.PinB1);
                PinClear(IMotor.PGpio, IMotor.PinB2);
                break;
            case 1: // 0b0010
                PinClear(IMotor.PGpio, IMotor.PinA1);
                PinClear(IMotor.PGpio, IMotor.PinA2);
                PinSet  (IMotor.PGpio, IMotor.PinB1);
                PinClear(IMotor.PGpio, IMotor.PinB2);
                break;
            case 2: // 0b0110
                PinClear(IMotor.PGpio, IMotor.PinA1);
                PinSet  (IMotor.PGpio, IMotor.PinA2);
                PinSet  (IMotor.PGpio, IMotor.PinB1);
                PinClear(IMotor.PGpio, IMotor.PinB2);
                break;
            case 3: // 0b0100
                PinClear(IMotor.PGpio, IMotor.PinA1);
                PinSet  (IMotor.PGpio, IMotor.PinA2);
                PinClear(IMotor.PGpio, IMotor.PinB1);
                PinClear(IMotor.PGpio, IMotor.PinB2);
                break;
            case 4: // 0b0101
                PinClear(IMotor.PGpio, IMotor.PinA1);
                PinSet  (IMotor.PGpio, IMotor.PinA2);
                PinClear(IMotor.PGpio, IMotor.PinB1);
                PinSet  (IMotor.PGpio, IMotor.PinB2);
                break;
            case 5: // 0b0001
                PinClear(IMotor.PGpio, IMotor.PinA1);
                PinClear(IMotor.PGpio, IMotor.PinA2);
                PinClear(IMotor.PGpio, IMotor.PinB1);
                PinSet  (IMotor.PGpio, IMotor.PinB2);
                break;
            case 6: // 0b1001
                PinSet  (IMotor.PGpio, IMotor.PinA1);
                PinClear(IMotor.PGpio, IMotor.PinA2);
                PinClear(IMotor.PGpio, IMotor.PinB1);
                PinSet  (IMotor.PGpio, IMotor.PinB2);
                break;
            case 7: // 0b1000
                PinSet  (IMotor.PGpio, IMotor.PinA1);
                PinClear(IMotor.PGpio, IMotor.PinA2);
                PinClear(IMotor.PGpio, IMotor.PinB1);
                PinClear(IMotor.PGpio, IMotor.PinB2);
                break;
        }
    }

//    Uart.PrintfI("\r StepIndex %u", StepIndex);

    switch(Rotation) {
        case srClockwise:
            if (StepIndex < Steps_CNT)
                StepIndex ++;
            else
                StepIndex = 0;
            break;
        case srReverse:
            if (StepIndex > 0)
                StepIndex --;
            else
                StepIndex = Steps_CNT;
            break;
    }
}




