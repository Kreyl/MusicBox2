/*
 * SteppingMotor.h
 *
 *  Created on: 05 ����. 2017 �.
 *      Author: Elessar
 */

#ifndef STEPPINGMOTOR_H_
#define STEPPINGMOTOR_H_

#include "board.h"
#include "uart.h"
#include "kl_lib.h"

//const uint8_t FullStepOperation[] = {
//        0b1010,
//        0b0110,
//        0b0101,
//        0b1001,
////        0b0000,
//};
//#define FullStep_CNT    countof(FullStepOperation)
//#define HalfStep_CNT    8

struct MotorSetupPins_t {
    GPIO_TypeDef *PGpio;
    uint8_t PinA1;
    uint8_t PinA2;
    uint8_t PinB1;
    uint8_t PinB2;
};

enum PowerDelay_t {pdDelay = true, pdNoDelay = false};
enum StepMode_t {smLikeBefore, smFullStep, smHalftep};
enum Rotation_t {srClockwise, srReverse};

void StepperTmrCallback(void *p);    // VirtualTimer callback


class SteppingMotor_t {
private:
    virtual_timer_t StepTMR;
    uint8_t StepIndex = 0;
    systime_t StepInterval = 30;
    uint8_t Steps_CNT = 3;
    StepMode_t MotorMode = smFullStep;
    Rotation_t Rotation = srClockwise;

    const MotorSetupPins_t IMotor;
    uint8_t PinSHDN;
    uint8_t StepAngle;
    uint16_t GearRatio;

    void StepperTmrStsrtI(){
        if(chVTIsArmedI(&StepTMR)) chVTResetI(&StepTMR);
        chVTSetI(&StepTMR, MS2ST(StepInterval), StepperTmrCallback, this);
    }
    void TaskI();

public:
    SteppingMotor_t( MotorSetupPins_t AMotor, uint8_t APinSHDN, uint8_t AStepAngle, uint16_t AGearRatio) :
        IMotor(AMotor), PinSHDN(APinSHDN), StepAngle(AStepAngle), GearRatio(AGearRatio) {}

    void StepperTmrCallbacHandler() {
        chSysLockFromISR();
        StepperTmrStsrtI();
        TaskI();
        chSysUnlockFromISR();
    }
    void Init(const PowerDelay_t Delay = pdDelay) {
        PinSetupOut(IMotor.PGpio, IMotor.PinA1, omPushPull);
        PinSetupOut(IMotor.PGpio, IMotor.PinA2, omPushPull);
        PinSetupOut(IMotor.PGpio, IMotor.PinB1, omPushPull);
        PinSetupOut(IMotor.PGpio, IMotor.PinB2, omPushPull);
        PinSetupOut(IMotor.PGpio, PinSHDN, omPushPull);
        PinSetHi(IMotor.PGpio, PinSHDN);
        if (Delay) chThdSleepMicroseconds(1500);
    }
    void Start() {
        chSysLock();
        StepperTmrStsrtI();
        chSysUnlock();
    }
    void SetSpeed(int32_t Speed, const StepMode_t Mode);
    int32_t GetSpeed() {
        int32_t Result;
        switch(MotorMode) {
            case smFullStep:
                Result = (1000*60*StepAngle*5)/(360 * GearRatio * StepInterval);
                break;
            case smHalftep:
                Result = (1000*60*StepAngle*10)/(360 * GearRatio * StepInterval);
                break;
            default: break;
        }
        if (Rotation == srReverse) return -Result;
        else return Result;
    }
    void Stop() {
        chVTReset(&StepTMR);
        StepIndex = 0;
        PinSetLo(IMotor.PGpio, IMotor.PinA1);
        PinSetLo(IMotor.PGpio, IMotor.PinA2);
        PinSetLo(IMotor.PGpio, IMotor.PinB1);
        PinSetLo(IMotor.PGpio, IMotor.PinB2);
    }
    void Sleep() {
        Stop();
        PinSetLo(IMotor.PGpio, PinSHDN);
    }
};

#endif /* STEPPINGMOTOR_H_ */
