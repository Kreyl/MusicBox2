/*
 * led.cpp
 *
 *  Created on: 9 ���. 2016 �.
 *      Author: Kreyl
 */

#include "led.h"

#ifdef LED_Smooth
void LedSmoothTmrCallback(void *p) {
    chSysLockFromISR();
    ((LedSmooth_t*)p)->IProcessSequenceI();
    chSysUnlockFromISR();
}
#endif
#ifdef LED_RGB_Parent
void LedRGBTmrCallback(void *p) {
    chSysLockFromISR();
    ((LedRGB_t*)p)->IProcessSequenceI();
    chSysUnlockFromISR();
}
#endif
