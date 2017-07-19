/*
 * File:   main.cpp
 * Author: Elessar
 * Project: MasonOrder
 *
 * Created on May 27, 2016, 6:37 PM
 */

#include "main.h"
#include "sound.h"
#include "Soundlist.h"
#include "SimpleSensors.h"
#include "buttons.h"
#include "led.h"
#include "Sequences.h"
#include "SteppingMotor.h"
//#include "WS2812BforSPI.h"
//#include "WS2812BforTMR.h"

App_t App;
SndList_t SndList;
Periphy_t Periphy;
//PinInput_t WKUPpin(WKUP_pin);
PinInput_t ExternalPWR(ExternalPWR_Pin);
PinInput_t Box1Opened(Sensor1_Pin);
PinInput_t Box2Opened(Sensor2_Pin);
PinOutput_t BattMeasureSW(BattMeasSW_Pin);
SteppingMotor_t Motor(MotorPins, MotorSHDN, MotorAngle, MotorRatio);
LedSmooth_t Backlight(LED_PIN);

// =============================== Main ========================================
FIL MyFile;

int main() {
    // ==== Init ====
    // ==== Setup clock ====
    Clk.UpdateFreqValues();
    uint8_t ClkResult = retvFail;
    Clk.SetupFlashLatency(24);  // Setup Flash Latency for clock in MHz
    // 12 MHz/6 = 2; 2*192 = 384; 384/8 = 48 (preAHB divider); 384/8 = 48 (USB clock)
    Clk.SetupPLLDividers(6, 192, pllSysDiv8, 8);
    // 48/4 = 12 MHz core clock. APB1 & APB2 clock derive on AHB clock
    Clk.SetupBusDividers(ahbDiv4, apbDiv1, apbDiv1);    // for 24MHz work: ahbDiv2, apbDiv1, apbDiv1
    if((ClkResult = Clk.SwitchToPLL()) == 0) Clk.HSIDisable();
    Clk.UpdateFreqValues();

    // ==== Init OS ====
    halInit();
    chSysInit();
    App.InitThread();

    // ==== Init Hard & Soft ====
    Uart.Init(115200, UART_GPIO, UART_TX_PIN, UART_GPIO, UART_RX_PIN);
    Uart.Printf("\r%S %S\r", APP_NAME, BUILD_TIME);
    Clk.PrintFreqs();
    // Report problem with clock if any
    if(ClkResult) Uart.Printf("Clock failure\r");

    // Battery: ADC
    PinSetupAnalog(BattMeas_Pin);
    BattMeasureSW.Init();
    BattMeasureSW.SetHi();
    Adc.Init();
    Adc.EnableVref();

    // Setup inputs
    SimpleSensors::Init();
    // Setup outputs
    Periphy.InitSwich();


    App.PowerON();
    // USB related
    MassStorage.Init();


//    FRESULT Rslt = f_open(&MyFile, "test.txt", FA_READ+FA_OPEN_EXISTING);
//    Uart.Printf("Rslt: %u\r", Rslt);

    // ==== Main cycle ====
    App.ITask();
}


void App_t::PowerON() {
    Periphy.ON();
    chThdSleepMilliseconds(200);    // Let power to stabilize
    // Stepping Motor
    Motor.Init(pdNoDelay);
    SD.Init();      // No power delay
    // Sound
    Sound.Init(chThdGetSelfX());
    // LED
//    Backlight.Init();
//    Backlight.SetBrightness(0);
//    Backlight.SetPwmFrequencyHz(1000);
    App.LoadSettings("Settings.ini");

    if (Box1Opened.IsHi() or Box2Opened.IsHi()) {
        SndList.PlayRandomFileFromDir(PlayDir);
//        Backlight.StartOrContinue(lsqFadeIn);
    }
    else if (ExternalPWR.IsHi()) SignalEvt(EVT_USB_CONNECTED);
    else ShutDown();
}


void App_t::LoadSettings(const char* SettingsFileName) {
    // Load Sound Settings
    uint8_t VolLevel = 0;
    if(Sleep::WasInStandby()) {
        Uart.Printf("WasInStandby\r");
        Sleep::DisableWakeupPin();
        Sleep::ClearStandbyFlag();
        SndList.SetPreviousTrack(PlayDir, BackupSpc::ReadBackupRegister(TrackNumberBKP));
//        Uart.Printf("\r Load TrackNumber: %u", BackupSpc::ReadBackupRegister(TrackNumberBKP));
        Sound.SetVolume(BackupSpc::ReadBackupRegister(VolumeBKP));
        BackupSpc::DisableAccess();
    }
    else {
        Uart.Printf("PowerON\r");
        if (iniRead(SettingsFileName, "Sound", "DefVolume", &VolLevel) == retvOk)
            Sound.SetVolume(VolLevel);
        else {
            Uart.Printf("Sound <- Def VolLevel\r");
            Sound.SetVolume(DEF_VolLevel);
        }
    }

    // Load Motor Settings
    int32_t Speed = 0;
    if (iniRead(SettingsFileName, "Motor", "Speed", &Speed) == retvOk) {
        if (ABS(Speed) > 100) {
            Motor.SetSpeed(Speed, smHalftep);
            Motor.Start();
        } else
            Motor.Sleep();
    } else {
        Uart.Printf("Motor <- Def Speed\r");
        Motor.SetSpeed(DEF_MotorSpeed, smHalftep);   // smHalftep / smFullStep
        Motor.Start();
    }
}


__attribute__ ((__noreturn__))
void App_t::ITask() {

#if 0 // Создать файл (проба)
    FIL File;
    FRESULT Rslt = f_open(&File, "ID_Store.ini",FA_READ+FA_OPEN_EXISTING+FA_CREATE_NEW);
    if(Rslt != FR_OK) {
        if (Rslt == FR_NO_FILE) Uart.Printf("\r%S: not found", "ID_Store.ini");
        else Uart.Printf("\r%S: openFile error: %u", "ID_Store.ini", Rslt);
     }
#endif

while(true) {
//    eventmask_t EvtMsk = chEvtWaitAnyTimeout(EVT_PLAY_ENDS, MS2ST(306));
    eventmask_t EvtMsk = chEvtWaitAny(ALL_EVENTS);

    if(EvtMsk & EVT_PLAY_ENDS) {
        if (!ExternalPWR.IsHi()){
//        if (!Periphy._5V_is_here()){
            SndList.PlayRandomFileFromDir(PlayDir);
        }
    }

    if(EvtMsk & EVT_BUTTONS) {
//        Uart.Printf("BtnsEvt\r");
        BtnEvtInfo_t EInfo;
        while(BtnGetEvt(&EInfo) == retvOk) {
            if(EInfo.Type == beShortPress) {
//                Uart.Printf("Btn %u press\r", EInfo.BtnID);
                switch(EInfo.BtnID) {
                    case VolUpIndex: Sound.VolumeIncrease(); break;
                    case VolDownIndex: Sound.VolumeDecrease(); break;
                }
            }
            else if(EInfo.Type == beRepeat) {
//                Uart.Printf("Btn %u repeat\r", EInfo.BtnID);
                switch(EInfo.BtnID) {
                    case VolUpIndex:   Sound.VolumeIncreaseBig(); break;
                    case VolDownIndex: Sound.VolumeDecreaseBig(); break;
                }
            }
        }
    }

    if(EvtMsk & EVT_BOX1_CLOSED) {
        if (!Box2Opened.IsHi() and !ExternalPWR.IsHi()){
            ShutDown();
        }
    }
    if(EvtMsk & EVT_BOX2_CLOSED) {
        if (!Box1Opened.IsHi() and !ExternalPWR.IsHi()){
            ShutDown();
        }
    }

    if(EvtMsk & EVT_ADC_DONE) {
        uint16_t BatAdc = 2 * (Adc.GetResult(BAT_CHNL) - CallConst); // to count R divider
        uint8_t NewBatPercent = mV2PercentLiIon(BatAdc);
        Uart.Printf("mV=%u; percent=%u\r", BatAdc, NewBatPercent);
//        Adc.DisableVref();
    }

 // ==== USB connected/disconnected ====
    if(EvtMsk & EVT_USB_CONNECTED) {
        Sound.Stop();
        Motor.Stop();
        Backlight.SetBrightness(0);
        chSysLock();
        Clk.SetFreq48Mhz();
        chSysUnlock();
        Usb.Init();
        chThdSleepMilliseconds(540);
        Usb.Connect();
        Uart.Printf("Usb On\r");
        Clk.PrintFreqs();
    }
    if(EvtMsk & EVT_USB_DISCONNECTED) {
        Usb.Shutdown();
        MassStorage.Reset();
        chSysLock();
        Clk.SetFreq12Mhz();
        chSysUnlock();
        Uart.Printf("Usb Off\r");
        Clk.PrintFreqs();
        if (!Box1Opened.IsHi() and !Box2Opened.IsHi())
            ShutDown();
        else{
            SndList.UpdateDir(PlayDir);
            SndList.PlayRandomFileFromDir(PlayDir);
            Motor.Start();
//            Backlight.StartOrContinue(lsqFadeIn);
        }
    }

    if(EvtMsk & EVT_UART_NEW_CMD) {
        OnCmd(&Uart);
        Uart.SignalCmdProcessed();
    }

    } // while true
}

void App_t::OnCmd(Shell_t *PShell) {
    Cmd_t *PCmd = &PShell->Cmd;
    __attribute__((unused)) int32_t Data = 0;  // May be unused in some configurations
    Uart.Printf("\r New Cmd: %S\r", PCmd->Name);
    // Handle command
    if(PCmd->NameIs("Ping")) {
        PShell->Ack(retvOk);
    }
    else if(PCmd->NameIs("Next")) {
        SndList.PlayRandomFileFromDir(PlayDir);
        PShell->Ack(retvOk);
    }

    else if(PCmd->NameIs("PerON")) {
        PowerON();
        PShell->Ack(retvOk);
    }
    else if(PCmd->NameIs("PerOFF")) {
        Sound.Shutdown();
        Periphy.OFF();
        PShell->Ack(retvOk);
    }

    else if(PCmd->NameIs("MotorSetF")) {
        if(PCmd->GetNextInt32(&Data) == retvOk) {
            Uart.Printf("Data=%d\r ", Data);
            Motor.SetSpeed(Data, smFullStep);
        }
        PShell->Ack(retvOk);
    }
    else if(PCmd->NameIs("MotorSetH")) {
        if(PCmd->GetNextInt32(&Data) == retvOk) {
            Uart.Printf("Data=%d\r ", Data);
            Motor.SetSpeed(Data, smHalftep);
        }
        PShell->Ack(retvOk);
    }
    else if(PCmd->NameIs("MotorStart")) {
        Motor.Start();
        PShell->Ack(retvOk);
    }
    else if(PCmd->NameIs("MotorStop")) {
        Motor.Stop();
        PShell->Ack(retvOk);
    }

    else PShell->Ack(retvCmdUnknown);
}


void App_t::ShutDown() {
    Sound.Stop();
    Sound.Shutdown();
    Motor.Stop();
    chThdSleepMilliseconds(700);
//    if (!WKUPpin.IsHi()){
    if (!Box1Opened.IsHi() and !Box2Opened.IsHi() and !ExternalPWR.IsHi()){
        chSysLock();
        Uart.PrintfNow("Sleep\r\r");
        Sound.Shutdown();
//        Backlight.SetBrightness(0);
//        Motor.Sleep();
        Periphy.OFF();
        BackupSpc::EnableAccess();
        BackupSpc::WriteBackupRegister(TrackNumberBKP, SndList.GetTrackNumber(PlayDir));
        BackupSpc::WriteBackupRegister(VolumeBKP, Sound.GetVolume());
        Sleep::EnableWakeupPin();
        Sleep::EnterStandby();
        chSysUnlock();
    }
    else {
        Motor.Start();
        SndList.PlayRandomFileFromDir(PlayDir);
    }
}

// Snsors
void Process5VSns(PinSnsState_t *PState, uint32_t Len) {
//    Uart.Printf("  %S\r", __FUNCTION__);
    if(PState[0] == pssRising) App.SignalEvt(EVT_USB_CONNECTED);
    else if(PState[0] == pssFalling) App.SignalEvt(EVT_USB_DISCONNECTED);
}
void Process3VSns1(PinSnsState_t *PState, uint32_t Len) {
//    Uart.Printf("  %S\r", __FUNCTION__);
    if(PState[0] == pssFalling) App.SignalEvt(EVT_BOX1_CLOSED);
}
void Process3VSns2(PinSnsState_t *PState, uint32_t Len) {
//    Uart.Printf("  %S\r", __FUNCTION__);
    if(PState[0] == pssFalling) App.SignalEvt(EVT_BOX2_CLOSED);
}
