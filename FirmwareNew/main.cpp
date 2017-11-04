/*
 * File:   main.cpp
 * Author: Elessar
 * Project: MasonOrder
 *
 * Created on May 27, 2016, 6:37 PM
 */

#include "main.h"
#include "SimpleSensors.h"
#include "buttons.h"
#include "kl_adc.h"
#include "led.h"
#include "Sequences.h"
#include "sound.h"
#include "Soundlist.h"
#include "SteppingMotor.h"
#include "RotaryDial.h"
#include "ws2812b.h"
#include "LEDs.h"


#if 1 // ======================== Variables and defines ========================
App_t App;
SndList_t SndList;
Periphy_t Periphy;
//PinInput_t WKUPpin(WKUP_pin);
PinInput_t ExternalPWR(ExternalPWR_Pin);
PinInput_t Box1Opened(Sensor1_Pin);
#if defined MusicBox
PinInput_t Box2Opened(Sensor2_Pin);
#elif defined Phone
PinInput_t Box2Opened(Sensor1_Pin);
#endif
PinOutput_t BattMeasureSW(BattMeasSW_Pin);
SteppingMotor_t Motor(MotorPins, MotorSHDN, MotorAngle, MotorRatio);
LedSmooth_t Backlight(LED_PIN);
TmrKL_t TmrOFF { MS2ST(OFF_delay_MS), EVT_OFF_TimeOut, tktOneShot };
TmrKL_t TmrWait { EVT_WAIT_TimeOut, tktOneShot };
LEDs_t LEDs;
//Dialer_t Dialer;


enum AppState_t {
    asOff, asPlay, asStop,   // all
    asBeep, asProcNumber, asWaiting, asSecondStop   // Phone
};
AppState_t State = asOff;

void BtnHandler(BtnEvt_t BtnEvt, uint8_t BtnID);
void LoadSettings(const char* FileName);
void WakeUp();
void ShutDown();

#endif

// =============================== Main ========================================

int main() {
#if 1  // ==== Init ====
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
    Uart.Printf("LEDs CNT %u\r", LED_CNT);
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
    // Periphy
    Periphy.InitSwich();
    Periphy.ON();
    chThdSleepMilliseconds(200);    // Let power to stabilize

    if (ExternalPWR.IsHi()) App.SignalEvt(EVT_USB_CONNECTED);
    else if (!Box1Opened.IsHi() and !Box2Opened.IsHi()) ShutDown();

    // Random
    Random::TrueInit();

    // SD
    SD.Init();      // No power delay
    LoadSettings("Settings.ini");

    // USB related
    MassStorage.Init();

    // Sound
    Sound.Init();
    Sound.SetupSeqEndEvt(EVT_PLAY_ENDS);

    // Timers
    TmrOFF.Init();

#if defined MusicBox
    // LEDs
    LEDs.Init();
    LEDs.SetupSeqEndEvt(EVT_LED_DONE);
    LEDs.SetProfile(DEF_LEDsProf);
    LEDs.Start();
    LEDs.SetAll(StartIntensity, StartProcessTime, StartPause);
    LEDs.GenerationParam();
//    Backlight.Init();
//    Backlight.SetBrightness(0);
//    Backlight.SetPwmFrequencyHz(1000);
//    Backlight.StartOrContinue(lsqFadeIn);
    // Stepping Motor
    Motor.Init();
    // Play
    State = asPlay;
    SndList.PlayRandomFileFromDir(PlayDir);
#elif defined Phone
    // Timers
    TmrWait.Init();
    // Dialer
    Dialer.Init();
    Dialer.SetupSeqEvents(EVT_DIAL_ARMED, EVT_DIAL_REDY);
    // Play
    State = asBeep;
    Sound.Play(BeepTrack);
#endif

#endif
    // ==== Main cycle ====
    App.ITask();
}


void LoadSettings(const char* SettingsFileName) {
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
#if defined MusicBox
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

    // Load LEDs Settings
    uint8_t Level = 0;
    if (iniRead(SettingsFileName, "RGB_LEDs", "Level R", &Level) == retvOk)
        LEDs.SetIntensityLevel(Level, cpR);
    else LEDs.SetIntensityLevel(DEF_Level_R);
    if (iniRead(SettingsFileName, "RGB_LEDs", "Level G", &Level) == retvOk)
        LEDs.SetIntensityLevel(Level, cpG);
    else LEDs.SetIntensityLevel(DEF_Level_R);
    if (iniRead(SettingsFileName, "RGB_LEDs", "Level B", &Level) == retvOk)
        LEDs.SetIntensityLevel(Level, cpR);
    else LEDs.SetIntensityLevel(DEF_Level_B);
    if (iniRead(SettingsFileName, "RGB_LEDs", "Max Intensity", &Level) == retvOk)
        LEDs.SetLimit_MIN(Level, cpRGB);
    else LEDs.SetLimit_MAX(DEF_Limit_MAX);
    if (iniRead(SettingsFileName, "RGB_LEDs", "Min Intensity", &Level) == retvOk)
        LEDs.SetLimit_MIN(Level, cpRGB);
    else LEDs.SetLimit_MIN(DEF_Limit_MIN);
#endif
}


__attribute__ ((__noreturn__))
void App_t::ITask() {
while(true) {
    eventmask_t EvtMsk = chEvtWaitAny(ALL_EVENTS);

    if(EvtMsk & EVT_PLAY_ENDS) {
//        if (!Periphy._5V_is_here()){
        if (!ExternalPWR.IsHi()) {
#if defined MusicBox
            if (State != asStop)
                SndList.PlayRandomFileFromDir(PlayDir);
#elif defined Phone
            switch(State) {
                case asBeep:
                    Sound.Play(BeepTrack);
                    break;
                case asWaiting:
                    Sound.Play(WaitTrack);
                    break;
                case asPlay:
                    Sound.Play(BusyTrack);
                    break;
                default: break;
            }
#endif
        }
    }

    if(EvtMsk & EVT_LED_DONE) {
#if defined MusicBox
        if (!ExternalPWR.IsHi()) {
            LEDs.GenerationParam();
            LEDs.GenerationParam();
        }
#endif
    }

    if(EvtMsk & EVT_BUTTONS) {
        BtnEvtInfo_t EInfo;
        while(BtnGetEvt(&EInfo) == retvOk) BtnHandler(EInfo.Type, EInfo.BtnID);
    }

    if( (EvtMsk & EVT_BOX1_CLOSED) or (EvtMsk & EVT_BOX2_CLOSED) ) {
        TmrOFF.StartOrRestart();
        Sound.Stop();
        Motor.Stop();
#if defined Phone
        TmrWait.Stop();
        if (State == asBeep)
            State = asSecondStop;
        else
            State = asStop;
#endif
    }
    if( (EvtMsk & EVT_BOX1_OPENED) or (EvtMsk & EVT_BOX2_OPENED) ) {
#if defined MusicBox
        Motor.Start();
        SndList.PlayRandomFileFromDir(PlayDir);
        State = asPlay;
#elif defined Phone
        if (State == asSecondStop) {
            SndList.PlayRandomFileFromDir(PlayDir);
            State = asPlay;
        }
        else {
            Sound.Play(BeepTrack);
            State = asBeep;
        }
#endif
    }
    if(EvtMsk & EVT_OFF_TimeOut) {
//        if (!WKUPpin.IsHi()){
        if (!Box1Opened.IsHi() and !Box2Opened.IsHi() and !ExternalPWR.IsHi())
            ShutDown();
    }

#if defined Phone
    if(EvtMsk & EVT_DIAL_ARMED) {
        if (State != asPlay) {
        State = asProcNumber;
        Sound.Stop();
        }
    }
    if(EvtMsk & EVT_DIAL_REDY) {
        if (!ExternalPWR.IsHi() and State != asPlay) {
            volatile systime_t PlayDelay_MS = Random::Generate(minWait_MS, maxWait_MS);
            Uart.Printf("PlayDelay=%uMS\r", PlayDelay_MS);
            TmrWait.StartOrRestart(MS2ST(PlayDelay_MS));
            Sound.Play(WaitTrack);
            State = asWaiting;
        }
    }
    if(EvtMsk & EVT_WAIT_TimeOut) {
        State = asPlay;
        switch(Dialer.GetNumber()) {
            case 0x1A3: // 103
            case 0xA3:  // 03
                SndList.PlayRandomFileFromDir(Dir_03);
                break;
            default:
                SndList.PlayRandomFileFromDir(Dir_Any);
                break;
        }
    }
#endif

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
        LEDs.SetAll(StopIntensity, StopProcessTime, StopPause);
//        Backlight.SetBrightness(0);
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
            LEDs.SetAll(StartIntensity, StartProcessTime, StartPause);
            LEDs.GenerationParam();
//            Backlight.StartOrContinue(lsqFadeIn);
        }
    }

    if(EvtMsk & EVT_UART_NEW_CMD) {
        OnCmd(&Uart);
        Uart.SignalCmdProcessed();
    }

    } // while true
} // App_t::ITask()

void BtnHandler(BtnEvt_t BtnEvt, uint8_t BtnID) {
//    if(BtnEvt == beShortPress) Uart.Printf("Btn %u Short\r", BtnID);
//    if(BtnEvt == beLongPress)  Uart.Printf("Btn %u Long\r", BtnID);
//    if(BtnEvt == beRelease)    Uart.Printf("Btn %u Release\r", BtnID);
//    if(BtnEvt == beRepeat)    Uart.Printf("Btn %u Repeat\r", BtnID);
//    if(BtnEvt == beClick)      Uart.Printf("Btn %u Click\r", BtnID);
//    if(BtnEvt == beDoubleClick)Uart.Printf("Btn %u DoubleClick\r", BtnID);

    if(BtnEvt == beShortPress) {
        switch(BtnID) {
            case VolUpIndex: Sound.VolumeIncrease(); break;
            case VolDownIndex: Sound.VolumeDecrease(); break;
        }
    }
    else if(BtnEvt == beRepeat) {
        switch(BtnID) {
            case VolUpIndex:   Sound.VolumeIncreaseBig(); break;
            case VolDownIndex: Sound.VolumeDecreaseBig(); break;
        }
    }
}

void ShutDown() {
    chSysLock();
    Uart.PrintfNow("Sleep\r\r");
    Periphy.OFF();
    BackupSpc::EnableAccess();
    BackupSpc::WriteBackupRegister(TrackNumberBKP, SndList.GetTrackNumber(PlayDir));
    BackupSpc::WriteBackupRegister(VolumeBKP, Sound.GetVolume());
    Sleep::EnableWakeupPin();
    Sleep::EnterStandby();
    chSysUnlock();
}

// Snsors
void Process5VSns(PinSnsState_t *PState, uint32_t Len) {
//    Uart.Printf("  %S\r", __FUNCTION__);
    if(PState[0] == pssRising) App.SignalEvt(EVT_USB_CONNECTED);
    else if(PState[0] == pssFalling) App.SignalEvt(EVT_USB_DISCONNECTED);
}
void Process3VSns1(PinSnsState_t *PState, uint32_t Len) {
    if(PState[0] == pssFalling) App.SignalEvt(EVT_BOX1_CLOSED);
    else if (PState[0] == pssRising) App.SignalEvt(EVT_BOX1_OPENED);
}
void Process3VSns2(PinSnsState_t *PState, uint32_t Len) {
#if !defined Phone
    if(PState[0] == pssFalling) App.SignalEvt(EVT_BOX2_CLOSED);
    else if (PState[0] == pssRising) App.SignalEvt(EVT_BOX2_OPENED);
#endif
}

#if UART_RX_ENABLED // ================= Command processing ====================
void App_t::OnCmd(Shell_t *PShell) {
    Cmd_t *PCmd = &PShell->Cmd;
    __attribute__((unused)) int32_t Data = 0;  // May be unused in some configurations
//    Uart.Printf("\r New Cmd: %S\r", PCmd->Name);
    // Handle command
    if(PCmd->NameIs("Ping")) {
        PShell->Ack(retvOk);
    }
    else if(PCmd->NameIs("Next")) {
        SndList.PlayRandomFileFromDir(PlayDir);
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

    else if(PCmd->NameIs("SetLim_min")) {
        if(PCmd->GetNextInt32(&Data) == retvOk) {
            Uart.Printf("Data=%d\r ", Data);
            LEDs.SetLimit_MIN(Data);
        }
        PShell->Ack(retvOk);
    }
    else if(PCmd->NameIs("SetLim_max")) {
        if(PCmd->GetNextInt32(&Data) == retvOk) {
            Uart.Printf("Data=%d\r ", Data);
            LEDs.SetLimit_MAX(Data);
        }
        PShell->Ack(retvOk);
    }

    else PShell->Ack(retvCmdUnknown);
}
#endif
