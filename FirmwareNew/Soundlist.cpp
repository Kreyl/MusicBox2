/*
 * Soundlist.cpp
 *
 *  Created on: 18 ���. 2015 �.
 *      Author: Kreyl
 */

#include "Soundlist.h"
#include "uart.h"
#include "sound.h"

FRESULT SndList_t::CountFilesInDir(const char* DirName, uint32_t *PCnt) {
    FRESULT Rslt = f_opendir(&Dir, DirName);
    Uart.Printf("R=%u\r", Rslt);
    if(Rslt != FR_OK) return Rslt;
    *PCnt = 0;
    while(true) {
        Rslt = f_readdir(&Dir, &FileInfo);
        if(Rslt != FR_OK) return Rslt;
        if((FileInfo.fname[0] == 0) and (FileInfo.lfname[0] == 0)) return FR_OK;   // No files left
        else { // Filename ok, check if not dir
            if(!(FileInfo.fattrib & AM_DIR)) {
                // Check if wav or mp3
                char *FName = (FileInfo.lfname[0] == 0)? FileInfo.fname : FileInfo.lfname;
//                Uart.Printf("\r%S", FName);
                uint32_t Len = strlen(FName);
                if(Len > 4) {
                    if((strcasecmp(&FName[Len-3], "mp3") == 0) or (strcasecmp(&FName[Len-3], "wav") == 0)) (*PCnt)++;
                } // if Len>4
            } // if not dir
        } // Filename ok
    }
    return FR_OK;
}

void SndList_t::PlayRandomFileFromDir(const char* DirName) {
    uint32_t Cnt=0;
    FRESULT Rslt = CountFilesInDir(DirName, &Cnt);
    Uart.Printf("R=%u; Cnt=%u\r", Rslt, Cnt);
    if(Rslt != FR_OK or Cnt == 0) return;       // Get out if nothing to play
    // Select number of file
    uint32_t Number = 0;
    if (Cnt == 2 and PreviousN == 0) Number = 1;
    else if (Cnt > 2) {   // Get random number if count > 2
        do {
            Number = Random(Cnt-1);      // [0; Cnt-1]
        } while(Number == PreviousN);    // skip same as previous
    }
//    Uart.Printf("; TrackNumber=%u", N);
    PreviousN = Number;
    // Iterate files in dir until success
    uint32_t Counter = 0;
    Rslt = f_opendir(&Dir, DirName);
    if(Rslt != FR_OK) return;
    while(true) {
        Rslt = f_readdir(&Dir, &FileInfo);
        if(Rslt != FR_OK) return;
        if((FileInfo.fname[0] == 0) and (FileInfo.lfname[0] == 0)) return;  // somehow no files left
        else { // Filename ok, check if not dir
            if(!(FileInfo.fattrib & AM_DIR)) {
                // Check if wav or mp3
                char *FName = (FileInfo.lfname[0] == 0)? FileInfo.fname : FileInfo.lfname;
//                Uart.Printf("\r%S  Cnt=%u", FName, Counter);
                uint32_t Len = strlen(FName);
                if(Len > 4) {
                    if((strcasecmp(&FName[Len-3], "mp3") == 0) or (strcasecmp(&FName[Len-3], "wav") == 0)) {
                        if(Number == Counter) {
                            // Build full filename with path
                            // Check if root dir. Empty string allowed, too
                            int Len = strlen(DirName);
                            if((Len > 1) or (Len == 1 and *DirName != '/' and *DirName != '\\')) {
                                strcpy(Filename, DirName);
                                Filename[Len] = '/';
                            }
                            strcpy(&Filename[Len+1], FName);
                            Sound.Play(Filename);
                            return;
                        }
                        else Counter++;
                    }
                } // if Len>4
            } // if not dir
        } // Filename o
    } // while true
}





