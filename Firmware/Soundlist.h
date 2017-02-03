/*
 * Soundlist.h
 *
 *  Created on: 18 џэт. 2015 у.
 *      Author: Kreyl
 */

#ifndef SRC_SOUNDLIST_H_
#define SRC_SOUNDLIST_H_

#include "kl_lib_f2xx.h"
#include "kl_sd.h"


class SndList_t {
private:
    char Filename[MAX_NAME_LEN];    // to store name with path
    uint32_t PreviousN = UINT32_MAX;
    DIR Dir;
    FILINFO FileInfo;
    FRESULT CountFilesInDir(const char* DirName, uint32_t *PCnt);
public:
    void PlayRandomFileFromDir(const char* DirName);
    uint32_t GetTrackNumber() {return PreviousN;}
    void SetPreviousTrack(uint32_t N) {PreviousN = N;}
};



#endif /* SRC_SOUNDLIST_H_ */
