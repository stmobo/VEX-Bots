#ifndef ENTERPRISE_C
#define ENTERPRISE_C

#define MAX_FLASH_FILE_SIZE 10810
#include "./rcfs/FlashLib.h"

const float snapshotFreq = 30; // Hz
const float deltaT = (1/snapshotFreq) * 1000; // time between snapshots in milliseconds

#define TEST_BIT(x, i) (((x)&(1<<(i))) > 0)

struct replay_t {
	unsigned char streamData[10802];
	unsigned int streamIndex;
	unsigned int streamSize;
	bool loaded;
};

void initReplayData(replay_t* data) {
	data->streamIndex = 2;
	data->streamSize = 0;
	data->loaded = false;
}

unsigned char readNextByte(replay_t* data) {
    if(data->streamIndex >= data->streamSize)
        return 0;

	unsigned char ret = data->streamData[data->streamIndex];
	data->streamIndex += 1;
	return ret;
}

void writeByte(replay_t* data, unsigned char dat) {
    if(data->streamIndex > 10802)
        return;

	data->streamData[data->streamIndex] = dat;
	data->streamIndex += 1;
}

void findFile(char* name, flash_file* out) {
	flash_file cur;

    RCFS_FileInit(&cur);
    RCFS_FileInit(out);

	if(RCFS_FindFirstFile(&cur) < 0)
		return;

#ifdef DEBUG
    int i = 0;
#endif

	do {
		if( strcmp(name, (char*)cur.name) == 0 ) {
			*out = cur;
#ifdef DEBUG
            i++;
            writeDebugStreamLine("Found iteration %d of file %s.", i, name);
#endif
		}
	} while(RCFS_FindNextFile(&cur) >= 0);
}

/*
 * Stream on-flash file format (current):
 *  2 bytes: stream size in bytes
 *  n bytes: stream data
 */

void saveReplayToFile(char* name, replay_t* repSt) {
#ifdef DEBUG
	writeDebugStreamLine("Killed motors, now finding file:");
	writeDebugStreamLine(name);
#endif

	repSt->streamSize = repSt->streamIndex;
    repSt->streamData[0] = (repSt->streamSize & 0xFF);
    repSt->streamData[1] = ((repSt->streamSize & 0xFF00) >> 8) & 0xFF;

    clearLCDLine(0);
    clearLCDLine(1);
    displayLCDCenteredString(0, "! WRITING !");

#ifdef DEBUG
    writeDebugStreamLine("BEGINNING WRITE!");
#endif

    signed int err = 0;
    if((err = RCFS_AddFile((unsigned char*)repSt->streamData, repSt->streamSize, name)) < 0) {
        clearLCDLine(0);
        displayLCDCenteredString(0, "Write failed!");
#ifdef DEBUG
        writeDebugStreamLine("Write failed, code: %d", err);
#endif
    }

#ifdef DEBUG
   writeDebugStreamLine("Write complete.");
#endif
}

void loadReplayFromFile(const char* name, replay_t* repSt) {
	clearLCDLine(0);
	clearLCDLine(1);
#ifdef DEBUG
	writeDebugStreamLine("Attempting to load stream:");
	writeDebugStreamLine(name);
#endif
	displayLCDCenteredString(0, "Finding file...");

	flash_file fHandle;
	findFile(name, &fHandle);
	if(fHandle.addr != NULL) {
#ifdef DEBUG
		writeDebugStreamLine("Found file!");
#endif

		displayLCDCenteredString(0, "Loading replay...");

		unsigned int streamSz = (fHandle.data[0] | (((unsigned int)(fHandle.data[1])) << 8));
		for(unsigned int i=0;i<streamSz;i++) {
			repSt->streamData[i] = fHandle.data[i];
		}
		repSt->streamSize = streamSz;
#ifdef DEBUG
		writeDebugStreamLine("Loaded %i bytes.", streamSz);
#endif

        repSt->streamIndex = 2; // stream filesize = 2 bytes at positions 0-1
        repSt->loaded = true;
	} else {
		clearLCDLine(0);
		displayLCDCenteredString(0, "File not found!");
#ifdef DEBUG
		writeDebugStreamLine("Could not find file.");
#endif
	}
}

#endif /* end of include guard: ENTERPRISE_C */
