#define MAX_FLASH_FILE_SIZE 10810
#include "./rcfs-master/FlashLib.h"

const float snapshotFreq = 30; // Hz
const float deltaT = (1/snapshotFreq) * 1000; // time between snapshots in milliseconds

struct replayData {
	unsigned char streamData[10802];
	unsigned int streamIndex;
	unsigned int streamSize;
	bool loaded;
};

void initReplayData(replayData* data) {
	data->streamIndex = 0;
	data->streamSize = 0;
	data->loaded = false;
}

unsigned char readNextByte(replayData* data) {
	unsigned char ret = data->streamData[data->streamIndex];
	data->streamIndex += 1;
	return ret;
}

void writeByte(replayData* data, unsigned char dat) {
	data->streamData[data->streamIndex] = dat;
	data->streamIndex += 1;
}

void findFile(char* name, flash_file* out) {
	flash_file* cur = out;

	if(RCFS_FindFirstFile(cur) < 0)
		return;

	do {
		if( strcmp(name, &(cur->name[0])) == 0 ) {
			return;
		}
	} while(RCFS_FindNextFile(cur) >= 0);

	RCFS_FileInit(cur);
}

/*
 * Stream on-flash file format (current):
 *  2 bytes: stream size in bytes
 *  n bytes: stream data
 */

void saveReplayToFile(char* name, replayData* repSt) {
	// discarded parameters to RCFS_GetFile
	unsigned char* tmp1;
	int tmp2;

#ifdef DEBUG
	writeDebugStreamLine("Killed motors, now finding file:");
	writeDebugStreamLine(name);
#endif

	repSt->streamSize = repSt->streamIndex+1;

	flash_file fHandle;
	findFile(name, &fHandle);
	if(fHandle.addr == NULL) {
#ifdef DEBUG
		writeDebugStreamLine("File not found, adding to FS.");
#endif

		repSt->streamData[0] = (repSt->streamSize & 0xFF);
		repSt->streamData[1] = ((repSt->streamSize & 0xFF00) >> 8) & 0xFF;

		clearLCDLine(0);
		clearLCDLine(1);
		displayLCDCenteredString(0, "! WRITING !");
#ifdef DEBUG
		writeDebugStreamLine("BEGINNING WRITE!");
#endif
		signed int err = 0;
		if((err = RCFS_AddFile((unsigned char*)repSt->streamData, 10802, name)) < 0) {
			clearLCDLine(0);
			displayLCDCenteredString(0, "Write failed!");
#ifdef DEBUG
			writeDebugStreamLine("Write failed, code: %d", err);
#endif
		}
	} else {
#ifdef DEBUG
		writeDebugStreamLine("Found existing file.");
#endif
		fHandle.data[0] = (repSt->streamSize & 0xFF);
		fHandle.data[1] = ((repSt->streamSize & 0xFF00) >> 8) & 0xFF;

		for(unsigned int i=2;i<repSt->streamSize;i++) {
			fHandle.data[i] = repSt->streamData[i];
		}

		clearLCDLine(0);
		clearLCDLine(1);
		displayLCDCenteredString(0, "! WRITING !");
#ifdef DEBUG
		writeDebugStreamLine("BEGINNING WRITE!");
#endif
		RCFS_Write(&fHandle);

		clearLCDLine(0);
		clearLCDLine(1);
		displayLCDCenteredString(0, "Write complete.");
	}
#ifdef DEBUG
	writeDebugStreamLine("Write complete.");
#endif
}

void loadReplayFromFile(const char* name, replayData* repSt) {
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
	repSt->loaded = true;
	} else {
		clearLCDLine(0);
		displayLCDCenteredString(0, "File not found!");
#ifdef DEBUG
		writeDebugStreamLine("Could not find file.");
#endif
	}
}
