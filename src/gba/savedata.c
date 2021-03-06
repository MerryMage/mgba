/* Copyright (c) 2013-2015 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "savedata.h"

#include "gba/gba.h"
#include "gba/serialize.h"

#include "util/memory.h"
#include "util/vfs.h"

#include <errno.h>
#include <fcntl.h>

// Some testing was done here...
// Erase cycles can vary greatly.
// Some games may vary anywhere between about 2000 cycles to up to 30000 cycles. (Observed on a Macronix (09C2) chip).
// Other games vary from very little, with a fairly solid 20500 cycle count. (Observed on a SST (D4BF) chip).
// An average estimation is as follows.
#define FLASH_ERASE_CYCLES 30000
#define FLASH_PROGRAM_CYCLES 18000
// This needs real testing, and is only an estimation currently
#define EEPROM_SETTLE_CYCLES 1450
#define CLEANUP_THRESHOLD 15

mLOG_DEFINE_CATEGORY(GBA_SAVE, "GBA Savedata");

static void _flashSwitchBank(struct GBASavedata* savedata, int bank);
static void _flashErase(struct GBASavedata* savedata);
static void _flashEraseSector(struct GBASavedata* savedata, uint16_t sectorStart);

void GBASavedataInit(struct GBASavedata* savedata, struct VFile* vf) {
	savedata->type = SAVEDATA_AUTODETECT;
	savedata->data = 0;
	savedata->command = EEPROM_COMMAND_NULL;
	savedata->flashState = FLASH_STATE_RAW;
	savedata->vf = vf;
	savedata->realVf = vf;
	savedata->mapMode = MAP_WRITE;
	savedata->dirty = 0;
	savedata->dirtAge = 0;
}

void GBASavedataDeinit(struct GBASavedata* savedata) {
	if (savedata->vf) {
		size_t size = GBASavedataSize(savedata);
		savedata->vf->unmap(savedata->vf, savedata->data, size);
		if (savedata->vf != savedata->realVf) {
			savedata->vf->close(savedata->vf);
		}
		savedata->vf = 0;
	} else {
		switch (savedata->type) {
		case SAVEDATA_SRAM:
			mappedMemoryFree(savedata->data, SIZE_CART_SRAM);
			break;
		case SAVEDATA_FLASH512:
			mappedMemoryFree(savedata->data, SIZE_CART_FLASH512);
			break;
		case SAVEDATA_FLASH1M:
			mappedMemoryFree(savedata->data, SIZE_CART_FLASH1M);
			break;
		case SAVEDATA_EEPROM:
			mappedMemoryFree(savedata->data, SIZE_CART_EEPROM);
			break;
		case SAVEDATA_FORCE_NONE:
		case SAVEDATA_AUTODETECT:
			break;
		}
	}
	savedata->data = 0;
	savedata->type = SAVEDATA_AUTODETECT;
}

void GBASavedataMask(struct GBASavedata* savedata, struct VFile* vf) {
	enum SavedataType type = savedata->type;
	GBASavedataDeinit(savedata);
	savedata->vf = vf;
	savedata->mapMode = MAP_READ;
	GBASavedataForceType(savedata, type, savedata->realisticTiming);
}

void GBASavedataUnmask(struct GBASavedata* savedata) {
	if (savedata->mapMode != MAP_READ) {
		return;
	}
	enum SavedataType type = savedata->type;
	GBASavedataDeinit(savedata);
	savedata->vf = savedata->realVf;
	savedata->mapMode = MAP_WRITE;
	GBASavedataForceType(savedata, type, savedata->realisticTiming);
}

bool GBASavedataClone(struct GBASavedata* savedata, struct VFile* out) {
	if (savedata->data) {
		switch (savedata->type) {
		case SAVEDATA_SRAM:
			return out->write(out, savedata->data, SIZE_CART_SRAM) == SIZE_CART_SRAM;
		case SAVEDATA_FLASH512:
			return out->write(out, savedata->data, SIZE_CART_FLASH512) == SIZE_CART_FLASH512;
		case SAVEDATA_FLASH1M:
			return out->write(out, savedata->data, SIZE_CART_FLASH1M) == SIZE_CART_FLASH1M;
		case SAVEDATA_EEPROM:
			return out->write(out, savedata->data, SIZE_CART_EEPROM) == SIZE_CART_EEPROM;
		case SAVEDATA_AUTODETECT:
		case SAVEDATA_FORCE_NONE:
			return true;
		}
	} else if (savedata->vf) {
		off_t read = 0;
		uint8_t buffer[2048];
		do {
			read = savedata->vf->read(savedata->vf, buffer, sizeof(buffer));
			out->write(out, buffer, read);
		} while (read == sizeof(buffer));
		return read >= 0;
	}
	return true;
}

size_t GBASavedataSize(struct GBASavedata* savedata) {
	switch (savedata->type) {
	case SAVEDATA_SRAM:
		return SIZE_CART_SRAM;
	case SAVEDATA_FLASH512:
		return SIZE_CART_FLASH512;
	case SAVEDATA_FLASH1M:
		return SIZE_CART_FLASH1M;
	case SAVEDATA_EEPROM:
		return SIZE_CART_EEPROM;
	case SAVEDATA_FORCE_NONE:
		return 0;
	case SAVEDATA_AUTODETECT:
	default:
		if (savedata->vf) {
			return savedata->vf->size(savedata->vf);
		}
		return 0;
	}
}

bool GBASavedataLoad(struct GBASavedata* savedata, struct VFile* in) {
	if (savedata->vf) {
		off_t read = 0;
		uint8_t buffer[2048];
		memset(buffer, 0xFF, sizeof(buffer));
		savedata->vf->seek(savedata->vf, 0, SEEK_SET);
		while (savedata->vf->seek(savedata->vf, 0, SEEK_CUR) < savedata->vf->size(savedata->vf)) {
			savedata->vf->write(savedata->vf, buffer, sizeof(buffer));
		}
		savedata->vf->seek(savedata->vf, 0, SEEK_SET);
		if (in) {
			do {
				read = in->read(in, buffer, sizeof(buffer));
				read = savedata->vf->write(savedata->vf, buffer, read);
			} while (read == sizeof(buffer));
		}
		return read >= 0;
	} else if (savedata->data) {
		if (!in && savedata->type != SAVEDATA_FORCE_NONE) {
			return false;
		}
		switch (savedata->type) {
		case SAVEDATA_SRAM:
			return in->read(in, savedata->data, SIZE_CART_SRAM) == SIZE_CART_SRAM;
		case SAVEDATA_FLASH512:
			return in->read(in, savedata->data, SIZE_CART_FLASH512) == SIZE_CART_FLASH512;
		case SAVEDATA_FLASH1M:
			return in->read(in, savedata->data, SIZE_CART_FLASH1M) == SIZE_CART_FLASH1M;
		case SAVEDATA_EEPROM:
			return in->read(in, savedata->data, SIZE_CART_EEPROM) == SIZE_CART_EEPROM;
		case SAVEDATA_AUTODETECT:
		case SAVEDATA_FORCE_NONE:
			return true;
		}
	}
	return true;
}

void GBASavedataForceType(struct GBASavedata* savedata, enum SavedataType type, bool realisticTiming) {
	if (savedata->type != SAVEDATA_AUTODETECT) {
		struct VFile* vf = savedata->vf;
		GBASavedataDeinit(savedata);
		GBASavedataInit(savedata, vf);
	}
	switch (type) {
	case SAVEDATA_FLASH512:
	case SAVEDATA_FLASH1M:
		savedata->type = type;
		GBASavedataInitFlash(savedata, realisticTiming);
		break;
	case SAVEDATA_EEPROM:
		GBASavedataInitEEPROM(savedata, realisticTiming);
		break;
	case SAVEDATA_SRAM:
		GBASavedataInitSRAM(savedata);
		break;
	case SAVEDATA_FORCE_NONE:
		savedata->type = SAVEDATA_FORCE_NONE;
		break;
	case SAVEDATA_AUTODETECT:
		break;
	}
}

void GBASavedataInitFlash(struct GBASavedata* savedata, bool realisticTiming) {
	if (savedata->type == SAVEDATA_AUTODETECT) {
		savedata->type = SAVEDATA_FLASH512;
	}
	if (savedata->type != SAVEDATA_FLASH512 && savedata->type != SAVEDATA_FLASH1M) {
		mLOG(GBA_SAVE, WARN, "Can't re-initialize savedata");
		return;
	}
	int32_t flashSize = SIZE_CART_FLASH512;
	if (savedata->type == SAVEDATA_FLASH1M) {
		flashSize = SIZE_CART_FLASH1M;
	}
	off_t end;
	if (!savedata->vf) {
		end = 0;
		savedata->data = anonymousMemoryMap(SIZE_CART_FLASH1M);
	} else {
		end = savedata->vf->size(savedata->vf);
		if (end < flashSize) {
			savedata->vf->truncate(savedata->vf, SIZE_CART_FLASH1M);
			flashSize = SIZE_CART_FLASH1M;
		}
		savedata->data = savedata->vf->map(savedata->vf, SIZE_CART_FLASH1M, savedata->mapMode);
	}

	savedata->currentBank = savedata->data;
	savedata->dust = 0;
	savedata->realisticTiming = realisticTiming;
	if (end < SIZE_CART_FLASH512) {
		memset(&savedata->data[end], 0xFF, flashSize - end);
	}
}

void GBASavedataInitEEPROM(struct GBASavedata* savedata, bool realisticTiming) {
	if (savedata->type == SAVEDATA_AUTODETECT) {
		savedata->type = SAVEDATA_EEPROM;
	} else {
		mLOG(GBA_SAVE, WARN, "Can't re-initialize savedata");
		return;
	}
	off_t end;
	if (!savedata->vf) {
		end = 0;
		savedata->data = anonymousMemoryMap(SIZE_CART_EEPROM);
	} else {
		end = savedata->vf->size(savedata->vf);
		if (end < SIZE_CART_EEPROM) {
			savedata->vf->truncate(savedata->vf, SIZE_CART_EEPROM);
		}
		savedata->data = savedata->vf->map(savedata->vf, SIZE_CART_EEPROM, savedata->mapMode);
	}
	savedata->dust = 0;
	savedata->realisticTiming = realisticTiming;
	if (end < SIZE_CART_EEPROM) {
		memset(&savedata->data[end], 0xFF, SIZE_CART_EEPROM - end);
	}
}

void GBASavedataInitSRAM(struct GBASavedata* savedata) {
	if (savedata->type == SAVEDATA_AUTODETECT) {
		savedata->type = SAVEDATA_SRAM;
	} else {
		mLOG(GBA_SAVE, WARN, "Can't re-initialize savedata");
		return;
	}
	off_t end;
	if (!savedata->vf) {
		end = 0;
		savedata->data = anonymousMemoryMap(SIZE_CART_SRAM);
	} else {
		end = savedata->vf->size(savedata->vf);
		if (end < SIZE_CART_SRAM) {
			savedata->vf->truncate(savedata->vf, SIZE_CART_SRAM);
		}
		savedata->data = savedata->vf->map(savedata->vf, SIZE_CART_SRAM, savedata->mapMode);
	}

	if (end < SIZE_CART_SRAM) {
		memset(&savedata->data[end], 0xFF, SIZE_CART_SRAM - end);
	}
}

uint8_t GBASavedataReadFlash(struct GBASavedata* savedata, uint16_t address) {
	if (savedata->command == FLASH_COMMAND_ID) {
		if (savedata->type == SAVEDATA_FLASH512) {
			if (address < 2) {
				return FLASH_MFG_PANASONIC >> (address * 8);
			}
		} else if (savedata->type == SAVEDATA_FLASH1M) {
			if (address < 2) {
				return FLASH_MFG_SANYO >> (address * 8);
			}
		}
	}
	if (savedata->dust > 0 && (address >> 12) == savedata->settling) {
		// Give some overhead for waitstates and the comparison
		// This estimation can probably be improved
		savedata->dust -= 5000;
		return 0x5F;
	}
	return savedata->currentBank[address];
}

void GBASavedataWriteFlash(struct GBASavedata* savedata, uint16_t address, uint8_t value) {
	switch (savedata->flashState) {
	case FLASH_STATE_RAW:
		switch (savedata->command) {
		case FLASH_COMMAND_PROGRAM:
			savedata->dirty |= SAVEDATA_DIRT_NEW;
			savedata->currentBank[address] = value;
			savedata->command = FLASH_COMMAND_NONE;
			if (savedata->realisticTiming) {
				savedata->dust = FLASH_PROGRAM_CYCLES;
			}
			break;
		case FLASH_COMMAND_SWITCH_BANK:
			if (address == 0 && value < 2) {
				_flashSwitchBank(savedata, value);
			} else {
				mLOG(GBA_SAVE, GAME_ERROR, "Bad flash bank switch");
				savedata->command = FLASH_COMMAND_NONE;
			}
			savedata->command = FLASH_COMMAND_NONE;
			break;
		default:
			if (address == FLASH_BASE_HI && value == FLASH_COMMAND_START) {
				savedata->flashState = FLASH_STATE_START;
			} else {
				mLOG(GBA_SAVE, GAME_ERROR, "Bad flash write: %#04x = %#02x", address, value);
			}
			break;
		}
		break;
	case FLASH_STATE_START:
		if (address == FLASH_BASE_LO && value == FLASH_COMMAND_CONTINUE) {
			savedata->flashState = FLASH_STATE_CONTINUE;
		} else {
			mLOG(GBA_SAVE, GAME_ERROR, "Bad flash write: %#04x = %#02x", address, value);
			savedata->flashState = FLASH_STATE_RAW;
		}
		break;
	case FLASH_STATE_CONTINUE:
		savedata->flashState = FLASH_STATE_RAW;
		if (address == FLASH_BASE_HI) {
			switch (savedata->command) {
			case FLASH_COMMAND_NONE:
				switch (value) {
				case FLASH_COMMAND_ERASE:
				case FLASH_COMMAND_ID:
				case FLASH_COMMAND_PROGRAM:
				case FLASH_COMMAND_SWITCH_BANK:
					savedata->command = value;
					break;
				default:
					mLOG(GBA_SAVE, GAME_ERROR, "Unsupported flash operation: %#02x", value);
					break;
				}
				break;
			case FLASH_COMMAND_ERASE:
				switch (value) {
				case FLASH_COMMAND_ERASE_CHIP:
					_flashErase(savedata);
					break;
				default:
					mLOG(GBA_SAVE, GAME_ERROR, "Unsupported flash erase operation: %#02x", value);
					break;
				}
				savedata->command = FLASH_COMMAND_NONE;
				break;
			case FLASH_COMMAND_ID:
				if (value == FLASH_COMMAND_TERMINATE) {
					savedata->command = FLASH_COMMAND_NONE;
				}
				break;
			default:
				mLOG(GBA_SAVE, ERROR, "Flash entered bad state: %#02x", savedata->command);
				savedata->command = FLASH_COMMAND_NONE;
				break;
			}
		} else if (savedata->command == FLASH_COMMAND_ERASE) {
			if (value == FLASH_COMMAND_ERASE_SECTOR) {
				_flashEraseSector(savedata, address);
				savedata->command = FLASH_COMMAND_NONE;
			} else {
				mLOG(GBA_SAVE, GAME_ERROR, "Unsupported flash erase operation: %#02x", value);
			}
		}
		break;
	}
}

void GBASavedataWriteEEPROM(struct GBASavedata* savedata, uint16_t value, uint32_t writeSize) {
	switch (savedata->command) {
	// Read header
	case EEPROM_COMMAND_NULL:
	default:
		savedata->command = value & 0x1;
		break;
	case EEPROM_COMMAND_PENDING:
		savedata->command <<= 1;
		savedata->command |= value & 0x1;
		if (savedata->command == EEPROM_COMMAND_WRITE) {
			savedata->writeAddress = 0;
		} else {
			savedata->readAddress = 0;
		}
		break;
	// Do commands
	case EEPROM_COMMAND_WRITE:
		// Write
		if (writeSize > 65) {
			savedata->writeAddress <<= 1;
			savedata->writeAddress |= (value & 0x1) << 6;
		} else if (writeSize == 1) {
			savedata->command = EEPROM_COMMAND_NULL;
		} else if ((savedata->writeAddress >> 3) < SIZE_CART_EEPROM) {
			uint8_t current = savedata->data[savedata->writeAddress >> 3];
			current &= ~(1 << (0x7 - (savedata->writeAddress & 0x7)));
			current |= (value & 0x1) << (0x7 - (savedata->writeAddress & 0x7));
			savedata->dirty |= SAVEDATA_DIRT_NEW;
			savedata->data[savedata->writeAddress >> 3] = current;
			if (savedata->realisticTiming) {
				savedata->dust = EEPROM_SETTLE_CYCLES;
			}
			++savedata->writeAddress;
		} else {
			mLOG(GBA_SAVE, GAME_ERROR, "Writing beyond end of EEPROM: %08X", (savedata->writeAddress >> 3));
		}
		break;
	case EEPROM_COMMAND_READ_PENDING:
		// Read
		if (writeSize > 1) {
			savedata->readAddress <<= 1;
			if (value & 0x1) {
				savedata->readAddress |= 0x40;
			}
		} else {
			savedata->readBitsRemaining = 68;
			savedata->command = EEPROM_COMMAND_READ;
		}
		break;
	}
}

uint16_t GBASavedataReadEEPROM(struct GBASavedata* savedata) {
	if (savedata->command != EEPROM_COMMAND_READ) {
		if (!savedata->realisticTiming || savedata->dust <= 0) {
			return 1;
		} else {
			// Give some overhead for waitstates and the comparison
			// This estimation can probably be improved
			--savedata->dust;
			return 0;
		}
	}
	--savedata->readBitsRemaining;
	if (savedata->readBitsRemaining < 64) {
		int step = 63 - savedata->readBitsRemaining;
		uint32_t address = (savedata->readAddress + step) >> 3;
		if (address >= SIZE_CART_EEPROM) {
			mLOG(GBA_SAVE, GAME_ERROR, "Reading beyond end of EEPROM: %08X", address);
			return 0xFF;
		}
		uint8_t data = savedata->data[address] >> (0x7 - (step & 0x7));
		if (!savedata->readBitsRemaining) {
			savedata->command = EEPROM_COMMAND_NULL;
		}
		return data & 0x1;
	}
	return 0;
}

void GBASavedataClean(struct GBASavedata* savedata, uint32_t frameCount) {
	if (!savedata->vf) {
		return;
	}
	if (savedata->dirty & SAVEDATA_DIRT_NEW) {
		savedata->dirty &= ~SAVEDATA_DIRT_NEW;
		if (!(savedata->dirty & SAVEDATA_DIRT_SEEN)) {
			savedata->dirtAge = frameCount;
			savedata->dirty |= SAVEDATA_DIRT_SEEN;
		}
	} else if ((savedata->dirty & SAVEDATA_DIRT_SEEN) && frameCount - savedata->dirtAge > CLEANUP_THRESHOLD) {
		size_t size = GBASavedataSize(savedata);
		savedata->dirty = 0;
		if (savedata->data && savedata->vf->sync(savedata->vf, savedata->data, size)) {
			mLOG(GBA_SAVE, INFO, "Savedata synced");
		} else {
			mLOG(GBA_SAVE, INFO, "Savedata failed to sync!");
		}
	}
}

void GBASavedataSerialize(const struct GBASavedata* savedata, struct GBASerializedState* state) {
	state->savedata.type = savedata->type;
	state->savedata.command = savedata->command;
	GBASerializedSavedataFlags flags = 0;
	flags = GBASerializedSavedataFlagsSetFlashState(flags, savedata->flashState);
	flags = GBASerializedSavedataFlagsTestFillFlashBank(flags, savedata->currentBank == &savedata->data[0x10000]);
	state->savedata.flags = flags;
	STORE_32(savedata->readBitsRemaining, 0, &state->savedata.readBitsRemaining);
	STORE_32(savedata->readAddress, 0, &state->savedata.readAddress);
	STORE_32(savedata->writeAddress, 0, &state->savedata.writeAddress);
	STORE_16(savedata->settling, 0, &state->savedata.settlingSector);
	STORE_16(savedata->dust, 0, &state->savedata.settlingDust);
}

void GBASavedataDeserialize(struct GBASavedata* savedata, const struct GBASerializedState* state) {
	if (savedata->type != state->savedata.type) {
		mLOG(GBA_SAVE, DEBUG, "Switching save types");
		GBASavedataForceType(savedata, state->savedata.type, savedata->realisticTiming);
	}
	savedata->command = state->savedata.command;
	GBASerializedSavedataFlags flags = state->savedata.flags;
	savedata->flashState = GBASerializedSavedataFlagsGetFlashState(flags);
	LOAD_32(savedata->readBitsRemaining, 0, &state->savedata.readBitsRemaining);
	LOAD_32(savedata->readAddress, 0, &state->savedata.readAddress);
	LOAD_32(savedata->writeAddress, 0, &state->savedata.writeAddress);
	LOAD_16(savedata->settling, 0, &state->savedata.settlingSector);
	LOAD_16(savedata->dust, 0, &state->savedata.settlingDust);

	if (savedata->type == SAVEDATA_FLASH1M) {
		_flashSwitchBank(savedata, GBASerializedSavedataFlagsGetFlashBank(flags));
	}
}

void _flashSwitchBank(struct GBASavedata* savedata, int bank) {
	mLOG(GBA_SAVE, DEBUG, "Performing flash bank switch to bank %i", bank);
	savedata->currentBank = &savedata->data[bank << 16];
	if (bank > 0 && savedata->type == SAVEDATA_FLASH512) {
		savedata->type = SAVEDATA_FLASH1M;
		if (savedata->vf) {
			savedata->vf->truncate(savedata->vf, SIZE_CART_FLASH1M);
			memset(&savedata->data[SIZE_CART_FLASH512], 0xFF, SIZE_CART_FLASH512);
		}
	}
}

void _flashErase(struct GBASavedata* savedata) {
	mLOG(GBA_SAVE, DEBUG, "Performing flash chip erase");
	savedata->dirty |= SAVEDATA_DIRT_NEW;
	size_t size = SIZE_CART_FLASH512;
	if (savedata->type == SAVEDATA_FLASH1M) {
		size = SIZE_CART_FLASH1M;
	}
	memset(savedata->data, 0xFF, size);
}

void _flashEraseSector(struct GBASavedata* savedata, uint16_t sectorStart) {
	mLOG(GBA_SAVE, DEBUG, "Performing flash sector erase at 0x%04x", sectorStart);
	savedata->dirty |= SAVEDATA_DIRT_NEW;
	size_t size = 0x1000;
	if (savedata->type == SAVEDATA_FLASH1M) {
		mLOG(GBA_SAVE, DEBUG, "Performing unknown sector-size erase at 0x%04x", sectorStart);
	}
	savedata->settling = sectorStart >> 12;
	if (savedata->realisticTiming) {
		savedata->dust = FLASH_ERASE_CYCLES;
	}
	memset(&savedata->currentBank[sectorStart & ~(size - 1)], 0xFF, size);
}
