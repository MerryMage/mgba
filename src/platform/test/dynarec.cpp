/* Copyright (c) 2013-2015 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "core/config.h"
#include "core/core.h"
#include "gba/core.h"
#include "gba/gba.h"
#include "gba/serialize.h"

#include "feature/commandline.h"
#include "util/memory.h"
#include "util/string.h"
#include "util/vfs.h"

#include <errno.h>
#include <signal.h>

int main(int argc, char** argv) {
	struct mCore* core = GBACoreCreate();
	core->init(core);
	mCoreInitConfig(core, "fuzz");
	mCoreConfigSetDefaultValue(&core->config, "idleOptimization", "remove");

	applyArguments(&args, NULL, &core->config);

	void* outputBuffer;
	outputBuffer = 0;


	((struct GBA*) core->board)->hardCrash = false;
	mCoreLoadFile(core, args.fname);

	struct VFile* savestate = 0;
	struct VFile* savestateOverlay = 0;
	size_t overlayOffset;

	if (fuzzOpts.savestate) {
		savestate = VFileOpen(fuzzOpts.savestate, O_RDONLY);
		free(fuzzOpts.savestate);
	}
	if (fuzzOpts.ssOverlay) {
		overlayOffset = fuzzOpts.overlayOffset;
		if (overlayOffset < sizeof(struct GBASerializedState)) {
			savestateOverlay = VFileOpen(fuzzOpts.ssOverlay, O_RDONLY);
		}
		free(fuzzOpts.ssOverlay);
	}
	if (savestate) {
		if (!savestateOverlay) {
			mCoreLoadStateNamed(core, savestate, 0);
		} else {
			struct GBASerializedState* state = GBAAllocateState();
			savestate->read(savestate, state, sizeof(*state));
			savestateOverlay->read(savestateOverlay, (uint8_t*) state + overlayOffset, sizeof(*state) - overlayOffset);
			GBADeserialize(core->board, state);
			GBADeallocateState(state);
			savestateOverlay->close(savestateOverlay);
			savestateOverlay = 0;
		}
		savestate->close(savestate);
		savestate = 0;
	}

	blip_set_rates(core->getAudioChannel(core, 0), GBA_ARM7TDMI_FREQUENCY, 0x8000);
	blip_set_rates(core->getAudioChannel(core, 1), GBA_ARM7TDMI_FREQUENCY, 0x8000);

	core->reset(core);

	_GBAFuzzRunloop(core, fuzzOpts.frames);

	core->unloadROM(core);

	if (savestate) {
		savestate->close(savestate);
	}
	if (savestateOverlay) {
		savestateOverlay->close(savestateOverlay);
	}

	freeArguments(&args);
	if (outputBuffer) {
		free(outputBuffer);
	}
	core->deinit(core);

	return 0;
}

static void _GBAFuzzRunloop(struct mCore* core, int frames) {
	do {
		core->runFrame(core);
		blip_clear(core->getAudioChannel(core, 0));
		blip_clear(core->getAudioChannel(core, 1));
	} while (core->frameCounter(core) < frames && !_dispatchExiting);
}

static void _GBAFuzzShutdown(int signal) {
	UNUSED(signal);
	_dispatchExiting = true;
}

static bool _parseFuzzOpts(struct mSubParser* parser, int option, const char* arg) {
	struct FuzzOpts* opts = parser->opts;
	errno = 0;
	switch (option) {
		case 'F':
			opts->frames = strtoul(arg, 0, 10);
			return !errno;
		case 'N':
			opts->noVideo = true;
			return true;
		case 'O':
			opts->overlayOffset = strtoul(arg, 0, 10);
			return !errno;
		case 'S':
			opts->savestate = strdup(arg);
			return true;
		case 'V':
			opts->ssOverlay = strdup(arg);
			return true;
		default:
			return false;
	}
}
