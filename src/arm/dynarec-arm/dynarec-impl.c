/* Copyright (c) 2013-2016 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include <assert.h>

#include "arm/decoder.h"
#include "arm/dynarec.h"
#include "arm/isa-thumb.h"
#include "arm/dynarec-arm/emitter.h"
#include "arm/macros.h"

static void lazySetPrefetch(struct ARMDynarecContext* ctx, uint32_t op0, uint32_t op1) {
	ctx->prefetch_flushed = false;
	ctx->prefetch[0] = op0;
	ctx->prefetch[1] = op1;
}

static void flushPrefetch(struct ARMDynarecContext* ctx) {
	if (!ctx->prefetch_flushed) {
		EMIT_IMM(ctx, AL, REG_SCRATCH0, ctx->prefetch[0]);
		EMIT(ctx, STRI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, prefetch) + 0 * sizeof(uint32_t));
		EMIT_IMM(ctx, AL, REG_SCRATCH0, ctx->prefetch[1]);
		EMIT(ctx, STRI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, prefetch) + 1 * sizeof(uint32_t));
		ctx->prefetch_flushed = true;
	}
}

static void flushPC(struct ARMDynarecContext* ctx) {
	if (!ctx->gpr_15_flushed) {
		EMIT_IMM(ctx, AL, REG_SCRATCH0, ctx->gpr_15);
		EMIT(ctx, STRI, AL, REG_SCRATCH0, REG_ARMCore, 15 * sizeof(uint32_t));
		ctx->gpr_15_flushed = true;
	}
}

static void interpretInstruction(struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPrefetch(ctx);
	flushPC(ctx);
	ThumbInstruction instruction = _thumbTable[opcode >> 6];
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, BL, AL, ctx->code, instruction);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT(ctx, POP, AL, REGLIST_RETURN);
}

static void InterpretThumbInstructionNormally(struct ARMCore* cpu) {
	uint32_t opcode = cpu->prefetch[0];
	cpu->prefetch[0] = cpu->prefetch[1];
	cpu->gprs[ARM_PC] += WORD_SIZE_THUMB;
	LOAD_16(cpu->prefetch[1], cpu->gprs[ARM_PC] & cpu->memory.activeMask, cpu->memory.activeRegion);
	ThumbInstruction instruction = _thumbTable[opcode >> 6];
	instruction(cpu, opcode);
}

void ARMDynarecEmitPrelude(struct ARMCore* cpu) {
	code_t* code = (code_t*) cpu->dynarec.buffer;
	cpu->dynarec.execute = (void (*)(struct ARMCore*, void*)) code;

	// Common prologue
	EMIT_L(code, PUSH, AL, 0x4DF0);
	EMIT_L(code, PUSH, AL, REGLIST_RETURN);
	EMIT_L(code, MOV, AL, 15, 1);

	// Common epilogue
	EMIT_L(code, POP, AL, 0x8DF0);

	cpu->dynarec.buffer = code;
	__clear_cache(cpu->dynarec.execute, code);
}

void ARMDynarecExecuteTrace(struct ARMCore* cpu, struct ARMDynarecTrace* trace) {
	if (!trace->entryPlus4) return;
	assert((uint32_t)cpu->gprs[15] == trace->start + WORD_SIZE_THUMB);

	// First, we're going to empty prefetch.
	InterpretThumbInstructionNormally(cpu);
	if (cpu->cycles >= cpu->nextEvent || (uint32_t)cpu->gprs[15] != trace->start + 2 * WORD_SIZE_THUMB)
		return;
	InterpretThumbInstructionNormally(cpu);
	if (cpu->cycles >= cpu->nextEvent || (uint32_t)cpu->gprs[15] != trace->start + 3 * WORD_SIZE_THUMB)
		return;

	// We've emptied the prefetcher. The first instruction to execute is trace->start + 2 * WORD_SIZE_THUMB
	cpu->dynarec.execute(cpu, trace->entryPlus4);
}

void ARMDynarecRecompileTrace(struct ARMCore* cpu, struct ARMDynarecTrace* trace) {
#ifndef NDEBUG
	printf("%08X (%c)\n", trace->start, trace->mode == MODE_THUMB ? 'T' : 'A');
	printf("%u\n", cpu->nextEvent - cpu->cycles);
#endif

	struct ARMDynarecContext ctx = {
		.code = cpu->dynarec.buffer,
		.gpr_15 = trace->start + 1 * WORD_SIZE_THUMB,
		.cycles = 0,
		.gpr_15_flushed = true,
		.prefetch_flushed = true,
		.scratch0_in_use = false,
		.scratch1_in_use = false,
		.scratch2_in_use = false,
	};

	if (trace->mode == MODE_ARM) {
		return;
	} else {
		trace->entry = 1;
		trace->entryPlus4 = (void*) ctx.code;
		ctx.gpr_15 = trace->start + 3 * WORD_SIZE_THUMB;
		while (true) {
			ctx.gpr_15_flushed = false;
			ctx.gpr_15 += WORD_SIZE_THUMB;

			uint16_t opcode, op0, op1;
			LOAD_16(opcode, (ctx.gpr_15 - 2 * WORD_SIZE_THUMB) & cpu->memory.activeMask, cpu->memory.activeRegion);
			LOAD_16(op0,    (ctx.gpr_15 - 1 * WORD_SIZE_THUMB) & cpu->memory.activeMask, cpu->memory.activeRegion);
			LOAD_16(op1,    (ctx.gpr_15 - 0 * WORD_SIZE_THUMB) & cpu->memory.activeMask, cpu->memory.activeRegion);
			lazySetPrefetch(&ctx, op0, op1);

			interpretInstruction(&ctx, opcode);
			break;
		}
	}
	//__clear_cache(trace->entry, ctx.code);
	__clear_cache(trace->entryPlus4, ctx.code);
	cpu->dynarec.buffer = ctx.code;
}
