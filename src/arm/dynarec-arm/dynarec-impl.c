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
#include "arm/emitter-thumb.h"

typedef bool (*ThumbCompiler)(struct ARMCore*, struct ARMDynarecContext*, uint16_t opcode);
extern const ThumbCompiler _thumbCompilerTable[0x400];

void ARMDynarecEmitPrelude(struct ARMCore* cpu) {
	code_t* code = (code_t*) cpu->dynarec.buffer;

	// Common prologue
	cpu->dynarec.execute = (void (*)(struct ARMCore*, void*)) code;
	EMIT_L(code, PUSH, AL, 0x4DF0);
	EMIT_L(code, MOV, AL, 15, 1);

	// Common epilogue
	cpu->dynarec.epilogue = (void*) code;
	EMIT_L(code, POP, AL, 0x8DF0);

	cpu->dynarec.buffer = code;
	__clear_cache(cpu->dynarec.execute, code);
}

static void InterpretThumbInstructionNormally(struct ARMCore* cpu) {
    uint32_t opcode = cpu->prefetch[0];
    cpu->prefetch[0] = cpu->prefetch[1];
    cpu->gprs[ARM_PC] += WORD_SIZE_THUMB;
    LOAD_16(cpu->prefetch[1], cpu->gprs[ARM_PC] & cpu->memory.activeMask, cpu->memory.activeRegion);
    ThumbInstruction instruction = _thumbTable[opcode >> 6];
    instruction(cpu, opcode);
}

void ARMDynarecExecuteTrace(struct ARMCore* cpu, struct ARMDynarecTrace* trace) {
	if (!trace->entryPlus4) return;
	assert(cpu->executionMode == MODE_THUMB);
	assert((uint32_t)cpu->gprs[15] == trace->start + WORD_SIZE_THUMB);

	// First, we're going to empty cpu->prefetch of unknown values.
	if (cpu->cycles >= cpu->nextEvent) {
		cpu->irqh.processEvents(cpu);
		return;
	}
	// cpu->prefetch[0] == unknown
	// cpu->prefetch[1] == unknown
	InterpretThumbInstructionNormally(cpu);
	if (cpu->cycles >= cpu->nextEvent || (uint32_t)cpu->gprs[15] != trace->start + 2 * WORD_SIZE_THUMB) {
		if (cpu->cycles >= cpu->nextEvent) {
			cpu->irqh.processEvents(cpu);
		}
		return;
	}
	// cpu->prefetch[0] == unknown
	// cpu->prefetch[1] == instruction at trace->start + 2 * WORD_SIZE_THUMB
	InterpretThumbInstructionNormally(cpu);
	if (cpu->cycles >= cpu->nextEvent || (uint32_t)cpu->gprs[15] != trace->start + 3 * WORD_SIZE_THUMB) {
		if (cpu->cycles >= cpu->nextEvent) {
			cpu->irqh.processEvents(cpu);
		}
		return;
	}
	// cpu->prefetch[0] == instruction at trace->start + 2 * WORD_SIZE_THUMB
	// cpu->prefetch[1] == instruction at trace->start + 3 * WORD_SIZE_THUMB

	// We've emptied the prefetcher. The first instruction to execute is trace->start + 2 * WORD_SIZE_THUMB
	assert((uint32_t)cpu->gprs[15] == trace->start + 3 * WORD_SIZE_THUMB);
	cpu->dynarec.execute(cpu, trace->entryPlus4);

	if (cpu->cycles >= cpu->nextEvent) {
		cpu->irqh.processEvents(cpu);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void flushPC(struct ARMDynarecContext* ctx) {
	assert(!ctx->scratch_in_use[0]);
	if (!ctx->gpr_15_flushed) {
		EMIT_IMM(ctx, AL, REG_SCRATCH0, ctx->gpr_15);
		EMIT(ctx, STRI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
		ctx->gpr_15_flushed = true;
	}
}

static void flushPrefetch(struct ARMDynarecContext* ctx) {
	assert(!ctx->scratch_in_use[0]);
	if (!ctx->prefetch_flushed) {
		EMIT_IMM(ctx, AL, REG_SCRATCH0, ctx->prefetch[0]);
		EMIT(ctx, STRI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, prefetch) + 0 * sizeof(uint32_t));
		EMIT_IMM(ctx, AL, REG_SCRATCH0, ctx->prefetch[1]);
		EMIT(ctx, STRI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, prefetch) + 1 * sizeof(uint32_t));
		ctx->prefetch_flushed = true;
	}
}

void ARMDynarecRecompileTrace(struct ARMCore* cpu, struct ARMDynarecTrace* trace) {
#ifndef NDEBUG
//	printf("%08X (%c)\n", trace->start, trace->mode == MODE_THUMB ? 'T' : 'A');
//	printf("%u\n", cpu->nextEvent - cpu->cycles);
#endif

	if (trace->mode == MODE_ARM) {
		return;
	} else {
		struct ARMDynarecContext ctx = {
			.code = cpu->dynarec.buffer,
			.gpr_15 = trace->start + 3 * WORD_SIZE_THUMB,
			.cycles_register_valid = false,
			.cycles = 0,
			.gpr_15_flushed = false,
			.prefetch_flushed = false,
			.nzcv_location = CONTEXT_NZCV_IN_MEMORY,
		};
		LOAD_16(ctx.prefetch[0], (trace->start + 2 * WORD_SIZE_THUMB) & cpu->memory.activeMask, cpu->memory.activeRegion);
		LOAD_16(ctx.prefetch[1], (trace->start + 3 * WORD_SIZE_THUMB) & cpu->memory.activeMask, cpu->memory.activeRegion);

		trace->entry = 1;
		trace->entryPlus4 = (void*) ctx.code;

		bool continue_compilation = true;
		while (continue_compilation) {
			// emit: if (cpu->cycles >= cpu->nextEvent) return;
			EMIT(&ctx, LDRI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, cycles));
			EMIT(&ctx, LDRI, AL, REG_SCRATCH1, REG_ARMCore, offsetof(struct ARMCore, nextEvent));
			EMIT(&ctx, CMP, AL, REG_SCRATCH0, REG_SCRATCH1);
			EMIT(&ctx, B, GE, ctx.code, cpu->dynarec.epilogue);

			// ThumbStep
			uint32_t opcode = ctx.prefetch[0];
			ctx.prefetch[0] = ctx.prefetch[1];
			ctx.gpr_15 += WORD_SIZE_THUMB;
			LOAD_16(ctx.prefetch[1], ctx.gpr_15 & cpu->memory.activeMask, cpu->memory.activeRegion);
			ctx.prefetch_flushed = false;
			ctx.gpr_15_flushed = false;
			ThumbCompiler instruction = _thumbCompilerTable[opcode >> 6];
			continue_compilation = instruction(cpu, &ctx, opcode);
		}

		//__clear_cache(trace->entry, ctx.code);
		__clear_cache(trace->entryPlus4, ctx.code);
		cpu->dynarec.buffer = ctx.code;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loadNZCV(struct ARMDynarecContext* ctx) {
	assert(!ctx->scratch_in_use[0]);
	EMIT(ctx, LDRI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, cpsr));
	EMIT(ctx, MSR, AL, true, false, REG_SCRATCH0);
}

void flushNZCV(struct ARMDynarecContext* ctx) {
	assert(!ctx->scratch_in_use[0]);
	EMIT(ctx, MRS, AL, REG_SCRATCH0);
	EMIT(ctx, MOV_LSRI, AL, REG_SCRATCH0, REG_SCRATCH0, 24);
	EMIT(ctx, STRBI, AL, REG_SCRATCH0, REG_ARMCore, (int)offsetof(struct ARMCore, cpsr) + 3);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool _ThumbCompilerLSL1(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerLSR1(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerASR1(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerADD3(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerSUB3(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerADD1(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerSUB1(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerMOV1(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerCMP1(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerADD2(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerSUB2(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerAND(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerEOR(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerLSL2(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerLSR2(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerASR2(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerADC(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerSBC(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerROR(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerTST(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerNEG(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerCMP2(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerCMN(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerORR(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerMUL(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerBIC(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerMVN(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerADD400(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerADD401(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerADD410(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerADD411(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerCMP300(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerCMP301(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerCMP310(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerCMP311(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerMOV300(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerMOV301(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerMOV310(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerMOV311(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerBX(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerILL(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerLDR3(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerSTR2(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerSTRH2(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerSTRB2(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerLDRSB(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerLDR2(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerLDRH2(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerLDRB2(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerLDRSH(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerSTR1(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerLDR1(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerSTRB1(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerLDRB1(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerSTRH1(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerLDRH1(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerSTR3(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerLDR4(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerADD5(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerADD6(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerADD7(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerSUB4(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerPUSH(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerPUSHR(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerPOP(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerPOPR(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerBKPT(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerSTMIA(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerLDMIA(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerBEQ(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerBNE(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerBCS(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerBCC(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerBMI(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerBPL(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerBVS(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerBVC(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerBHI(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerBLS(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerBGE(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerBLT(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerBGT(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerBLE(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerSWI(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerB(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, B, LE, ctx->code, cpu->dynarec.epilogue);
	return false;
}

bool _ThumbCompilerBL1(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, gprs) + 15 * sizeof(uint32_t));
	EMIT(ctx, CMP, AL, 1, 2);
	EMIT(ctx, B, NE, ctx->code, cpu->dynarec.epilogue);
	return true;
}

bool _ThumbCompilerBL2(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	flushPC(ctx);
	flushPrefetch(ctx);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, AL, ctx->code, _thumbTable[opcode >> 6]);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, ctx->gpr_15);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
	return false;
}


const ThumbCompiler _thumbCompilerTable[0x400] = {
		DECLARE_THUMB_EMITTER_BLOCK(_ThumbCompiler)
};
