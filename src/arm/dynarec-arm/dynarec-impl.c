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

static void lazySetPrefetch(struct ARMDynarecContext* ctx, uint32_t op0, uint32_t op1) {
	ctx->prefetch_flushed = false;
	ctx->prefetch[0] = op0;
	ctx->prefetch[1] = op1;
}

static void lazySetPrefetchForCurrentPC(struct ARMCore* cpu, struct ARMDynarecContext* ctx) {
	uint16_t op0, op1;
	LOAD_16(op0, (ctx->gpr_15 - 1 * WORD_SIZE_THUMB) & cpu->memory.activeMask, cpu->memory.activeRegion);
	LOAD_16(op1, (ctx->gpr_15 - 0 * WORD_SIZE_THUMB) & cpu->memory.activeMask, cpu->memory.activeRegion);
	lazySetPrefetch(ctx, op0, op1);
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

static void lazyAddCycles(struct ARMDynarecContext* ctx, int cycles) {
	ctx->cycles += cycles;
}

void flushCyclesRegister(struct ARMDynarecContext* ctx) {
	assert(!ctx->cycles);
	if (ctx->cycles_register_valid) {
		assert(!ctx->scratch_in_use[1]);
		EMIT(ctx, LDRI, AL, REG_SCRATCH1, REG_ARMCore, offsetof(struct ARMCore, nextEvent));
		EMIT(ctx, SUB, AL, REG_SCRATCH1, REG_SCRATCH1, REG_CYCLES);
		EMIT(ctx, SUBI, AL, REG_SCRATCH1, REG_SCRATCH1, 1); // MAGIC TRICK: CORRECT THE OFF BY ONE
		EMIT(ctx, STRI, AL, REG_SCRATCH1, REG_ARMCore, offsetof(struct ARMCore, cycles));
		ctx->cycles_register_valid = false;
	}
}

void loadNZCV(struct ARMDynarecContext* ctx) {
	if (ctx->nzcv_location != CONTEXT_NZCV_IN_HOST) {
		if (ctx->nzcv_location == CONTEXT_NZCV_IN_TMPREG) {

			EMIT(ctx, MSR, AL, true, false, REG_NZCV_TMP);
		} else if (ctx->nzcv_location == CONTEXT_NZCV_IN_MEMORY) {
			assert(!ctx->scratch_in_use[0]);
			EMIT(ctx, LDRI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, cpsr));
			EMIT(ctx, MSR, AL, true, false, REG_SCRATCH0);
		} else {
			abort();
		}
		ctx->nzcv_location = CONTEXT_NZCV_IN_HOST;
	}
}

void flushNZCV(struct ARMDynarecContext* ctx) {
	if (ctx->nzcv_location != CONTEXT_NZCV_IN_MEMORY) {
		if (ctx->nzcv_location == CONTEXT_NZCV_IN_HOST) {
			assert(!ctx->scratch_in_use[0]);
			EMIT(ctx, MRS, AL, REG_SCRATCH0);
			EMIT(ctx, MOV_LSRI, AL, REG_SCRATCH0, REG_SCRATCH0, 24);
			EMIT(ctx, STRBI, AL, REG_SCRATCH0, REG_ARMCore, (int)offsetof(struct ARMCore, cpsr) + 3);
		} else if (ctx->nzcv_location == CONTEXT_NZCV_IN_TMPREG) {

			EMIT(ctx, MOV_LSRI, AL, REG_NZCV_TMP, REG_NZCV_TMP, 24);
			EMIT(ctx, STRBI, AL, REG_NZCV_TMP, REG_ARMCore, (int)offsetof(struct ARMCore, cpsr) + 3);
		} else {
			abort();
		}
		ctx->nzcv_location = CONTEXT_NZCV_IN_MEMORY;
	}
}

static void flushPC(struct ARMDynarecContext* ctx) {
	if (!ctx->gpr_15_flushed) {
		EMIT_IMM(ctx, AL, REG_SCRATCH0, ctx->gpr_15);
		EMIT(ctx, STRI, AL, REG_SCRATCH0, REG_ARMCore, 15 * sizeof(uint32_t));
		ctx->gpr_15_flushed = true;
	}
}

static unsigned loadReg(struct ARMDynarecContext* ctx, unsigned guest_reg) {
	assert(guest_reg <= 15);

	if (guest_reg <= 7) {
		switch (guest_reg) {
		case 0: return REG_GUEST_R0;
		case 1: return REG_GUEST_R1;
		case 2: return REG_GUEST_R2;
		case 3: return REG_GUEST_R3;
		case 4: return REG_GUEST_R4;
		case 5: return REG_GUEST_R5;
		case 6: return REG_GUEST_R6;
		case 7: return REG_GUEST_R7;
		default: abort();
		}
	}

	for (unsigned i = 0; i < 3; i++) {
		if (ctx->scratch_in_use[i] && ctx->scratch_guest[i] == guest_reg) {
			unsigned sysreg = i + 1;
			return sysreg;
		}
	}
	for (unsigned i = 0; i < 3; i++) {
		if (!ctx->scratch_in_use[i]) {
			unsigned sysreg = i + 1;
			if (sysreg == REG_NZCV_TMP) {
				assert(ctx->nzcv_location != CONTEXT_NZCV_IN_TMPREG);
			}
			ctx->scratch_in_use[i] = true;
			ctx->scratch_guest[i] = guest_reg;
			if (guest_reg != 15) {
				EMIT(ctx, LDRI, AL, sysreg, REG_ARMCore, guest_reg * sizeof(uint32_t));
			} else {
				EMIT_IMM(ctx, AL, sysreg, ctx->gpr_15);
			}
			return sysreg;
		}
	}
	abort();
}

static void flushReg(struct ARMDynarecContext* ctx, unsigned guest_reg, unsigned sysreg) {
	assert(guest_reg <= 15);

	if (guest_reg <= 7) {
		switch (guest_reg) {
		case 0: assert(sysreg == REG_GUEST_R0); return;
		case 1: assert(sysreg == REG_GUEST_R1); return;
		case 2: assert(sysreg == REG_GUEST_R2); return;
		case 3: assert(sysreg == REG_GUEST_R3); return;
		case 4: assert(sysreg == REG_GUEST_R4); return;
		case 5: assert(sysreg == REG_GUEST_R5); return;
		case 6: assert(sysreg == REG_GUEST_R6); return;
		case 7: assert(sysreg == REG_GUEST_R7); return;
		default: abort();
		}
	}

	assert(sysreg >= 1 && sysreg <= 3);
	unsigned index = sysreg - 1;
	assert(ctx->scratch_in_use[index]);
	assert(ctx->scratch_guest[index] == guest_reg);
	ctx->scratch_in_use[index] = false;
	ctx->scratch_guest[index] = 99;
	EMIT(ctx, STRI, AL, sysreg, REG_ARMCore, guest_reg * sizeof(uint32_t));
}

static void flushRegCache(struct ARMDynarecContext* ctx) {
	EMIT(ctx, STMIA, AL, REG_ARMCore, REGLIST_GUESTREGS);
}

static void destroyAllReg(struct ARMDynarecContext* ctx) {
	for (unsigned i = 0; i < 3; i++) {
		ctx->scratch_in_use[i] = false;
		ctx->scratch_guest[i] = 99;
	}
}

static void checkCycles(struct ARMCore* cpu, struct ARMDynarecContext* ctx) {
	if (!ctx->cycles) {
		return;
	}

	if (ctx->nzcv_location == CONTEXT_NZCV_IN_HOST) {
		assert(!ctx->scratch_in_use[2]);
		EMIT(ctx, MRS, AL, REG_NZCV_TMP);
		ctx->nzcv_location = CONTEXT_NZCV_IN_TMPREG;
	}

	if (!ctx->cycles_register_valid) {
		assert(!ctx->scratch_in_use[0] && !ctx->scratch_in_use[1]);
		EMIT(ctx, LDRI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, cycles));
		EMIT(ctx, LDRI, AL, REG_SCRATCH1, REG_ARMCore, offsetof(struct ARMCore, nextEvent));
		EMIT(ctx, SUB, AL, REG_CYCLES, REG_SCRATCH1, REG_SCRATCH0); // cpu->nextEvent - cpu->cycles
		ctx->cycles++; // MAGIC TRICK: INTENTIONALLY BE OFF BY ONE
		ctx->cycles_register_valid = true;
	}
	EMIT(ctx, SUBS, AL, REG_CYCLES, REG_CYCLES, ctx->cycles);
	ctx->cycles = 0;

	assert(!ctx->scratch_in_use[0]);
	EMIT_IMM(ctx, MI, REG_SCRATCH0, ctx->gpr_15);
	if (ctx->nzcv_location == CONTEXT_NZCV_IN_TMPREG) {
		EMIT(ctx, B, MI, ctx->code, cpu->dynarec.saveNzcvAndCycleCheckHandler);
	} else {
		EMIT(ctx, B, MI, ctx->code, cpu->dynarec.cycleCheckHandler);
	}
}

static void interpretInstruction(struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {
	assert(!ctx->cycles);
	flushRegCache(ctx);
	flushNZCV(ctx);
	flushPC(ctx);
	flushPrefetch(ctx);
	flushCyclesRegister(ctx);
	ThumbInstruction instruction = _thumbTable[opcode >> 6];
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, BL, AL, ctx->code, instruction);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT(ctx, B, AL, ctx->code, cpu->dynarec.epilogue);
}

static void InterpretThumbInstructionNormally(struct ARMCore* cpu) {
	uint32_t opcode = cpu->prefetch[0];
	cpu->prefetch[0] = cpu->prefetch[1];
	cpu->gprs[ARM_PC] += WORD_SIZE_THUMB;
	LOAD_16(cpu->prefetch[1], cpu->gprs[ARM_PC] & cpu->memory.activeMask, cpu->memory.activeRegion);
	ThumbInstruction instruction = _thumbTable[opcode >> 6];
	instruction(cpu, opcode);
}

static void cyclesExceededCallback(struct ARMCore* cpu, int32_t cycles_remaining) {
	cpu->cycles = cpu->nextEvent - cycles_remaining - 1; // MAGIC TRICK: CORRECT THE OFF BY ONE
	uint32_t pc = cpu->gprs[15];
	uint16_t op0, op1;
	LOAD_16(op0, (pc - 1 * WORD_SIZE_THUMB) & cpu->memory.activeMask, cpu->memory.activeRegion);
	LOAD_16(op1, (pc - 0 * WORD_SIZE_THUMB) & cpu->memory.activeMask, cpu->memory.activeRegion);
	cpu->prefetch[0] = op0;
	cpu->prefetch[1] = op1;
}

void ARMDynarecEmitPrelude(struct ARMCore* cpu) {
	code_t* code = (code_t*) cpu->dynarec.buffer;

	// Common prologue
	cpu->dynarec.execute = (void (*)(struct ARMCore*, void*)) code;
	EMIT_L(code, PUSH, AL, 0x4DF0);
	EMIT_L(code, LDMIA, AL, REG_ARMCore, REGLIST_GUESTREGS);
	EMIT_L(code, MOV, AL, 15, 1);

	// Cycle check handler
	cpu->dynarec.saveNzcvAndCycleCheckHandler = (void*) code;
	EMIT_L(code, MOV_LSRI, AL, REG_NZCV_TMP, REG_NZCV_TMP, 24);
	EMIT_L(code, STRBI, AL, REG_NZCV_TMP, REG_ARMCore, (int)offsetof(struct ARMCore, cpsr) + 3);
	cpu->dynarec.cycleCheckHandler = (void*) code;
	EMIT_L(code, STRI, AL, REG_SCRATCH0, REG_ARMCore, 15 * sizeof(uint32_t));
	EMIT_L(code, STMIA, AL, REG_ARMCore, REGLIST_GUESTREGS);
	EMIT_L(code, MOV, AL, 1, REG_CYCLES);
	EMIT_L(code, BL, AL, code, &cyclesExceededCallback);
	// fallthrough to epilogue

	// Common epilogue
	cpu->dynarec.epilogue = (void*) code;
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

	if (cpu->cycles >= cpu->nextEvent) {
		cpu->irqh.processEvents(cpu);
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
			.gpr_15_flushed = true,
			.prefetch_flushed = true,
			.nzcv_location = CONTEXT_NZCV_IN_MEMORY,
		};
		destroyAllReg(&ctx);
		lazySetPrefetchForCurrentPC(cpu, &ctx);

		trace->entry = 1;
		trace->entryPlus4 = (void*) ctx.code;

		bool continue_compilation = true;
		while (true) {
			ctx.gpr_15_flushed = false;
			ctx.gpr_15 += WORD_SIZE_THUMB;

			uint16_t opcode = ctx.prefetch[0];
			lazySetPrefetchForCurrentPC(cpu, &ctx);

			continue_compilation = _thumbCompilerTable[opcode >> 6](cpu, &ctx, opcode);

			if (!continue_compilation){
				assert(!ctx.cycles);
				break;
			}
			checkCycles(cpu, &ctx);
		}

		//__clear_cache(trace->entry, ctx.code);
		__clear_cache(trace->entryPlus4, ctx.code);
		cpu->dynarec.buffer = ctx.code;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define THUMB_ADDITION_S \
	ctx->nzcv_location = CONTEXT_NZCV_IN_HOST;

#define THUMB_SUBTRACTION_S \
	ctx->nzcv_location = CONTEXT_NZCV_IN_HOST;

#define THUMB_NEUTRAL_S \
	assert(ctx->nzcv_location == CONTEXT_NZCV_IN_HOST);

#define THUMB_ADDITION(D, M, N) \
	int n = N; \
	int m = M; \
	D = M + N; \
	THUMB_ADDITION_S(m, n, D)

#define THUMB_SUBTRACTION(D, M, N) \
	int n = N; \
	int m = M; \
	D = M - N; \
	THUMB_SUBTRACTION_S(m, n, D)

#define THUMB_PREFETCH_CYCLES (1 + cpu->memory.activeSeqCycles16)

#define THUMB_LOAD_POST_BODY \
	currentCycles += cpu->memory.activeNonseqCycles16 - cpu->memory.activeSeqCycles16;

#define THUMB_STORE_POST_BODY \
	currentCycles += cpu->memory.activeNonseqCycles16 - cpu->memory.activeSeqCycles16;

#define DEFINE_INSTRUCTION_THUMB(NAME, BODY) \
	static bool _ThumbCompiler ## NAME (struct ARMCore* cpu, struct ARMDynarecContext* ctx, uint16_t opcode) {  \
		int currentCycles = THUMB_PREFETCH_CYCLES; \
		bool continue_compilation = true; \
		BODY; \
		lazyAddCycles(ctx, currentCycles); \
		return continue_compilation; \
	}

#define DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(NAME, BODY) \
	DEFINE_INSTRUCTION_THUMB(NAME, \
		int immediate = (opcode >> 6) & 0x001F; \
		int rd = opcode & 0x0007; \
		int rm = (opcode >> 3) & 0x0007; \
		BODY;)

DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(LSL1,
	printf("lsl1 ");
	loadNZCV(ctx);
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rm = loadReg(ctx, rm);
	EMIT(ctx, MOVS_LSLI, AL, reg_rd, reg_rm, immediate);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_NEUTRAL_S)

DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(LSR1,
	printf("lsr1 ");
	loadNZCV(ctx);
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rm = loadReg(ctx, rm);
	EMIT(ctx, MOVS_LSRI, AL, reg_rd, reg_rm, immediate);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_NEUTRAL_S)

DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(ASR1,
	printf("asr1 ");
	loadNZCV(ctx);
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rm = loadReg(ctx, rm);
	EMIT(ctx, MOVS_ASRI, AL, reg_rd, reg_rm, immediate);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_NEUTRAL_S)

DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(LDR1, interpretInstruction(cpu, ctx, opcode); return false;
	/*cpu->gprs[rd] = cpu->memory.load32(cpu, cpu->gprs[rm] + immediate * 4, &currentCycles); THUMB_LOAD_POST_BODY;*/)
DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(LDRB1, interpretInstruction(cpu, ctx, opcode); return false;
	/*cpu->gprs[rd] = cpu->memory.load8(cpu, cpu->gprs[rm] + immediate, &currentCycles); THUMB_LOAD_POST_BODY;*/)
DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(LDRH1, interpretInstruction(cpu, ctx, opcode); return false;
	/*cpu->gprs[rd] = cpu->memory.load16(cpu, cpu->gprs[rm] + immediate * 2, &currentCycles); THUMB_LOAD_POST_BODY;*/)
DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(STR1, interpretInstruction(cpu, ctx, opcode); return false;
	/*cpu->memory.store32(cpu, cpu->gprs[rm] + immediate * 4, cpu->gprs[rd], &currentCycles); THUMB_STORE_POST_BODY;*/)
DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(STRB1, interpretInstruction(cpu, ctx, opcode); return false;
	/*cpu->memory.store8(cpu, cpu->gprs[rm] + immediate, cpu->gprs[rd], &currentCycles); THUMB_STORE_POST_BODY;*/)
DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(STRH1, interpretInstruction(cpu, ctx, opcode); return false;
	/*cpu->memory.store16(cpu, cpu->gprs[rm] + immediate * 2, cpu->gprs[rd], &currentCycles); THUMB_STORE_POST_BODY;*/)

#define DEFINE_DATA_FORM_1_INSTRUCTION_THUMB(NAME, BODY) \
	DEFINE_INSTRUCTION_THUMB(NAME, \
		int rm = (opcode >> 6) & 0x0007; \
		int rd = opcode & 0x0007; \
		int rn = (opcode >> 3) & 0x0007; \
		BODY;)

DEFINE_DATA_FORM_1_INSTRUCTION_THUMB(ADD3,
	printf("add3 ");
	if (ctx->nzcv_location == CONTEXT_NZCV_IN_TMPREG)
		ctx->scratch_in_use[2] = false; // This is safe because we don't care about the previous value.
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	unsigned reg_rm = loadReg(ctx, rm);
	EMIT(ctx, ADDS, AL, reg_rd, reg_rn, reg_rm);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_ADDITION_S)
DEFINE_DATA_FORM_1_INSTRUCTION_THUMB(SUB3,
	printf("sub3 ");
	if (ctx->nzcv_location == CONTEXT_NZCV_IN_TMPREG)
		ctx->scratch_in_use[2] = false; // This is safe because we don't care about the previous value.
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	unsigned reg_rm = loadReg(ctx, rm);
	EMIT(ctx, SUBS, AL, reg_rd, reg_rn, reg_rm);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_SUBTRACTION_S)

#define DEFINE_DATA_FORM_2_INSTRUCTION_THUMB(NAME, BODY) \
	DEFINE_INSTRUCTION_THUMB(NAME, \
		int immediate = (opcode >> 6) & 0x0007; \
		int rd = opcode & 0x0007; \
		int rn = (opcode >> 3) & 0x0007; \
		BODY;)

DEFINE_DATA_FORM_2_INSTRUCTION_THUMB(ADD1,
	printf("add1 ");
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	EMIT(ctx, ADDSI, AL, reg_rd, reg_rn, immediate);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_ADDITION_S)
DEFINE_DATA_FORM_2_INSTRUCTION_THUMB(SUB1,
	printf("sub1 ");
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	EMIT(ctx, SUBSI, AL, reg_rd, reg_rn, immediate);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_SUBTRACTION_S)

#define DEFINE_DATA_FORM_3_INSTRUCTION_THUMB(NAME, BODY) \
	DEFINE_INSTRUCTION_THUMB(NAME, \
		int rd = (opcode >> 8) & 0x0007; \
		int immediate = opcode & 0x00FF; \
		BODY;)

DEFINE_DATA_FORM_3_INSTRUCTION_THUMB(ADD2,
	printf("add2 ");
	unsigned reg_rd = loadReg(ctx, rd);
	EMIT(ctx, ADDSI, AL, reg_rd, reg_rd, immediate);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_ADDITION_S)
DEFINE_DATA_FORM_3_INSTRUCTION_THUMB(CMP1,
	printf("cmp1 ");
	unsigned reg_rd = loadReg(ctx, rd);
	EMIT(ctx, CMPI, AL, reg_rd, immediate);
	destroyAllReg(ctx);
	THUMB_SUBTRACTION_S)
DEFINE_DATA_FORM_3_INSTRUCTION_THUMB(MOV1,
	printf("mov1 ");
	loadNZCV(ctx);
	unsigned reg_rd = loadReg(ctx, rd);
	EMIT(ctx, MOVSI, AL, reg_rd, immediate);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_NEUTRAL_S)
DEFINE_DATA_FORM_3_INSTRUCTION_THUMB(SUB2,
	printf("sub2 ");
	unsigned reg_rd = loadReg(ctx, rd);
	EMIT(ctx, SUBSI, AL, reg_rd, reg_rd, immediate);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_SUBTRACTION_S)

#define DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(NAME, BODY) \
	DEFINE_INSTRUCTION_THUMB(NAME, \
		int rd = opcode & 0x0007; \
		int rn = (opcode >> 3) & 0x0007; \
		BODY;)

DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(AND,
	printf("and ");
	loadNZCV(ctx);
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	EMIT(ctx, ANDS, AL, reg_rd, reg_rd, reg_rn);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_NEUTRAL_S)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(EOR,
	printf("eor ");
	loadNZCV(ctx);
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	EMIT(ctx, EORS, AL, reg_rd, reg_rd, reg_rn);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_NEUTRAL_S)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(LSL2,
	printf("lsl2 ");
	loadNZCV(ctx);
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	EMIT(ctx, MOVS_LSL, AL, reg_rd, reg_rd, reg_rn);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_NEUTRAL_S)

DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(LSR2,
	printf("lsl2 ");
	loadNZCV(ctx);
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	EMIT(ctx, MOVS_LSR, AL, reg_rd, reg_rd, reg_rn);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_NEUTRAL_S)

DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(ASR2,
	printf("asr2 ");
	loadNZCV(ctx);
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	EMIT(ctx, MOVS_ASR, AL, reg_rd, reg_rd, reg_rn);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_NEUTRAL_S)

DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(ADC,
	printf("adc ");
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	EMIT(ctx, ADCS, AL, reg_rd, reg_rd, reg_rn);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_ADDITION_S)

DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(SBC,
	printf("sbc ");
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	EMIT(ctx, SBCS, AL, reg_rd, reg_rd, reg_rn);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_SUBTRACTION_S)

DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(ROR,
	printf("ror ");
	loadNZCV(ctx);
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	EMIT(ctx, MOVS_ROR, AL, reg_rd, reg_rd, reg_rn);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_NEUTRAL_S)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(TST,
	printf("tst ");
	loadNZCV(ctx);
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	EMIT(ctx, TST, AL, reg_rd, reg_rn);
	destroyAllReg(ctx);
	THUMB_NEUTRAL_S)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(NEG,
	printf("neg ");
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	EMIT(ctx, RSBSI, AL, reg_rd, reg_rn, 0);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_SUBTRACTION_S)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(CMP2,
	printf("cmp2 ");
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	EMIT(ctx, CMP, AL, reg_rd, reg_rn);
	destroyAllReg(ctx);
	THUMB_SUBTRACTION_S)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(CMN,
	printf("cmn ");
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	EMIT(ctx, CMN, AL, reg_rd, reg_rn);
	destroyAllReg(ctx);
	THUMB_SUBTRACTION_S)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(ORR,
	printf("orr ");
	loadNZCV(ctx);
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	EMIT(ctx, ORRS, AL, reg_rd, reg_rd, reg_rn);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_NEUTRAL_S)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(MUL, interpretInstruction(cpu, ctx, opcode); return false;
	/*ARM_WAIT_MUL(cpu->gprs[rd]); cpu->gprs[rd] *= cpu->gprs[rn]; THUMB_NEUTRAL_S( , , cpu->gprs[rd]); currentCycles += cpu->memory.activeNonseqCycles16 - cpu->memory.activeSeqCycles16*/)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(BIC,
	printf("bic ");
	loadNZCV(ctx);
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	EMIT(ctx, BICS, AL, reg_rd, reg_rd, reg_rn);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_NEUTRAL_S)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(MVN,
	printf("mvn ");
	loadNZCV(ctx);
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rn = loadReg(ctx, rn);
	EMIT(ctx, MVNS, AL, reg_rd, reg_rn);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_NEUTRAL_S)

#define DEFINE_INSTRUCTION_WITH_HIGH_EX_THUMB(NAME, H1, H2, BODY) \
	DEFINE_INSTRUCTION_THUMB(NAME, \
		int rd = (opcode & 0x0007) | H1; \
		int rm = ((opcode >> 3) & 0x0007) | H2; \
		BODY;)

#define DEFINE_INSTRUCTION_WITH_HIGH_THUMB(NAME, BODY) \
	DEFINE_INSTRUCTION_WITH_HIGH_EX_THUMB(NAME ## 00, 0, 0, BODY) \
	DEFINE_INSTRUCTION_WITH_HIGH_EX_THUMB(NAME ## 01, 0, 8, BODY) \
	DEFINE_INSTRUCTION_WITH_HIGH_EX_THUMB(NAME ## 10, 8, 0, BODY) \
	DEFINE_INSTRUCTION_WITH_HIGH_EX_THUMB(NAME ## 11, 8, 8, BODY)

DEFINE_INSTRUCTION_WITH_HIGH_THUMB(ADD4,
	interpretInstruction(cpu, ctx, opcode); return false;
	/*cpu->gprs[rd] += cpu->gprs[rm];
	if (rd == ARM_PC) {
		THUMB_WRITE_PC;
	}*/)

DEFINE_INSTRUCTION_WITH_HIGH_THUMB(CMP3, interpretInstruction(cpu, ctx, opcode); return false;
	/*int32_t aluOut = cpu->gprs[rd] - cpu->gprs[rm]; THUMB_SUBTRACTION_S(cpu->gprs[rd], cpu->gprs[rm], aluOut)*/)
DEFINE_INSTRUCTION_WITH_HIGH_THUMB(MOV3,
	interpretInstruction(cpu, ctx, opcode); return false;
	/*cpu->gprs[rd] = cpu->gprs[rm];
	if (rd == ARM_PC) {
		THUMB_WRITE_PC;
	}*/)

#define DEFINE_IMMEDIATE_WITH_REGISTER_THUMB(NAME, BODY) \
	DEFINE_INSTRUCTION_THUMB(NAME, \
		int rd = (opcode >> 8) & 0x0007; \
		int immediate = (opcode & 0x00FF) << 2; \
		BODY;)

DEFINE_IMMEDIATE_WITH_REGISTER_THUMB(LDR3, interpretInstruction(cpu, ctx, opcode); return false;
	/*cpu->gprs[rd] = cpu->memory.load32(cpu, (cpu->gprs[ARM_PC] & 0xFFFFFFFC) + immediate, &currentCycles); THUMB_LOAD_POST_BODY;*/)
DEFINE_IMMEDIATE_WITH_REGISTER_THUMB(LDR4, interpretInstruction(cpu, ctx, opcode); return false;
	/*cpu->gprs[rd] = cpu->memory.load32(cpu, cpu->gprs[ARM_SP] + immediate, &currentCycles); THUMB_LOAD_POST_BODY;*/)
DEFINE_IMMEDIATE_WITH_REGISTER_THUMB(STR3, interpretInstruction(cpu, ctx, opcode); return false;
	/*cpu->memory.store32(cpu, cpu->gprs[ARM_SP] + immediate, cpu->gprs[rd], &currentCycles); THUMB_STORE_POST_BODY;*/)

DEFINE_IMMEDIATE_WITH_REGISTER_THUMB(ADD5,
	uint32_t imm_to_load = (ctx->gpr_15 & 0xFFFFFFFC) + immediate;
	unsigned reg_rd = loadReg(ctx, rd);
	EMIT_IMM(ctx, AL, reg_rd, imm_to_load);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);)
DEFINE_IMMEDIATE_WITH_REGISTER_THUMB(ADD6,
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_sp = loadReg(ctx, 13);
	EMIT(ctx, ADDI, AL, reg_rd, reg_sp, immediate);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);)

#define DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(NAME, BODY) \
	DEFINE_INSTRUCTION_THUMB(NAME, \
		int rm = (opcode >> 6) & 0x0007; \
		int rd = opcode & 0x0007; \
		int rn = (opcode >> 3) & 0x0007; \
		BODY;)

DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(LDR2, interpretInstruction(cpu, ctx, opcode); return false;
	/*cpu->gprs[rd] = cpu->memory.load32(cpu, cpu->gprs[rn] + cpu->gprs[rm], &currentCycles); THUMB_LOAD_POST_BODY;*/)
DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(LDRB2,interpretInstruction(cpu, ctx, opcode); return false;
	/* cpu->gprs[rd] = cpu->memory.load8(cpu, cpu->gprs[rn] + cpu->gprs[rm], &currentCycles); THUMB_LOAD_POST_BODY;*/)
DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(LDRH2,interpretInstruction(cpu, ctx, opcode); return false;
	/* cpu->gprs[rd] = cpu->memory.load16(cpu, cpu->gprs[rn] + cpu->gprs[rm], &currentCycles); THUMB_LOAD_POST_BODY;*/)
DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(LDRSB,interpretInstruction(cpu, ctx, opcode); return false;
	/* cpu->gprs[rd] = ARM_SXT_8(cpu->memory.load8(cpu, cpu->gprs[rn] + cpu->gprs[rm], &currentCycles)); THUMB_LOAD_POST_BODY;*/)
DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(LDRSH,interpretInstruction(cpu, ctx, opcode); return false;
	/* rm = cpu->gprs[rn] + cpu->gprs[rm]; cpu->gprs[rd] = rm & 1 ? ARM_SXT_8(cpu->memory.load16(cpu, rm, &currentCycles)) : ARM_SXT_16(cpu->memory.load16(cpu, rm, &currentCycles)); THUMB_LOAD_POST_BODY;*/)
DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(STR2, interpretInstruction(cpu, ctx, opcode); return false;
	/*cpu->memory.store32(cpu, cpu->gprs[rn] + cpu->gprs[rm], cpu->gprs[rd], &currentCycles); THUMB_STORE_POST_BODY;*/)
DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(STRB2,interpretInstruction(cpu, ctx, opcode); return false;
	/* cpu->memory.store8(cpu, cpu->gprs[rn] + cpu->gprs[rm], cpu->gprs[rd], &currentCycles); THUMB_STORE_POST_BODY;*/)
DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(STRH2,interpretInstruction(cpu, ctx, opcode); return false;
	/* cpu->memory.store16(cpu, cpu->gprs[rn] + cpu->gprs[rm], cpu->gprs[rd], &currentCycles); THUMB_STORE_POST_BODY;*/)

#define DEFINE_LOAD_STORE_MULTIPLE_THUMB(NAME, RN, LS, DIRECTION, PRE_BODY, WRITEBACK) \
	DEFINE_INSTRUCTION_THUMB(NAME, \
		int rn = RN; \
		UNUSED(rn); \
		int rs = opcode & 0xFF; \
		int32_t address = cpu->gprs[RN]; \
		PRE_BODY; \
		/* address = cpu->memory. LS ## Multiple(cpu, address, rs, LSM_ ## DIRECTION, &currentCycles); */ \
		WRITEBACK;)

DEFINE_LOAD_STORE_MULTIPLE_THUMB(LDMIA,
	(opcode >> 8) & 0x0007,
	load,
	IA,
	,
	interpretInstruction(cpu, ctx, opcode); return false;
	/*THUMB_LOAD_POST_BODY;
	if (!((1 << rn) & rs)) {
		cpu->gprs[rn] = address;
	}*/)

DEFINE_LOAD_STORE_MULTIPLE_THUMB(STMIA,
	(opcode >> 8) & 0x0007,
	store,
	IA,
	,
	interpretInstruction(cpu, ctx, opcode); return false;
	/*THUMB_STORE_POST_BODY;
	cpu->gprs[rn] = address;*/)

#define DEFINE_CONDITIONAL_BRANCH_THUMB(COND) \
	DEFINE_INSTRUCTION_THUMB(B ## COND, \
		interpretInstruction(cpu, ctx, opcode); return false; \
		/* if (ARM_COND_ ## COND) { */ \
			/* int8_t immediate = opcode; */ \
			/* cpu->gprs[ARM_PC] += (int32_t) immediate << 1; */ \
			/* THUMB_WRITE_PC; */ \
		/* } */)

DEFINE_CONDITIONAL_BRANCH_THUMB(EQ)
DEFINE_CONDITIONAL_BRANCH_THUMB(NE)
DEFINE_CONDITIONAL_BRANCH_THUMB(CS)
DEFINE_CONDITIONAL_BRANCH_THUMB(CC)
DEFINE_CONDITIONAL_BRANCH_THUMB(MI)
DEFINE_CONDITIONAL_BRANCH_THUMB(PL)
DEFINE_CONDITIONAL_BRANCH_THUMB(VS)
DEFINE_CONDITIONAL_BRANCH_THUMB(VC)
DEFINE_CONDITIONAL_BRANCH_THUMB(LS)
DEFINE_CONDITIONAL_BRANCH_THUMB(HI)
DEFINE_CONDITIONAL_BRANCH_THUMB(GE)
DEFINE_CONDITIONAL_BRANCH_THUMB(LT)
DEFINE_CONDITIONAL_BRANCH_THUMB(GT)
DEFINE_CONDITIONAL_BRANCH_THUMB(LE)

DEFINE_INSTRUCTION_THUMB(ADD7, interpretInstruction(cpu, ctx, opcode); return false; \
		/*cpu->gprs[ARM_SP] += (opcode & 0x7F) << 2*/)
DEFINE_INSTRUCTION_THUMB(SUB4, interpretInstruction(cpu, ctx, opcode); return false; \
		/*cpu->gprs[ARM_SP] -= (opcode & 0x7F) << 2*/)

DEFINE_LOAD_STORE_MULTIPLE_THUMB(POP,
	ARM_SP,
	load,
	IA,
	,
	interpretInstruction(cpu, ctx, opcode); return false; \
		/*THUMB_LOAD_POST_BODY;
	cpu->gprs[ARM_SP] = address*/)

DEFINE_LOAD_STORE_MULTIPLE_THUMB(POPR,
	ARM_SP,
	load,
	IA,
	/*rs |= 1 << ARM_PC*/,
	interpretInstruction(cpu, ctx, opcode); return false; \
		/*THUMB_LOAD_POST_BODY;
	cpu->gprs[ARM_SP] = address;
	THUMB_WRITE_PC;*/)

DEFINE_LOAD_STORE_MULTIPLE_THUMB(PUSH,
	ARM_SP,
	store,
	DB,
	,
	interpretInstruction(cpu, ctx, opcode); return false; \
		/*THUMB_STORE_POST_BODY;
	cpu->gprs[ARM_SP] = address*/)

DEFINE_LOAD_STORE_MULTIPLE_THUMB(PUSHR,
	ARM_SP,
	store,
	DB,
	/*rs |= 1 << ARM_LR*/,
	interpretInstruction(cpu, ctx, opcode); return false; \
		/*THUMB_STORE_POST_BODY;
	cpu->gprs[ARM_SP] = address*/)

DEFINE_INSTRUCTION_THUMB(ILL, interpretInstruction(cpu, ctx, opcode); return false; \
		/*ARM_ILL*/)
DEFINE_INSTRUCTION_THUMB(BKPT, interpretInstruction(cpu, ctx, opcode); return false; \
		/*cpu->irqh.bkpt16(cpu, opcode & 0xFF);*/)
DEFINE_INSTRUCTION_THUMB(B,
	interpretInstruction(cpu, ctx, opcode); return false; \
		/*int16_t immediate = (opcode & 0x07FF) << 5;
	cpu->gprs[ARM_PC] += (((int32_t) immediate) >> 4);
	THUMB_WRITE_PC;*/)

DEFINE_INSTRUCTION_THUMB(BL1,
	interpretInstruction(cpu, ctx, opcode); return false; \
		/*int16_t immediate = (opcode & 0x07FF) << 5;
	cpu->gprs[ARM_LR] = cpu->gprs[ARM_PC] + (((int32_t) immediate) << 7);*/)

DEFINE_INSTRUCTION_THUMB(BL2,
	interpretInstruction(cpu, ctx, opcode); return false; \
		/*uint16_t immediate = (opcode & 0x07FF) << 1;
	uint32_t pc = cpu->gprs[ARM_PC];
	cpu->gprs[ARM_PC] = cpu->gprs[ARM_LR] + immediate;
	cpu->gprs[ARM_LR] = pc - 1;
	THUMB_WRITE_PC;*/)

DEFINE_INSTRUCTION_THUMB(BX,
	interpretInstruction(cpu, ctx, opcode); return false; \
		/*int rm = (opcode >> 3) & 0xF;
	_ARMSetMode(cpu, cpu->gprs[rm] & 0x00000001);
	int misalign = 0;
	if (rm == ARM_PC) {
		misalign = cpu->gprs[rm] & 0x00000002;
	}
	cpu->gprs[ARM_PC] = (cpu->gprs[rm] & 0xFFFFFFFE) - misalign;
	if (cpu->executionMode == MODE_THUMB) {
		THUMB_WRITE_PC;
	} else {
		ARM_WRITE_PC;
	}*/)

DEFINE_INSTRUCTION_THUMB(SWI, interpretInstruction(cpu, ctx, opcode); return false; \
		/*cpu->irqh.swi16(cpu, opcode & 0xFF)*/)

const ThumbCompiler _thumbCompilerTable[0x400] = {
		DECLARE_THUMB_EMITTER_BLOCK(_ThumbCompiler)
};
