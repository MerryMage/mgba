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

void flushCycles(struct ARMDynarecContext* ctx) {
	if (ctx->cycles == 0)
		return;
	assert(!ctx->scratch_in_use[0]);
	EMIT(ctx, LDRI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, cycles));
	if (ctx->cycles <= 255) {
		EMIT(ctx, ADDI, AL, REG_SCRATCH0, REG_SCRATCH0, ctx->cycles);
	} else {
		assert(!ctx->scratch_in_use[1]);
		EMIT_IMM(ctx, AL, REG_SCRATCH1, ctx->cycles);
		EMIT(ctx, ADD, AL, REG_SCRATCH0, REG_SCRATCH0, REG_SCRATCH1);
	}
	EMIT(ctx, STRI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, cycles));
	ctx->cycles = 0;
}

void loadNZCV(struct ARMDynarecContext* ctx) {
	if (!ctx->nzcv_in_host_nzcv) {
		assert(!ctx->scratch_in_use[0]);
		EMIT(ctx, LDRI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, cpsr));
		EMIT(ctx, MSR, AL, true, false, REG_SCRATCH0);
		ctx->nzcv_in_host_nzcv = true;
	}
}

void flushNZCV(struct ARMDynarecContext* ctx) {
	if (ctx->nzcv_in_host_nzcv) {
		assert(!ctx->scratch_in_use[0]);
		EMIT(ctx, MRS, AL, REG_SCRATCH0);
		EMIT(ctx, MOV_LSRI, AL, REG_SCRATCH0, REG_SCRATCH0, 24);
		EMIT(ctx, STRBI, AL, REG_SCRATCH0, REG_ARMCore, (int)offsetof(struct ARMCore, cpsr) + 3);
		ctx->nzcv_in_host_nzcv = false;
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
	flushCycles(ctx);
	flushNZCV(ctx);
	flushPC(ctx);
	ThumbInstruction instruction = _thumbTable[opcode >> 6];
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT_IMM(ctx, AL, 1, opcode);
	EMIT(ctx, BL, AL, ctx->code, instruction);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT(ctx, POP, AL, REGLIST_RETURN);
}

static unsigned loadReg(struct ARMDynarecContext* ctx, unsigned guest_reg) {
	assert(guest_reg <= 15);
	for (unsigned i = 0; i < 3; i++) {
		if (ctx->scratch_in_use[i] && ctx->scratch_guest[i] == guest_reg) {
			unsigned sysreg = i + 1;
			return sysreg;
		}
	}
	for (unsigned i = 0; i < 3; i++) {
		if (!ctx->scratch_in_use[i]) {
			unsigned sysreg = i + 1;
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
	assert(sysreg >= 1 && sysreg <= 3);
	unsigned index = sysreg - 1;
	assert(ctx->scratch_in_use[index]);
	assert(ctx->scratch_guest[index] == guest_reg);
	ctx->scratch_in_use[index] = false;
	ctx->scratch_guest[index] = 99;
	EMIT(ctx, STRI, AL, sysreg, REG_ARMCore, guest_reg * sizeof(uint32_t));
}

static void destroyAllReg(struct ARMDynarecContext* ctx) {
	for (unsigned i = 0; i < 3; i++) {
		ctx->scratch_in_use[i] = false;
		ctx->scratch_guest[i] = 99;
	}
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
//	printf("%08X (%c)\n", trace->start, trace->mode == MODE_THUMB ? 'T' : 'A');
//	printf("%u\n", cpu->nextEvent - cpu->cycles);
#endif

	struct ARMDynarecContext ctx = {
		.code = cpu->dynarec.buffer,
		.gpr_15 = trace->start + 1 * WORD_SIZE_THUMB,
		.cycles = 0,
		.gpr_15_flushed = true,
		.prefetch_flushed = true,
		.nzcv_in_host_nzcv = false,
	};
	destroyAllReg(&ctx);

	if (trace->mode == MODE_ARM) {
		return;
	} else {
		trace->entry = 1;
		trace->entryPlus4 = (void*) ctx.code;
		ctx.gpr_15 = trace->start + 3 * WORD_SIZE_THUMB;

		bool continue_compilation = true;
		while (continue_compilation) {
			ctx.gpr_15_flushed = false;
			ctx.gpr_15 += WORD_SIZE_THUMB;

			uint16_t opcode, op0, op1;
			LOAD_16(opcode, (ctx.gpr_15 - 2 * WORD_SIZE_THUMB) & cpu->memory.activeMask, cpu->memory.activeRegion);
			LOAD_16(op0,    (ctx.gpr_15 - 1 * WORD_SIZE_THUMB) & cpu->memory.activeMask, cpu->memory.activeRegion);
			LOAD_16(op1,    (ctx.gpr_15 - 0 * WORD_SIZE_THUMB) & cpu->memory.activeMask, cpu->memory.activeRegion);
			lazySetPrefetch(&ctx, op0, op1);

			continue_compilation = _thumbCompilerTable[opcode >> 6](cpu, &ctx, opcode);
		}
	}
	//__clear_cache(trace->entry, ctx.code);
	__clear_cache(trace->entryPlus4, ctx.code);
	cpu->dynarec.buffer = ctx.code;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define THUMB_ADDITION_S(M, N, D) \
	ctx->nzcv_in_host_nzcv = true;

#define THUMB_SUBTRACTION_S(M, N, D) \
	ctx->nzcv_in_host_nzcv = true;

#define THUMB_NEUTRAL_S \
	assert(ctx->nzcv_in_host_nzcv == true);

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
	printf("compiling lsl1");
	loadNZCV(ctx);
	unsigned reg_rd = loadReg(ctx, rd);
	unsigned reg_rm = loadReg(ctx, rm);
	EMIT(ctx, MOVS_LSLI, AL, reg_rd, reg_rm, immediate);
	flushReg(ctx, rd, reg_rd);
	destroyAllReg(ctx);
	THUMB_NEUTRAL_S)

DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(LSR1,
	interpretInstruction(ctx, opcode);
	continue_compilation = false;
	/*if (!immediate) {
		cpu->cpsr.c = ARM_SIGN(cpu->gprs[rm]);
		cpu->gprs[rd] = 0;
	} else {
		cpu->cpsr.c = (cpu->gprs[rm] >> (immediate - 1)) & 1;
		cpu->gprs[rd] = ((uint32_t) cpu->gprs[rm]) >> immediate;
	}
	THUMB_NEUTRAL_S( , , cpu->gprs[rd]);*/)

DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(ASR1,
	interpretInstruction(ctx, opcode);
	continue_compilation = false;
	/*if (!immediate) {
		cpu->cpsr.c = ARM_SIGN(cpu->gprs[rm]);
		if (cpu->cpsr.c) {
			cpu->gprs[rd] = 0xFFFFFFFF;
		} else {
			cpu->gprs[rd] = 0;
		}
	} else {
		cpu->cpsr.c = (cpu->gprs[rm] >> (immediate - 1)) & 1;
		cpu->gprs[rd] = cpu->gprs[rm] >> immediate;
	}
	THUMB_NEUTRAL_S( , , cpu->gprs[rd]);*/)

DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(LDR1, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->gprs[rd] = cpu->memory.load32(cpu, cpu->gprs[rm] + immediate * 4, &currentCycles); THUMB_LOAD_POST_BODY;*/)
DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(LDRB1, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->gprs[rd] = cpu->memory.load8(cpu, cpu->gprs[rm] + immediate, &currentCycles); THUMB_LOAD_POST_BODY;*/)
DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(LDRH1, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->gprs[rd] = cpu->memory.load16(cpu, cpu->gprs[rm] + immediate * 2, &currentCycles); THUMB_LOAD_POST_BODY;*/)
DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(STR1, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->memory.store32(cpu, cpu->gprs[rm] + immediate * 4, cpu->gprs[rd], &currentCycles); THUMB_STORE_POST_BODY;*/)
DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(STRB1, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->memory.store8(cpu, cpu->gprs[rm] + immediate, cpu->gprs[rd], &currentCycles); THUMB_STORE_POST_BODY;*/)
DEFINE_IMMEDIATE_5_INSTRUCTION_THUMB(STRH1, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->memory.store16(cpu, cpu->gprs[rm] + immediate * 2, cpu->gprs[rd], &currentCycles); THUMB_STORE_POST_BODY;*/)

#define DEFINE_DATA_FORM_1_INSTRUCTION_THUMB(NAME, BODY) \
	DEFINE_INSTRUCTION_THUMB(NAME, \
		int rm = (opcode >> 6) & 0x0007; \
		int rd = opcode & 0x0007; \
		int rn = (opcode >> 3) & 0x0007; \
		BODY;)

DEFINE_DATA_FORM_1_INSTRUCTION_THUMB(ADD3, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*THUMB_ADDITION(cpu->gprs[rd], cpu->gprs[rn], cpu->gprs[rm])*/)
DEFINE_DATA_FORM_1_INSTRUCTION_THUMB(SUB3, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*THUMB_SUBTRACTION(cpu->gprs[rd], cpu->gprs[rn], cpu->gprs[rm])*/)

#define DEFINE_DATA_FORM_2_INSTRUCTION_THUMB(NAME, BODY) \
	DEFINE_INSTRUCTION_THUMB(NAME, \
		int immediate = (opcode >> 6) & 0x0007; \
		int rd = opcode & 0x0007; \
		int rn = (opcode >> 3) & 0x0007; \
		BODY;)

DEFINE_DATA_FORM_2_INSTRUCTION_THUMB(ADD1, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*THUMB_ADDITION(cpu->gprs[rd], cpu->gprs[rn], immediate)*/)
DEFINE_DATA_FORM_2_INSTRUCTION_THUMB(SUB1, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*THUMB_SUBTRACTION(cpu->gprs[rd], cpu->gprs[rn], immediate)*/)

#define DEFINE_DATA_FORM_3_INSTRUCTION_THUMB(NAME, BODY) \
	DEFINE_INSTRUCTION_THUMB(NAME, \
		int rd = (opcode >> 8) & 0x0007; \
		int immediate = opcode & 0x00FF; \
		BODY;)

DEFINE_DATA_FORM_3_INSTRUCTION_THUMB(ADD2, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*THUMB_ADDITION(cpu->gprs[rd], cpu->gprs[rd], immediate)*/)
DEFINE_DATA_FORM_3_INSTRUCTION_THUMB(CMP1, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*int aluOut = cpu->gprs[rd] - immediate; THUMB_SUBTRACTION_S(cpu->gprs[rd], immediate, aluOut)*/)
DEFINE_DATA_FORM_3_INSTRUCTION_THUMB(MOV1, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->gprs[rd] = immediate; THUMB_NEUTRAL_S(, , cpu->gprs[rd])*/)
DEFINE_DATA_FORM_3_INSTRUCTION_THUMB(SUB2, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*THUMB_SUBTRACTION(cpu->gprs[rd], cpu->gprs[rd], immediate)*/)

#define DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(NAME, BODY) \
	DEFINE_INSTRUCTION_THUMB(NAME, \
		int rd = opcode & 0x0007; \
		int rn = (opcode >> 3) & 0x0007; \
		BODY;)

DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(AND, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->gprs[rd] = cpu->gprs[rd] & cpu->gprs[rn]; THUMB_NEUTRAL_S( , , cpu->gprs[rd])*/)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(EOR, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->gprs[rd] = cpu->gprs[rd] ^ cpu->gprs[rn]; THUMB_NEUTRAL_S( , , cpu->gprs[rd])*/)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(LSL2,
	interpretInstruction(ctx, opcode); continue_compilation = false;
	/*int rs = cpu->gprs[rn] & 0xFF;
	if (rs) {
		if (rs < 32) {
			cpu->cpsr.c = (cpu->gprs[rd] >> (32 - rs)) & 1;
			cpu->gprs[rd] <<= rs;
		} else {
			if (rs > 32) {
				cpu->cpsr.c = 0;
			} else {
				cpu->cpsr.c = cpu->gprs[rd] & 0x00000001;
			}
			cpu->gprs[rd] = 0;
		}
	}
	THUMB_NEUTRAL_S( , , cpu->gprs[rd])*/)

DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(LSR2,
	interpretInstruction(ctx, opcode); continue_compilation = false;
	/*int rs = cpu->gprs[rn] & 0xFF;
	if (rs) {
		if (rs < 32) {
			cpu->cpsr.c = (cpu->gprs[rd] >> (rs - 1)) & 1;
			cpu->gprs[rd] = (uint32_t) cpu->gprs[rd] >> rs;
		} else {
			if (rs > 32) {
				cpu->cpsr.c = 0;
			} else {
				cpu->cpsr.c = ARM_SIGN(cpu->gprs[rd]);
			}
			cpu->gprs[rd] = 0;
		}
	}
	THUMB_NEUTRAL_S( , , cpu->gprs[rd])*/)

DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(ASR2,
	interpretInstruction(ctx, opcode); continue_compilation = false;
	/*int rs = cpu->gprs[rn] & 0xFF;
	if (rs) {
		if (rs < 32) {
			cpu->cpsr.c = (cpu->gprs[rd] >> (rs - 1)) & 1;
			cpu->gprs[rd] >>= rs;
		} else {
			cpu->cpsr.c = ARM_SIGN(cpu->gprs[rd]);
			if (cpu->cpsr.c) {
				cpu->gprs[rd] = 0xFFFFFFFF;
			} else {
				cpu->gprs[rd] = 0;
			}
		}
	}
	THUMB_NEUTRAL_S( , , cpu->gprs[rd])*/)

DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(ADC,
	interpretInstruction(ctx, opcode); continue_compilation = false;
	/*int n = cpu->gprs[rn];
	int d = cpu->gprs[rd];
	cpu->gprs[rd] = d + n + cpu->cpsr.c;
	THUMB_ADDITION_S(d, n, cpu->gprs[rd]);*/)

DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(SBC,
	interpretInstruction(ctx, opcode); continue_compilation = false;
	/*int n = cpu->gprs[rn] + !cpu->cpsr.c;
	int d = cpu->gprs[rd];
	cpu->gprs[rd] = d - n;
	THUMB_SUBTRACTION_S(d, n, cpu->gprs[rd]);*/)

DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(ROR,
	interpretInstruction(ctx, opcode); continue_compilation = false;
	/*int rs = cpu->gprs[rn] & 0xFF;
	if (rs) {
		int r4 = rs & 0x1F;
		if (r4 > 0) {
			cpu->cpsr.c = (cpu->gprs[rd] >> (r4 - 1)) & 1;
			cpu->gprs[rd] = ROR(cpu->gprs[rd], r4);
		} else {
			cpu->cpsr.c = ARM_SIGN(cpu->gprs[rd]);
		}
	}
	THUMB_NEUTRAL_S( , , cpu->gprs[rd]);*/)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(TST, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*int32_t aluOut = cpu->gprs[rd] & cpu->gprs[rn]; THUMB_NEUTRAL_S(cpu->gprs[rd], cpu->gprs[rn], aluOut)*/)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(NEG, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*THUMB_SUBTRACTION(cpu->gprs[rd], 0, cpu->gprs[rn])*/)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(CMP2, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*int32_t aluOut = cpu->gprs[rd] - cpu->gprs[rn]; THUMB_SUBTRACTION_S(cpu->gprs[rd], cpu->gprs[rn], aluOut)*/)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(CMN, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*int32_t aluOut = cpu->gprs[rd] + cpu->gprs[rn]; THUMB_ADDITION_S(cpu->gprs[rd], cpu->gprs[rn], aluOut)*/)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(ORR, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->gprs[rd] = cpu->gprs[rd] | cpu->gprs[rn]; THUMB_NEUTRAL_S( , , cpu->gprs[rd])*/)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(MUL, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*ARM_WAIT_MUL(cpu->gprs[rd]); cpu->gprs[rd] *= cpu->gprs[rn]; THUMB_NEUTRAL_S( , , cpu->gprs[rd]); currentCycles += cpu->memory.activeNonseqCycles16 - cpu->memory.activeSeqCycles16*/)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(BIC, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->gprs[rd] = cpu->gprs[rd] & ~cpu->gprs[rn]; THUMB_NEUTRAL_S( , , cpu->gprs[rd])*/)
DEFINE_DATA_FORM_5_INSTRUCTION_THUMB(MVN, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*->gprs[rd] = ~cpu->gprs[rn]; THUMB_NEUTRAL_S( , , cpu->gprs[rd])*/)

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
	interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->gprs[rd] += cpu->gprs[rm];
	if (rd == ARM_PC) {
		THUMB_WRITE_PC;
	}*/)

DEFINE_INSTRUCTION_WITH_HIGH_THUMB(CMP3, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*int32_t aluOut = cpu->gprs[rd] - cpu->gprs[rm]; THUMB_SUBTRACTION_S(cpu->gprs[rd], cpu->gprs[rm], aluOut)*/)
DEFINE_INSTRUCTION_WITH_HIGH_THUMB(MOV3,
	interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->gprs[rd] = cpu->gprs[rm];
	if (rd == ARM_PC) {
		THUMB_WRITE_PC;
	}*/)

#define DEFINE_IMMEDIATE_WITH_REGISTER_THUMB(NAME, BODY) \
	DEFINE_INSTRUCTION_THUMB(NAME, \
		int rd = (opcode >> 8) & 0x0007; \
		int immediate = (opcode & 0x00FF) << 2; \
		BODY;)

DEFINE_IMMEDIATE_WITH_REGISTER_THUMB(LDR3, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->gprs[rd] = cpu->memory.load32(cpu, (cpu->gprs[ARM_PC] & 0xFFFFFFFC) + immediate, &currentCycles); THUMB_LOAD_POST_BODY;*/)
DEFINE_IMMEDIATE_WITH_REGISTER_THUMB(LDR4, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->gprs[rd] = cpu->memory.load32(cpu, cpu->gprs[ARM_SP] + immediate, &currentCycles); THUMB_LOAD_POST_BODY;*/)
DEFINE_IMMEDIATE_WITH_REGISTER_THUMB(STR3, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->memory.store32(cpu, cpu->gprs[ARM_SP] + immediate, cpu->gprs[rd], &currentCycles); THUMB_STORE_POST_BODY;*/)

DEFINE_IMMEDIATE_WITH_REGISTER_THUMB(ADD5, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->gprs[rd] = (cpu->gprs[ARM_PC] & 0xFFFFFFFC) + immediate*/)
DEFINE_IMMEDIATE_WITH_REGISTER_THUMB(ADD6, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->gprs[rd] = cpu->gprs[ARM_SP] + immediate*/)

#define DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(NAME, BODY) \
	DEFINE_INSTRUCTION_THUMB(NAME, \
		int rm = (opcode >> 6) & 0x0007; \
		int rd = opcode & 0x0007; \
		int rn = (opcode >> 3) & 0x0007; \
		BODY;)

DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(LDR2, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->gprs[rd] = cpu->memory.load32(cpu, cpu->gprs[rn] + cpu->gprs[rm], &currentCycles); THUMB_LOAD_POST_BODY;*/)
DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(LDRB2,interpretInstruction(ctx, opcode); continue_compilation = false;
	/* cpu->gprs[rd] = cpu->memory.load8(cpu, cpu->gprs[rn] + cpu->gprs[rm], &currentCycles); THUMB_LOAD_POST_BODY;*/)
DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(LDRH2,interpretInstruction(ctx, opcode); continue_compilation = false;
	/* cpu->gprs[rd] = cpu->memory.load16(cpu, cpu->gprs[rn] + cpu->gprs[rm], &currentCycles); THUMB_LOAD_POST_BODY;*/)
DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(LDRSB,interpretInstruction(ctx, opcode); continue_compilation = false;
	/* cpu->gprs[rd] = ARM_SXT_8(cpu->memory.load8(cpu, cpu->gprs[rn] + cpu->gprs[rm], &currentCycles)); THUMB_LOAD_POST_BODY;*/)
DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(LDRSH,interpretInstruction(ctx, opcode); continue_compilation = false;
	/* rm = cpu->gprs[rn] + cpu->gprs[rm]; cpu->gprs[rd] = rm & 1 ? ARM_SXT_8(cpu->memory.load16(cpu, rm, &currentCycles)) : ARM_SXT_16(cpu->memory.load16(cpu, rm, &currentCycles)); THUMB_LOAD_POST_BODY;*/)
DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(STR2, interpretInstruction(ctx, opcode); continue_compilation = false;
	/*cpu->memory.store32(cpu, cpu->gprs[rn] + cpu->gprs[rm], cpu->gprs[rd], &currentCycles); THUMB_STORE_POST_BODY;*/)
DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(STRB2,interpretInstruction(ctx, opcode); continue_compilation = false;
	/* cpu->memory.store8(cpu, cpu->gprs[rn] + cpu->gprs[rm], cpu->gprs[rd], &currentCycles); THUMB_STORE_POST_BODY;*/)
DEFINE_LOAD_STORE_WITH_REGISTER_THUMB(STRH2,interpretInstruction(ctx, opcode); continue_compilation = false;
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
	interpretInstruction(ctx, opcode); continue_compilation = false;
	/*THUMB_LOAD_POST_BODY;
	if (!((1 << rn) & rs)) {
		cpu->gprs[rn] = address;
	}*/)

DEFINE_LOAD_STORE_MULTIPLE_THUMB(STMIA,
	(opcode >> 8) & 0x0007,
	store,
	IA,
	,
	interpretInstruction(ctx, opcode); continue_compilation = false;
	/*THUMB_STORE_POST_BODY;
	cpu->gprs[rn] = address;*/)

#define DEFINE_CONDITIONAL_BRANCH_THUMB(COND) \
	DEFINE_INSTRUCTION_THUMB(B ## COND, \
		interpretInstruction(ctx, opcode); continue_compilation = false; \
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

DEFINE_INSTRUCTION_THUMB(ADD7, interpretInstruction(ctx, opcode); continue_compilation = false; \
		/*cpu->gprs[ARM_SP] += (opcode & 0x7F) << 2*/)
DEFINE_INSTRUCTION_THUMB(SUB4, interpretInstruction(ctx, opcode); continue_compilation = false; \
		/*cpu->gprs[ARM_SP] -= (opcode & 0x7F) << 2*/)

DEFINE_LOAD_STORE_MULTIPLE_THUMB(POP,
	ARM_SP,
	load,
	IA,
	,
	interpretInstruction(ctx, opcode); continue_compilation = false; \
		/*THUMB_LOAD_POST_BODY;
	cpu->gprs[ARM_SP] = address*/)

DEFINE_LOAD_STORE_MULTIPLE_THUMB(POPR,
	ARM_SP,
	load,
	IA,
	/*rs |= 1 << ARM_PC*/,
	interpretInstruction(ctx, opcode); continue_compilation = false; \
		/*THUMB_LOAD_POST_BODY;
	cpu->gprs[ARM_SP] = address;
	THUMB_WRITE_PC;*/)

DEFINE_LOAD_STORE_MULTIPLE_THUMB(PUSH,
	ARM_SP,
	store,
	DB,
	,
	interpretInstruction(ctx, opcode); continue_compilation = false; \
		/*THUMB_STORE_POST_BODY;
	cpu->gprs[ARM_SP] = address*/)

DEFINE_LOAD_STORE_MULTIPLE_THUMB(PUSHR,
	ARM_SP,
	store,
	DB,
	/*rs |= 1 << ARM_LR*/,
	interpretInstruction(ctx, opcode); continue_compilation = false; \
		/*THUMB_STORE_POST_BODY;
	cpu->gprs[ARM_SP] = address*/)

DEFINE_INSTRUCTION_THUMB(ILL, interpretInstruction(ctx, opcode); continue_compilation = false; \
		/*ARM_ILL*/)
DEFINE_INSTRUCTION_THUMB(BKPT, interpretInstruction(ctx, opcode); continue_compilation = false; \
		/*cpu->irqh.bkpt16(cpu, opcode & 0xFF);*/)
DEFINE_INSTRUCTION_THUMB(B,
	interpretInstruction(ctx, opcode); continue_compilation = false; \
		/*int16_t immediate = (opcode & 0x07FF) << 5;
	cpu->gprs[ARM_PC] += (((int32_t) immediate) >> 4);
	THUMB_WRITE_PC;*/)

DEFINE_INSTRUCTION_THUMB(BL1,
	interpretInstruction(ctx, opcode); continue_compilation = false; \
		/*int16_t immediate = (opcode & 0x07FF) << 5;
	cpu->gprs[ARM_LR] = cpu->gprs[ARM_PC] + (((int32_t) immediate) << 7);*/)

DEFINE_INSTRUCTION_THUMB(BL2,
	interpretInstruction(ctx, opcode); continue_compilation = false; \
		/*uint16_t immediate = (opcode & 0x07FF) << 1;
	uint32_t pc = cpu->gprs[ARM_PC];
	cpu->gprs[ARM_PC] = cpu->gprs[ARM_LR] + immediate;
	cpu->gprs[ARM_LR] = pc - 1;
	THUMB_WRITE_PC;*/)

DEFINE_INSTRUCTION_THUMB(BX,
	interpretInstruction(ctx, opcode); continue_compilation = false; \
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

DEFINE_INSTRUCTION_THUMB(SWI, interpretInstruction(ctx, opcode); continue_compilation = false; \
		/*cpu->irqh.swi16(cpu, opcode & 0xFF)*/)

const ThumbCompiler _thumbCompilerTable[0x400] = {
		DECLARE_THUMB_EMITTER_BLOCK(_ThumbCompiler)
};
