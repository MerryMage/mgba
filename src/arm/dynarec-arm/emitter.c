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

#define OP_I       0x02000000
#define OP_S       0x00100000
#define ADDR1_ASRI 0x00000040
#define ADDR1_LSLI 0x00000000
#define ADDR1_LSRI 0x00000020
#define ADDR1_RORI 0x00000060

#define OP_ADC     0x00A00000
#define OP_ADD     0x00800000
#define OP_AND     0x00000000
#define OP_BIC     0x01C00000
#define OP_CMN     0x01700000
#define OP_CMP     0x01500000
#define OP_EOR     0x00200000
#define OP_MOV     0x01A00000
#define OP_MOVT    0x03400000
#define OP_MOVW    0x03000000
#define OP_MVN     0x01E00000
#define OP_ORR     0x01800000
#define OP_RSB     0x00600000
#define OP_RSC     0x00E00000
#define OP_SBC     0x00C00000
#define OP_SUB     0x00400000
#define OP_TEQ     0x01300000
#define OP_TST     0x01100000

#define OP_LDMIA   0x08900000
#define OP_LDRI    0x05100000
#define OP_POP     0x08BD0000
#define OP_PUSH    0x092D0000
#define OP_STMIA   0x08800000
#define OP_STRI    0x05000000
#define OP_STRBI   0x05400000

#define OP_B       0x0A000000
#define OP_BL      0x0B000000

#define OP_MRS   0x010F0000
#define OP_MSR   0x0120F000

static uint32_t calculateAddrMode1(unsigned imm) {
	if (imm < 0x100) {
		return imm;
	}
	int i;
	for (i = 0; i < 16; ++i) {
		unsigned t = ROR(imm, i * 2);
		if (t < 0x100) {
			return t | ((16 - i) << 8);
		}
	}
	abort();
}

#define DEFINE_ALU3_EMITTER(MN) \
	uint32_t emit##MN(unsigned dst, unsigned src, unsigned op2) { \
		return OP_##MN | (dst << 12) | (src << 16) | op2; \
	} \
	uint32_t emit##MN##I(unsigned dst, unsigned src, unsigned imm) { \
		return OP_##MN | OP_I | calculateAddrMode1(imm) | (dst << 12) | (src << 16); \
	} \
	uint32_t emit##MN##S(unsigned dst, unsigned src, unsigned op2) { \
		return OP_##MN | OP_S | (dst << 12) | (src << 16) | op2; \
	} \
	uint32_t emit##MN##SI(unsigned dst, unsigned src, unsigned imm) { \
		return OP_##MN | OP_S | OP_I | calculateAddrMode1(imm) | (dst << 12) | (src << 16); \
	} \
	uint32_t emit##MN##_ASRI(unsigned dst, unsigned src1, unsigned src2, unsigned imm) { \
		return OP_##MN | ADDR1_ASRI | (dst << 12) | (src1 << 16) | ((imm & 0x1F) << 7) | src2; \
	} \
	uint32_t emit##MN##_LSLI(unsigned dst, unsigned src1, unsigned src2, unsigned imm) { \
		return OP_##MN | ADDR1_LSLI | (dst << 12) | (src1 << 16) | ((imm & 0x1F) << 7) | src2; \
	} \
	uint32_t emit##MN##_LSRI(unsigned dst, unsigned src1, unsigned src2, unsigned imm) { \
		return OP_##MN | ADDR1_LSRI | (dst << 12) | (src1 << 16) | ((imm & 0x1F) << 7) | src2; \
	} \
	uint32_t emit##MN##_RORI(unsigned dst, unsigned src1, unsigned src2, unsigned imm) { \
		return OP_##MN | ADDR1_RORI | (dst << 12) | (src1 << 16) | ((imm & 0x1F) << 7) | src2; \
	}

#define DEFINE_ALU2_EMITTER(MN) \
	uint32_t emit##MN(unsigned dst, unsigned op2) { \
		return OP_##MN | (dst << 12) | op2; \
	} \
	uint32_t emit##MN##I(unsigned dst, unsigned imm) { \
		return OP_##MN | OP_I | calculateAddrMode1(imm) | (dst << 12); \
	} \
	uint32_t emit##MN##S(unsigned dst, unsigned op2) { \
		return OP_##MN | OP_S | (dst << 12) | op2; \
	} \
	uint32_t emit##MN##SI(unsigned dst, unsigned imm) { \
		return OP_##MN | OP_S | OP_I | calculateAddrMode1(imm) | (dst << 12); \
	} \
	uint32_t emit##MN##_ASRI(unsigned dst, unsigned src, unsigned imm) { \
		return OP_##MN | ADDR1_ASRI | (dst << 12) | ((imm & 0x1F) << 7) | src; \
	} \
	uint32_t emit##MN##_LSLI(unsigned dst, unsigned src, unsigned imm) { \
		return OP_##MN | ADDR1_LSLI | (dst << 12) | ((imm & 0x1F) << 7) | src; \
	} \
	uint32_t emit##MN##_LSRI(unsigned dst, unsigned src, unsigned imm) { \
		return OP_##MN | ADDR1_LSRI | (dst << 12) | ((imm & 0x1F) << 7) | src; \
	} \
	uint32_t emit##MN##_RORI(unsigned dst, unsigned src, unsigned imm) { \
		return OP_##MN | ADDR1_RORI | (dst << 12) | ((imm & 0x1F) << 7) | src; \
	}

#define DEFINE_ALU1_EMITTER(MN) \
	uint32_t emit##MN(unsigned src, unsigned op2) { \
		return OP_##MN | (src << 16) | op2; \
	} \
	uint32_t emit##MN##I(unsigned src, unsigned imm) { \
		return OP_##MN | OP_I | calculateAddrMode1(imm) | (src << 16); \
	} \
	uint32_t emit##MN##_ASRI(unsigned src1, unsigned src2, unsigned imm) { \
		return OP_##MN | ADDR1_ASRI | (src1 << 16) | ((imm & 0x1F) << 7) | src2; \
	} \
	uint32_t emit##MN##_LSLI(unsigned src1, unsigned src2, unsigned imm) { \
		return OP_##MN | ADDR1_LSLI | (src1 << 16) | ((imm & 0x1F) << 7) | src2; \
	} \
	uint32_t emit##MN##_LSRI(unsigned src1, unsigned src2, unsigned imm) { \
		return OP_##MN | ADDR1_LSRI | (src1 << 16) | ((imm & 0x1F) << 7) | src2; \
	} \
	uint32_t emit##MN##_RORI(unsigned src1, unsigned src2, unsigned imm) { \
		return OP_##MN | ADDR1_RORI | (src1 << 16) | ((imm & 0x1F) << 7) | src2; \
	}

DEFINE_ALU3_EMITTER(ADC)
DEFINE_ALU3_EMITTER(ADD)
DEFINE_ALU3_EMITTER(AND)
DEFINE_ALU3_EMITTER(BIC)
DEFINE_ALU1_EMITTER(CMN)
DEFINE_ALU1_EMITTER(CMP)
DEFINE_ALU3_EMITTER(EOR)
DEFINE_ALU2_EMITTER(MOV)
DEFINE_ALU2_EMITTER(MVN)
DEFINE_ALU3_EMITTER(ORR)
DEFINE_ALU3_EMITTER(RSB)
DEFINE_ALU3_EMITTER(RSC)
DEFINE_ALU3_EMITTER(SBC)
DEFINE_ALU3_EMITTER(SUB)
DEFINE_ALU1_EMITTER(TEQ)
DEFINE_ALU1_EMITTER(TST)

#undef DEFINE_ALU3_EMITTER
#undef DEFINE_ALU2_EMITTER
#undef DEFINE_ALU1_EMITTER

uint32_t emitMOVT(unsigned dst, uint16_t value) {
	return OP_MOVT | (dst << 12) | ((value & 0xF000) << 4) | (value & 0x0FFF);
}

uint32_t emitMOVW(unsigned dst, uint16_t value) {
	return OP_MOVW | (dst << 12) | ((value & 0xF000) << 4) | (value & 0x0FFF);
}

uint32_t emitLDMIA(unsigned base, unsigned mask) {
	return OP_LDMIA | (base << 16) | mask;
}

uint32_t emitLDRI(unsigned reg, unsigned base, int offset) {
	uint32_t op = OP_LDRI | (base << 16) | (reg << 12);
	if (offset > 0) {
		op |= 0x00800000 | offset;
	} else {
		op |= -offset & 0xFFF;
	}
	return op;
}

uint32_t emitPOP(unsigned mask) {
	return OP_POP | mask;
}

uint32_t emitPUSH(unsigned mask) {
	return OP_PUSH | mask;
}

uint32_t emitSTMIA(unsigned base, unsigned mask) {
	return OP_STMIA | (base << 16) | mask;
}

uint32_t emitSTRI(unsigned reg, unsigned base, int offset) {
	uint32_t op = OP_STRI | (base << 16) | (reg << 12);
	if (offset > 0) {
		op |= 0x00800000 | offset;
	} else {
		op |= -offset & 0xFFF;
	}
	return op;
}

uint32_t emitSTRBI(unsigned reg, unsigned base, int offset) {
	uint32_t op = OP_STRBI | (base << 16) | (reg << 12);
	if (offset > 0) {
		op |= 0x00800000 | offset;
	} else {
		op |= -offset & 0xFFF;
	}
	return op;
}

uint32_t emitB(void* base, void* target) {
	uint32_t diff = (intptr_t) target - (intptr_t) base - WORD_SIZE_ARM * 2;
	diff >>= 2;
	diff &= 0x00FFFFFF;
	return OP_B | diff;
}

uint32_t emitBL(void* base, void* target) {
	uint32_t diff = (intptr_t) target - (intptr_t) base - WORD_SIZE_ARM * 2;
	diff >>= 2;
	diff &= 0x00FFFFFF;
	return OP_BL | diff;
}

uint32_t emitMRS(unsigned dst) {
	return OP_MRS | (dst << 12);
}

uint32_t emitMSR(bool nzcvq, bool g, unsigned src) {
    return OP_MSR | (nzcvq << 19) | (g << 18) | src;
}

void updatePC(struct ARMDynarecContext* ctx, uint32_t address) {
	assert(!ctx->scratch0_in_use);
	EMIT_IMM(ctx, AL, REG_SCRATCH0, address);
	EMIT(ctx, STRI, AL, REG_SCRATCH0, REG_ARMCore, ARM_PC * sizeof(uint32_t));
}

void updateEvents(struct ARMDynarecContext* ctx, struct ARMCore* cpu, uint32_t expected_pc) {
	assert(!ctx->scratch0_in_use && !ctx->scratch1_in_use);
	flushNZCV(ctx);
	EMIT(ctx, ADDI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, cycles));
	EMIT(ctx, LDMIA, AL, REG_SCRATCH0, (1 << REG_SCRATCH0) | (1 << REG_SCRATCH1));
	EMIT(ctx, CMP, AL, REG_SCRATCH1, REG_SCRATCH0); // cpu->nextEvent - cpu->cycles
	EMIT(ctx, STRI, AL, REG_GUEST_SP, REG_ARMCore, ARM_SP * sizeof(uint32_t));
	EMIT(ctx, STMIA, AL, REG_ARMCore, REGLIST_GUESTREGS);
	EMIT(ctx, PUSH, AL, REGLIST_SAVE);
	EMIT(ctx, BL, LE, ctx->code, cpu->irqh.processEvents);
	EMIT(ctx, POP, AL, REGLIST_SAVE);
	EMIT(ctx, LDMIA, AL, REG_ARMCore, REGLIST_GUESTREGS);
	EMIT(ctx, LDRI, AL, REG_GUEST_SP, REG_ARMCore, ARM_SP * sizeof(uint32_t));
	EMIT(ctx, LDRI, AL, REG_SCRATCH0, REG_ARMCore, ARM_PC * sizeof(uint32_t));
	EMIT_IMM(ctx, AL, REG_SCRATCH1, expected_pc);
	EMIT(ctx, CMP, AL, REG_SCRATCH0, REG_SCRATCH1);
	EMIT(ctx, POP, NE, REGLIST_RETURN);
	loadNZCV(ctx);
}

void flushPrefetch(struct ARMDynarecContext* ctx, uint32_t op0, uint32_t op1) {
	assert(!ctx->scratch0_in_use);
	EMIT_IMM(ctx, AL, REG_SCRATCH0, op0);
	EMIT(ctx, STRI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, prefetch) + 0 * sizeof(uint32_t));
	EMIT_IMM(ctx, AL, REG_SCRATCH0, op1);
	EMIT(ctx, STRI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, prefetch) + 1 * sizeof(uint32_t));
}

unsigned loadReg(struct ARMDynarecContext* ctx, unsigned emureg) {
	switch (emureg) {
	case 0:
		return REG_GUEST_R0;
	case 1:
		return REG_GUEST_R1;
	case 2:
		return REG_GUEST_R2;
	case 3:
		return REG_GUEST_R3;
	case 4:
		return REG_GUEST_R4;
	case 5:
		return REG_GUEST_R5;
	case 6:
		return REG_GUEST_R6;
	case 7:
		return REG_GUEST_R7;
	case 13:
		return REG_GUEST_SP;
	}

	unsigned sysreg;
	if (!ctx->scratch0_in_use) {
		ctx->scratch0_in_use = true;
		sysreg = REG_SCRATCH0;
	} else if (!ctx->scratch1_in_use) {
		ctx->scratch1_in_use = true;
		sysreg = REG_SCRATCH1;
	} else {
		assert(!"unreachable");
	}
	if (emureg != 15) {
		EMIT(ctx, LDRI, AL, sysreg, REG_ARMCore, emureg * sizeof(uint32_t));
	} else {
		EMIT_IMM(ctx, AL, sysreg, ctx->address + WORD_SIZE_THUMB * 2);
	}
	return sysreg;
}

void flushReg(struct ARMDynarecContext* ctx, unsigned emureg, unsigned sysreg) {
	switch (emureg) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 13:
		return;
	}

	switch (sysreg) {
	case REG_SCRATCH0:
		assert(ctx->scratch0_in_use);
		ctx->scratch0_in_use = false;
		break;
	case REG_SCRATCH1:
		assert(ctx->scratch1_in_use);
		ctx->scratch1_in_use = false;
		break;
	default:
		assert(!"unreachable");
	}
	EMIT(ctx, STRI, AL, sysreg, REG_ARMCore, emureg * sizeof(uint32_t));
}

void scratchesNotInUse(struct ARMDynarecContext* ctx) {
	ctx->scratch0_in_use = false;
	ctx->scratch1_in_use = false;
}

void loadNZCV(struct ARMDynarecContext* ctx) {
	assert(!ctx->scratch0_in_use);
	EMIT(ctx, LDRI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, cpsr));
	EMIT(ctx, MSR, AL, true, false, REG_SCRATCH0);
}

void flushNZCV(struct ARMDynarecContext* ctx) {
	assert(!ctx->scratch0_in_use);
	EMIT(ctx, MRS, AL, REG_SCRATCH0);
	EMIT(ctx, MOV_LSRI, AL, REG_SCRATCH0, REG_SCRATCH0, 24);
	EMIT(ctx, STRBI, AL, REG_SCRATCH0, REG_ARMCore, 16 * sizeof(uint32_t) + 3);
}

void flushCycles(struct ARMDynarecContext* ctx) {
	assert(!ctx->scratch0_in_use);
	if (ctx->cycles == 0) {
		return;
	}
	EMIT(ctx, LDRI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, cycles));
	EMIT(ctx, ADDI, AL, REG_SCRATCH0, REG_SCRATCH0, ctx->cycles);
	EMIT(ctx, STRI, AL, REG_SCRATCH0, REG_ARMCore, offsetof(struct ARMCore, cycles));
	ctx->cycles = 0;
}
