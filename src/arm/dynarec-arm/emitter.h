/* Copyright (c) 2013-2016 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef ARM_DYNAREC_EMITTER_H
#define ARM_DYNAREC_EMITTER_H

#include "util/common.h"

#define REG_ARMCore  0
#define REG_SCRATCH0 1
#define REG_SCRATCH1 2
#define REG_GUEST_R0 4
#define REG_GUEST_R1 5
#define REG_GUEST_R2 6
#define REG_GUEST_R3 7
#define REG_GUEST_R4 8
#define REG_GUEST_R5 10
#define REG_GUEST_R6 11
#define REG_GUEST_R7 12
#define REG_GUEST_SP 3
#define REGLIST_SAVE 0x500F
#define REGLIST_RETURN 0x8001
#define REGLIST_GUESTREGS 0x1DF0 /*except SP*/

#define COND_EQ 0x00000000
#define COND_NE 0x10000000
#define COND_MI 0x40000000
#define COND_LE 0xD0000000
#define COND_AL 0xE0000000

typedef uint32_t code_t;

struct ARMDynarecContext {
	code_t* code;
	uint32_t address;
	int32_t cycles;
	bool scratch0_in_use;
	bool scratch1_in_use;
};

#define EMIT_L(DEST, OPCODE, COND, ...) \
        do { \
		*DEST = emit ## OPCODE (__VA_ARGS__) | COND_ ## COND; \
		++DEST; \
	} while (0)

#define EMIT(CTX, OPCODE, COND, ...) EMIT_L((CTX)->code, OPCODE, COND, __VA_ARGS__)

#define EMIT_IMM(DEST, COND, REG, VALUE) \
	do { \
		EMIT(DEST, MOVW, COND, REG, VALUE); \
		if (VALUE >= 0x10000) { \
			EMIT(DEST, MOVT, COND, REG, (VALUE) >> 16); \
		} \
	} while (0)

uint32_t calculateAddrMode1(unsigned imm);

uint32_t emitADCS(unsigned dst, unsigned src, unsigned op2);
uint32_t emitADD(unsigned dst, unsigned src, unsigned op2);
uint32_t emitADDI(unsigned dst, unsigned src, unsigned imm);
uint32_t emitADDS(unsigned dst, unsigned src, unsigned op2);
uint32_t emitADDSI(unsigned dst, unsigned src, unsigned imm);
uint32_t emitANDI(unsigned dst, unsigned src, unsigned imm);
uint32_t emitANDS(unsigned dst, unsigned src, unsigned op2);
uint32_t emitANDSI(unsigned dst, unsigned src, unsigned imm);
uint32_t emitB(void* base, void* target);
uint32_t emitBICI(unsigned dst, unsigned src, unsigned imm);
uint32_t emitBICS(unsigned dst, unsigned src, unsigned op2);
uint32_t emitBICSI(unsigned dst, unsigned src, unsigned imm);
uint32_t emitBL(void* base, void* target);
uint32_t emitCMP(unsigned src1, unsigned src2);
uint32_t emitEORI(unsigned dst, unsigned src, unsigned imm);
uint32_t emitEORS(unsigned dst, unsigned src, unsigned op2);
uint32_t emitEORSI(unsigned dst, unsigned src, unsigned imm);
uint32_t emitLDMIA(unsigned base, unsigned mask);
uint32_t emitLDRI(unsigned reg, unsigned base, int offset);
uint32_t emitMOV(unsigned dst, unsigned src);
uint32_t emitMOV_LSRI(unsigned dst, unsigned src, unsigned imm);
uint32_t emitMOVT(unsigned dst, uint16_t value);
uint32_t emitMOVW(unsigned dst, uint16_t value);
uint32_t emitMRS(unsigned dst);
uint32_t emitMSR(bool nzcvq, bool g, unsigned src);
uint32_t emitORRI(unsigned dst, unsigned src, unsigned imm);
uint32_t emitORRS(unsigned dst, unsigned src, unsigned op2);
uint32_t emitORRSI(unsigned dst, unsigned src, unsigned imm);
uint32_t emitPOP(unsigned mask);
uint32_t emitPUSH(unsigned mask);
uint32_t emitSTMIA(unsigned base, unsigned mask);
uint32_t emitSTRI(unsigned reg, unsigned base, int offset);
uint32_t emitSTRBI(unsigned reg, unsigned base, int offset);
uint32_t emitSUBI(unsigned dst, unsigned src1, unsigned src2);
uint32_t emitSUBS(unsigned dst, unsigned src1, unsigned src2);
uint32_t emitSUBSI(unsigned dst, unsigned src1, unsigned src2);

void updatePC(struct ARMDynarecContext* ctx, uint32_t address);
void updateEvents(struct ARMDynarecContext* ctx, struct ARMCore* cpu, uint32_t expected_pc);
void flushPrefetch(struct ARMDynarecContext* ctx, uint32_t op0, uint32_t op1);
unsigned loadReg(struct ARMDynarecContext* ctx, unsigned emureg);
void flushReg(struct ARMDynarecContext* ctx, unsigned emureg, unsigned sysreg);
void scratchesNotInUse(struct ARMDynarecContext* ctx);
void loadNZCV(struct ARMDynarecContext* ctx);
void flushNZCV(struct ARMDynarecContext* ctx);
void flushCycles(struct ARMDynarecContext* ctx);

#endif
