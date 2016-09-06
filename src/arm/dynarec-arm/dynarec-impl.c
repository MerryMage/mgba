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

static bool needsUpdatePrefetch(struct ARMInstructionInfo* info) {
	if ((info->operandFormat & (ARM_OPERAND_MEMORY_1 | ARM_OPERAND_AFFECTED_1)) == ARM_OPERAND_MEMORY_1) {
		return true;
	}
	if ((info->operandFormat & (ARM_OPERAND_MEMORY_2 | ARM_OPERAND_AFFECTED_2)) == ARM_OPERAND_MEMORY_2) {
		return true;
	}
	if ((info->operandFormat & (ARM_OPERAND_MEMORY_3 | ARM_OPERAND_AFFECTED_3)) == ARM_OPERAND_MEMORY_3) {
		return true;
	}
	if ((info->operandFormat & (ARM_OPERAND_MEMORY_4 | ARM_OPERAND_AFFECTED_4)) == ARM_OPERAND_MEMORY_4) {
		return true;
	}
	return false;
}

static bool needsUpdateEvents(struct ARMInstructionInfo* info) {
	if ((info->operandFormat & (ARM_OPERAND_MEMORY_1 | ARM_OPERAND_AFFECTED_1)) == (ARM_OPERAND_MEMORY_1 | ARM_OPERAND_AFFECTED_1)) {
		return true;
	}
	if ((info->operandFormat & (ARM_OPERAND_MEMORY_2 | ARM_OPERAND_AFFECTED_2)) == (ARM_OPERAND_MEMORY_2 | ARM_OPERAND_AFFECTED_2)) {
		return true;
	}
	if ((info->operandFormat & (ARM_OPERAND_MEMORY_3 | ARM_OPERAND_AFFECTED_3)) == (ARM_OPERAND_MEMORY_3 | ARM_OPERAND_AFFECTED_3)) {
		return true;
	}
	if ((info->operandFormat & (ARM_OPERAND_MEMORY_4 | ARM_OPERAND_AFFECTED_4)) == (ARM_OPERAND_MEMORY_4 | ARM_OPERAND_AFFECTED_4)) {
		return true;
	}
	if (info->branchType || info->traps) {
		return true;
	}
	return false;
}

static bool needsUpdatePC(struct ARMInstructionInfo* info) {
	if (needsUpdateEvents(info)) {
		return true;
	}
	if (info->operandFormat & ARM_OPERAND_REGISTER_1 && info->op1.reg == ARM_PC) {
		return true;
	}
	if (info->operandFormat & ARM_OPERAND_REGISTER_2 && info->op2.reg == ARM_PC) {
		return true;
	}
	if (info->operandFormat & ARM_OPERAND_REGISTER_3 && info->op3.reg == ARM_PC) {
		return true;
	}
	if (info->operandFormat & ARM_OPERAND_REGISTER_4 && info->op4.reg == ARM_PC) {
		return true;
	}
	if (info->operandFormat & ARM_OPERAND_MEMORY && info->memory.format & ARM_MEMORY_REGISTER_BASE && info->memory.baseReg == ARM_PC) {
		return true;
	}
	if (info->operandFormat & ARM_OPERAND_SHIFT_REGISTER_1 && info->op1.shifterReg == ARM_PC) {
		return true;
	}
	if (info->operandFormat & ARM_OPERAND_SHIFT_REGISTER_2 && info->op2.shifterReg == ARM_PC) {
		return true;
	}
	if (info->operandFormat & ARM_OPERAND_SHIFT_REGISTER_3 && info->op3.shifterReg == ARM_PC) {
		return true;
	}
	if (info->operandFormat & ARM_OPERAND_SHIFT_REGISTER_4 && info->op4.shifterReg == ARM_PC) {
		return true;
	}
	return false;
}

#define ADD_CYCLES \
	ctx.cycles += 1 + info.iCycles; \
	ctx.cycles += info.sInstructionCycles * cpu->memory.activeSeqCycles16; \
	ctx.cycles += info.nInstructionCycles * cpu->memory.activeNonseqCycles16;

#define RECOMPILE_ALU(MN) \
	do { \
		unsigned rd = loadReg(&ctx, info.op1.reg); \
		switch (info.operandFormat & (ARM_OPERAND_2 | ARM_OPERAND_3 | ARM_OPERAND_4)) { \
		case ARM_OPERAND_REGISTER_2 | ARM_OPERAND_REGISTER_3: { \
			unsigned rn = loadReg(&ctx, info.op2.reg); \
			unsigned rm = loadReg(&ctx, info.op3.reg); \
			if (info.affectsCPSR) \
				EMIT(&ctx, MN##S, AL, rd, rn, rm); \
			else \
				EMIT(&ctx, MN, AL, rd, rn, rm); \
			break; \
		} \
		case ARM_OPERAND_REGISTER_2 | ARM_OPERAND_IMMEDIATE_3: { \
			unsigned rn = loadReg(&ctx, info.op2.reg); \
			if (info.affectsCPSR) \
				EMIT(&ctx, MN##SI, AL, rd, rn, info.op3.immediate); \
			else \
				EMIT(&ctx, MN##I, AL, rd, rn, info.op3.immediate); \
			break; \
		} \
		case ARM_OPERAND_REGISTER_2: { \
			unsigned rm = loadReg(&ctx, info.op2.reg); \
			if (info.affectsCPSR) \
				EMIT(&ctx, MN##S, AL, rd, rd, rm); \
			else \
				EMIT(&ctx, MN, AL, rd, rd, rm); \
			break; \
		} \
		case ARM_OPERAND_IMMEDIATE_2: { \
			if (info.affectsCPSR) \
				EMIT(&ctx, MN##SI, AL, rd, rd, info.op2.immediate); \
			else \
				EMIT(&ctx, MN##I, AL, rd, rd, info.op2.immediate); \
			break; \
		} \
		default: \
			abort(); \
		} \
		if (info.operandFormat & ARM_OPERAND_AFFECTED_1) { \
			flushReg(&ctx, info.op1.reg, rdn); \
		} \
		scratchesNotInUse(&ctx); \
		ADD_CYCLES \
	} while (0)

void ARMDynarecEmitPrelude(struct ARMCore* cpu) {
	code_t* code = (code_t*) cpu->dynarec.buffer;
	cpu->dynarec.execute = (void (*)(struct ARMCore*, void*)) code;

	// Common prologue
	EMIT_L(code, PUSH, AL, 0x4DF0);
	EMIT_L(code, LDMIA, AL, 0, REGLIST_GUESTREGS);
	EMIT_L(code, LDRI, AL, REG_GUEST_SP, 0, ARM_SP * sizeof(uint32_t));
	EMIT_L(code, LDRI, AL, 2, REG_ARMCore, offsetof(struct ARMCore, cpsr));
	EMIT_L(code, MSR, AL, true, false, 2);
	EMIT_L(code, PUSH, AL, REGLIST_RETURN);
	EMIT_L(code, MOV, AL, 15, 1);

	// Common epilogue
	EMIT_L(code, STRI, AL, REG_GUEST_SP, 0, ARM_SP * sizeof(uint32_t));
	EMIT_L(code, STMIA, AL, 0, REGLIST_GUESTREGS);
	EMIT_L(code, POP, AL, 0x8DF0);

	cpu->dynarec.buffer = code;
	__clear_cache(cpu->dynarec.execute, code);
}

void ARMDynarecRecompileTrace(struct ARMCore* cpu, struct ARMDynarecTrace* trace) {
#ifndef NDEBUG
	printf("%08X (%c)\n", trace->start, trace->mode == MODE_THUMB ? 'T' : 'A');
	printf("%u\n", cpu->nextEvent - cpu->cycles);
#endif
	struct ARMDynarecContext ctx = {
		.code = cpu->dynarec.buffer,
		.address = trace->start,
		.cycles = 0,
		.scratch0_in_use = false,
		.scratch1_in_use = false,
	};
	if (trace->mode == MODE_ARM) {
		return;
	} else {
		trace->entry = (void*) ctx.code;
		__attribute__((aligned(64))) struct ARMInstructionInfo info;
		while (true) {
			uint16_t instruction = cpu->memory.load16(cpu, ctx.address, 0);
			ARMDecodeThumb(instruction, &info);
			ctx.address += WORD_SIZE_THUMB;
//			if (needsUpdatePC(&info)) {
				updatePC(&ctx, ctx.address + WORD_SIZE_THUMB);
//			}
//			if (needsUpdatePrefetch(&info)) {
				flushPrefetch(&ctx, cpu->memory.load16(cpu, ctx.address, 0), cpu->memory.load16(cpu, ctx.address + WORD_SIZE_THUMB, 0));
				flushCycles(&ctx);
//			}

			switch (info.mnemonic) {
			case ARM_MN_ADC:
				RECOMPILE_ALU(ADC);
				break;
			case ARM_MN_ADD:
				RECOMPILE_ALU(ADD);
				break;
			case ARM_MN_ASR:
				goto interpret;
			case ARM_MN_B:
				goto interpret;
			case ARM_MN_BIC:
				RECOMPILE_ALU(BIC);
				break;
			case ARM_MN_BKPT:
				goto interpret;
			case ARM_MN_BL:
				goto interpret;
			case ARM_MN_BX:
				goto interpret;
			case ARM_MN_CMN:
				goto interpret;
			case ARM_MN_CMP:
				RECOMPILE_ALU(CMP);
				break;
			case ARM_MN_EOR:
				RECOMPILE_ALU(EOR);
				break;
			case ARM_MN_LDM:
				goto interpret;
			case ARM_MN_LDR:
				goto interpret;
			case ARM_MN_LSL:
				goto interpret;
			case ARM_MN_LSR:
				goto interpret;
			case ARM_MN_MLA:
				goto interpret;
			case ARM_MN_MOV:
				RECOMPILE_ALU(MOV);
				break;
			case ARM_MN_MRS:
			case ARM_MN_MSR:
			case ARM_MN_MUL:
			case ARM_MN_MVN:
				RECOMPILE_ALU(MVN);
				break;
			case ARM_MN_NEG:
				goto interpret;
			case ARM_MN_ORR:
				RECOMPILE_ALU(ORR);
				break;
			case ARM_MN_ROR:
				goto interpret;
			case ARM_MN_RSB:
				RECOMPILE_ALU(RSB);
				break;
			case ARM_MN_RSC:
				RECOMPILE_ALU(RSC);
				break;
			case ARM_MN_SBC:
				RECOMPILE_ALU(SBC);
				break;
			case ARM_MN_STM:
				goto interpret;
			case ARM_MN_STR:
				goto interpret;
			case ARM_MN_SUB:
				RECOMPILE_ALU(SUB);
				break;
			case ARM_MN_SWI:
				goto interpret;
			case ARM_MN_TEQ:
				RECOMPILE_ALU(TEQ);
				break;
			case ARM_MN_TST:
				RECOMPILE_ALU(TST);
				break;
			interpret:
			default:
				flushNZCV(&ctx);
				EMIT(&ctx, STRI, AL, REG_GUEST_SP, 0, ARM_SP * sizeof(uint32_t));
				EMIT(&ctx, STMIA, AL, 0, REGLIST_GUESTREGS);
				EMIT(&ctx, PUSH, AL, REGLIST_SAVE);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
				EMIT_IMM(&ctx, AL, 1, instruction);
#pragma GCC diagnostic pop
				EMIT(&ctx, BL, AL, ctx.code, _thumbTable[instruction >> 6]);
				EMIT(&ctx, POP, AL, REGLIST_SAVE);
				EMIT(&ctx, LDMIA, AL, 0, REGLIST_GUESTREGS);
				EMIT(&ctx, LDRI, AL, REG_GUEST_SP, 0, ARM_SP * sizeof(uint32_t));
				loadNZCV(&ctx);
				break;
			}
//			if (needsUpdateEvents(&info)) {
//				flushCycles(&ctx);
				updateEvents(&ctx, cpu, ctx.address + WORD_SIZE_THUMB);
//			}
			if (info.branchType >= ARM_BRANCH || info.traps) {
				break;
			}
		}
		flushPrefetch(&ctx, cpu->memory.load16(cpu, ctx.address, 0), cpu->memory.load16(cpu, ctx.address + WORD_SIZE_THUMB, 0));
		flushCycles(&ctx);
		flushNZCV(&ctx);
		EMIT(&ctx, POP, AL, REGLIST_RETURN);
	}
	__clear_cache(trace->entry, ctx.code);
	cpu->dynarec.buffer = ctx.code;
}
