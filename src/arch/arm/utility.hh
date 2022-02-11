/*
 * Copyright (c) 2010, 2012-2013, 2016-2018 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 2007-2008 The Florida State University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Korey Sewell
 *          Stephen Hines
 */

#ifndef __ARCH_ARM_UTILITY_HH__
#define __ARCH_ARM_UTILITY_HH__

#include "arch/arm/isa_traits.hh"
#include "arch/arm/miscregs.hh"
#include "arch/arm/types.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "arch/arm/qarma.hh" //yh+
#include <stdio.h> //yh+
#include <stdlib.h> //yh+

class ArmSystem;

namespace ArmISA {

inline PCState
buildRetPC(const PCState &curPC, const PCState &callPC)
{
    PCState retPC = callPC;
    retPC.uEnd();
    return retPC;
}

inline bool
testPredicate(uint32_t nz, uint32_t c, uint32_t v, ConditionCode code)
{
    bool n = (nz & 0x2);
    bool z = (nz & 0x1);

    switch (code)
    {
        case COND_EQ: return  z;
        case COND_NE: return !z;
        case COND_CS: return  c;
        case COND_CC: return !c;
        case COND_MI: return  n;
        case COND_PL: return !n;
        case COND_VS: return  v;
        case COND_VC: return !v;
        case COND_HI: return  (c && !z);
        case COND_LS: return !(c && !z);
        case COND_GE: return !(n ^ v);
        case COND_LT: return  (n ^ v);
        case COND_GT: return !(n ^ v || z);
        case COND_LE: return  (n ^ v || z);
        case COND_AL: return true;
        case COND_UC: return true;
        default:
            panic("Unhandled predicate condition: %d\n", code);
    }
}

/**
 * Function to insure ISA semantics about 0 registers.
 * @param tc The thread context.
 */
template <class TC>
void zeroRegisters(TC *tc);

inline void startupCPU(ThreadContext *tc, int cpuId)
{
    tc->activate();
}

void copyRegs(ThreadContext *src, ThreadContext *dest);

static inline void
copyMiscRegs(ThreadContext *src, ThreadContext *dest)
{
    panic("Copy Misc. Regs Not Implemented Yet\n");
}

void initCPU(ThreadContext *tc, int cpuId);

static inline bool
inUserMode(CPSR cpsr)
{
    return cpsr.mode == MODE_USER || cpsr.mode == MODE_EL0T;
}

static inline bool
inUserMode(ThreadContext *tc)
{
    return inUserMode(tc->readMiscRegNoEffect(MISCREG_CPSR));
}

static inline bool
inPrivilegedMode(CPSR cpsr)
{
    return !inUserMode(cpsr);
}

static inline bool
inPrivilegedMode(ThreadContext *tc)
{
    return !inUserMode(tc);
}

bool inAArch64(ThreadContext *tc);

static inline OperatingMode
currOpMode(ThreadContext *tc)
{
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    return (OperatingMode) (uint8_t) cpsr.mode;
}

static inline ExceptionLevel
currEL(ThreadContext *tc)
{
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    return (ExceptionLevel) (uint8_t) cpsr.el;
}

/**
 * This function checks whether selected EL provided as an argument
 * is using the AArch32 ISA. This information might be unavailable
 * at the current EL status: it hence returns a pair of boolean values:
 * a first boolean, true if information is available (known),
 * and a second one, true if EL is using AArch32, false for AArch64.
 *
 * @param tc The thread context.
 * @param el The target exception level.
 * @retval known is FALSE for EL0 if the current Exception level
 *               is not EL0 and EL1 is using AArch64, since it cannot
 *               determine the state of EL0; TRUE otherwise.
 * @retval aarch32 is TRUE if the specified Exception level is using AArch32;
 *                 FALSE otherwise.
 */
std::pair<bool, bool>
ELUsingAArch32K(ThreadContext *tc, ExceptionLevel el);

bool ELIs32(ThreadContext *tc, ExceptionLevel el);

bool ELIs64(ThreadContext *tc, ExceptionLevel el);

bool isBigEndian64(ThreadContext *tc);

/**
 * badMode is checking if the execution mode provided as an argument is
 * valid and implemented for AArch32
 *
 * @param tc ThreadContext
 * @param mode OperatingMode to check
 * @return false if mode is valid and implemented, true otherwise
 */
bool badMode32(ThreadContext *tc, OperatingMode mode);

/**
 * badMode is checking if the execution mode provided as an argument is
 * valid and implemented.
 *
 * @param tc ThreadContext
 * @param mode OperatingMode to check
 * @return false if mode is valid and implemented, true otherwise
 */
bool badMode(ThreadContext *tc, OperatingMode mode);

static inline uint8_t
itState(CPSR psr)
{
    ITSTATE it = 0;
    it.top6 = psr.it2;
    it.bottom2 = psr.it1;

    return (uint8_t)it;
}

/**
 * Removes the tag from tagged addresses if that mode is enabled.
 * @param addr The address to be purified.
 * @param tc The thread context.
 * @param el The controlled exception level.
 * @return The purified address.
 */
Addr purifyTaggedAddr(Addr addr, ThreadContext *tc, ExceptionLevel el,
                      TTBCR tcr);
Addr purifyTaggedAddr(Addr addr, ThreadContext *tc, ExceptionLevel el);

static inline bool
inSecureState(SCR scr, CPSR cpsr)
{
    switch ((OperatingMode) (uint8_t) cpsr.mode) {
      case MODE_MON:
      case MODE_EL3T:
      case MODE_EL3H:
        return true;
      case MODE_HYP:
      case MODE_EL2T:
      case MODE_EL2H:
        return false;
      default:
        return !scr.ns;
    }
}

bool inSecureState(ThreadContext *tc);

/**
 * Return TRUE if an Exception level below EL3 is in Secure state.
 * Differs from inSecureState in that it ignores the current EL
 * or Mode in considering security state.
 */
inline bool isSecureBelowEL3(ThreadContext *tc);

bool longDescFormatInUse(ThreadContext *tc);

/** This helper function is either returing the value of
 * MPIDR_EL1 (by calling getMPIDR), or it is issuing a read
 * to VMPIDR_EL2 (as it happens in virtualized systems) */
RegVal readMPIDR(ArmSystem *arm_sys, ThreadContext *tc);

/** This helper function is returing the value of MPIDR_EL1 */
RegVal getMPIDR(ArmSystem *arm_sys, ThreadContext *tc);

static inline uint32_t
mcrMrcIssBuild(bool isRead, uint32_t crm, IntRegIndex rt, uint32_t crn,
               uint32_t opc1, uint32_t opc2)
{
    return (isRead <<  0) |
           (crm    <<  1) |
           (rt     <<  5) |
           (crn    << 10) |
           (opc1   << 14) |
           (opc2   << 17);
}

static inline void
mcrMrcIssExtract(uint32_t iss, bool &isRead, uint32_t &crm, IntRegIndex &rt,
                 uint32_t &crn, uint32_t &opc1, uint32_t &opc2)
{
    isRead = (iss >>  0) & 0x1;
    crm    = (iss >>  1) & 0xF;
    rt     = (IntRegIndex) ((iss >>  5) & 0xF);
    crn    = (iss >> 10) & 0xF;
    opc1   = (iss >> 14) & 0x7;
    opc2   = (iss >> 17) & 0x7;
}

static inline uint32_t
mcrrMrrcIssBuild(bool isRead, uint32_t crm, IntRegIndex rt, IntRegIndex rt2,
                 uint32_t opc1)
{
    return (isRead <<  0) |
           (crm    <<  1) |
           (rt     <<  5) |
           (rt2    << 10) |
           (opc1   << 16);
}

static inline uint32_t
msrMrs64IssBuild(bool isRead, uint32_t op0, uint32_t op1, uint32_t crn,
                 uint32_t crm, uint32_t op2, IntRegIndex rt)
{
    return isRead |
        (crm << 1) |
        (rt << 5) |
        (crn << 10) |
        (op1 << 14) |
        (op2 << 17) |
        (op0 << 20);
}

bool
mcrMrc15TrapToHyp(const MiscRegIndex miscReg, HCR hcr, CPSR cpsr, SCR scr,
                  HDCR hdcr, HSTR hstr, HCPTR hcptr, uint32_t iss);
bool
mcrMrc14TrapToHyp(const MiscRegIndex miscReg, HCR hcr, CPSR cpsr, SCR scr,
                  HDCR hdcr, HSTR hstr, HCPTR hcptr, uint32_t iss);
bool
mcrrMrrc15TrapToHyp(const MiscRegIndex miscReg, CPSR cpsr, SCR scr, HSTR hstr,
                    HCR hcr, uint32_t iss);

bool SPAlignmentCheckEnabled(ThreadContext* tc);

uint64_t getArgument(ThreadContext *tc, int &number, uint16_t size, bool fp);

void skipFunction(ThreadContext *tc);

inline void
advancePC(PCState &pc, const StaticInstPtr &inst)
{
    inst->advancePC(pc);
}

Addr truncPage(Addr addr);
Addr roundPage(Addr addr);

inline uint64_t
getExecutingAsid(ThreadContext *tc)
{
    return tc->readMiscReg(MISCREG_CONTEXTIDR);
}

// Decodes the register index to access based on the fields used in a MSR
// or MRS instruction
bool
decodeMrsMsrBankedReg(uint8_t sysM, bool r, bool &isIntReg, int &regIdx,
                      CPSR cpsr, SCR scr, NSACR nsacr,
                      bool checkSecurity = true);

// This wrapper function is used to turn the register index into a source
// parameter for the instruction. See Operands.isa
static inline int
decodeMrsMsrBankedIntRegIndex(uint8_t sysM, bool r)
{
    int  regIdx;
    bool isIntReg;
    bool validReg;

    validReg = decodeMrsMsrBankedReg(sysM, r, isIntReg, regIdx, 0, 0, 0, false);
    return (validReg && isIntReg) ? regIdx : INTREG_DUMMY;
}

/**
 * Returns the n. of PA bits corresponding to the specified encoding.
 */
int decodePhysAddrRange64(uint8_t pa_enc);

/**
 * Returns the encoding corresponding to the specified n. of PA bits.
 */
uint8_t encodePhysAddrRange64(int pa_size);

inline ByteOrder byteOrder(ThreadContext *tc)
{
    return isBigEndian64(tc) ? BigEndianByteOrder : LittleEndianByteOrder;
};

//yh+begin
static u64 w0, w1, k0, k1;
static u64 m;
//static u8 t[16];

static const u64 alpha = {0xC0AC29B7C97C50DD};
static const u64 c[8] = {0x0000000000000000, 0x13198A2E03707344, 0xA4093822299F31D0,
							0x082EFA98EC4E6C89, 0x452821E638D01377, 0xBE5466CF34E90C6C,
							0x3F84D5B5B5470917, 0x9216D5D98979FB1B};

static const int h[16] = {
	6, 5, 14, 15, 0, 1, 2, 3, 7, 12, 13, 4, 8, 9, 10, 11
};
static const int h_inv[] = {
	4, 5, 6, 7, 11, 1, 0, 8, 12, 13, 14, 15, 9, 10, 2, 3
};
static const int tau[16] = {
	0, 11, 6, 13, 10, 1, 12, 7, 5, 14, 3, 8, 15, 4, 9, 2
};
static const int tau_inv[] = {
	0, 5, 15, 10, 13, 8, 2, 7, 11, 14, 4, 1, 6, 3, 9, 12
};
static const u8 sigma[] = {
	0, 14, 2, 10, 9, 15, 8, 11, 6, 4, 3, 7, 13, 12, 1, 5
};

static const int M[4][4] = {	//note: Q = M
	{0, 1, 2, 1},
	{1, 0, 1, 2},
	{2, 1, 0, 1},
	{1, 2, 1, 0}
};

#define OMIGA_FORWARD(a, i) 	\
		(a[i] = (((a[i]) & 1) ^ ((a[i] >> 1) & 1)) << 3 ^ ((a[i] >> 1) & MASK64));

#define OMIGA_BACKWARD(a, i) 	\
		(a[i] = ((((a[i] >> 3) & 1) ^ (a[i] & 1))) ^ ((a[i] << 1) & MASK64));

#define PERMUTATION(a, b, per)  \
	for(int i = 0; i < 16; i ++){	\
		a[i] = b[per[i]];	\
	}	\

#define ROTR4(x, n) \
	(n == 0 ? 0 : (((x) >> (n) | ((x) << (4-(n)))) & MASK64))
#define ROTR64(x, n) \
	((x) >> (n) | ((x) << (64-(n))))
#define ROTL4(x, n) \
	(n == 0 ? 0 : (((x) << (n) | ((x) >> (4-(n)))) & MASK64))

static inline void u64_to_cells(u64 tt, u8 cells[16])
{
	for(int i = 0; i < 16; i ++){
		cells[15-i] = (u8)(tt & MASK64);
		tt >>= m;
	}
}

static u64 cells_to_u64(u8 cells[16])
{
	u64 tt = 0;

	tt += (u64)cells[0];
	for(int i = 1; i < 16; i ++){
		tt <<= m; 
		tt += (u64)cells[i];
	}

	return tt;
}

//void mix_columns(u8 src[4][4], u8 dst[4][4])
static void mix_columns(u8 src[16], u8 dst[16])
{
	for(int i = 0; i < 4; i ++){
		for(int j = 0; j < 4; j ++){
			//dst[i][j] = ROTL4(src[0][j], M[i][0]) ^ ROTL4(src[1][j], M[i][1]) 
			//				^ ROTL4(src[2][j], M[i][2]) ^ ROTL4(src[3][j], M[i][3]);
			dst[i*4+j] = ROTL4(src[0*4+j], M[i][0]) ^ ROTL4(src[1*4+j], M[i][1]) 
							^ ROTL4(src[2*4+j], M[i][2]) ^ ROTL4(src[3*4+j], M[i][3]);
		}
	}
}

static void key_spec(u64 k[2], int type)
{
	u8 cells1[16], cells2[16];

	if(type == ENC)
	{
		w0 = k[0];
		k0 = k[1];
		w1 = ROTR64(w0, 1) ^ (w0 >> 63);
		k1 = k0;
	} else if(type == DEC) {
		w1 = k[0];
		w0 = ROTR64(w1, 1) ^ (w1 >> 63);
		k0 = k[1] ^ alpha;
		u64_to_cells(k[1], cells1);
		//mix_columns((u8 **)cells1, (u8 **)cells2);
		mix_columns((u8 *)cells1, (u8 *)cells2);
		k1 = cells_to_u64(cells2);
	}
}

static u64 update_tweakey(u64 tt)
{
	u8 cells1[16],cells2[16];
	u64_to_cells(tt, cells1);
	PERMUTATION(cells2, cells1, h);
	OMIGA_FORWARD(cells2, 0);
	OMIGA_FORWARD(cells2, 1);
	OMIGA_FORWARD(cells2, 3);
	OMIGA_FORWARD(cells2, 4);
	OMIGA_FORWARD(cells2, 8);
	OMIGA_FORWARD(cells2, 11);
	OMIGA_FORWARD(cells2, 13);
	return cells_to_u64(cells2);
}

static u64 update_tweakey_inv(u64 tt)
{
	u8 cells1[16],cells2[16];

	u64_to_cells(tt, cells1);
	OMIGA_BACKWARD(cells1, 0);
	OMIGA_BACKWARD(cells1, 1);
	OMIGA_BACKWARD(cells1, 3);
	OMIGA_BACKWARD(cells1, 4);
	OMIGA_BACKWARD(cells1, 8);
	OMIGA_BACKWARD(cells1, 11);
	OMIGA_BACKWARD(cells1, 13);
	PERMUTATION(cells2, cells1, h_inv);
	return cells_to_u64(cells2);
}

static inline void sub_cells(u8 cells[16])
{
	for(int i = 0; i < 16; i ++){
		cells[i] = sigma[cells[i]];
	}
}

static void forward_round_func(u64 *tp, u64 *tt, int i)
{
	u8 cells1[16],cells2[16];

	(*tp) ^= k0 ^ c[i] ^ (*tt);		//add_round tweakey
	*tt = update_tweakey(*tt);

	u64_to_cells(*tp, cells1);
	if(i != 0) {
		PERMUTATION(cells2, cells1, tau);
		//mix_columns((u8 **)cells2, (u8 **)cells1);
		mix_columns((u8 *)cells2, (u8 *)cells1);
	}
	sub_cells(cells1);
	*tp = cells_to_u64(cells1);
}

static void pseudo_reflect(u64 *tp, u64 tt)
{
	u8 cells1[16],cells2[16];
	
	(*tp) ^= tt ^ w1;
	u64_to_cells(*tp, cells1);
	PERMUTATION(cells2, cells1, tau);
	//mix_columns((u8 **)cells2, (u8 **)cells1);
	mix_columns((u8 *)cells2, (u8 *)cells1);
	sub_cells(cells1);

	PERMUTATION(cells2, cells1, tau);
	//mix_columns((u8 **)cells2, (u8 **)cells1);
	mix_columns((u8 *)cells2, (u8 *)cells1);
	u64_to_cells(k1 ^ cells_to_u64(cells1), cells1);
	PERMUTATION(cells2, cells1, tau_inv);
	
	sub_cells(cells2);
	//mix_columns((u8 **)cells2, (u8 **)cells1);
	mix_columns((u8 *)cells2, (u8 *)cells1);
	PERMUTATION(cells2, cells1, tau_inv);
	(*tp) = cells_to_u64(cells2) ^ w0 ^ tt; 
}

static void backward_round_func(u64 *tp, u64 *tt, int i)
{
	u8 cells1[16],cells2[16];

	*tt = update_tweakey_inv(*tt);

	u64_to_cells(*tp, cells1);
	sub_cells(cells1);
	if(i != 0) {
		//mix_columns((u8 **)cells1, (u8 **)cells2);
		mix_columns((u8 *)cells1, (u8 *)cells2);
		PERMUTATION(cells1, cells2, tau_inv);
	}
	*tp = cells_to_u64(cells1);
	(*tp) ^= k0 ^ c[i] ^ (*tt) ^ alpha;
}

static u64 qarma_64(u64 p, u64 tweakey)
{
	u64 tp;
	u64 tt;

	tp = p ^ w0;
	tt = tweakey;
	m = LEN64;

	for(int i = 0; i < ROUND64; i ++) {
		forward_round_func(&tp, &tt, i);  
	}
	pseudo_reflect(&tp, tt);
	for(int i = ROUND64-1; i >= 0; i --) {
		backward_round_func(&tp, &tt, i);  
	}
	tp ^= w1;
	return tp;
}

inline u64 qarma_64_enc(u64 p, u64 tweakey)
{
	////TODO
  //u64 k[2] = {0x84be85ce9804e94b, 0xec2802d4e0a488e9};
	//return (p ^ k[0] ^ tweakey);

  u64 k[2] = {0x84be85ce9804e94b, 0xec2802d4e0a488e9};
	key_spec(k, ENC);

  u64 res = qarma_64(p, tweakey);
  //u64 PAC = (res << 52) >> 52;
  //printf("[QARMA] p: %lu tweakey: %lu res: %lu PAC: %lu\n", p, tweakey, res, PAC);

  return res;
	//return qarma_64(p, tweakey);
}

inline uint64_t getAHC(uint64_t Op164, uint64_t Op364)
{
    uint64_t ahc = 0;
    uint64_t imm = ~(Op164 ^ (Op164 + Op364));
    imm = imm & 0x1FFFFFFFFFFF;
    uint64_t val1 = 1;
    uint64_t val0 = 1;

    //printf("Op164: %lu Op364: %lu\n", Op164, Op364);

    //printf("val1: %lu val2: %lu\n",
    //        (uint64_t) ((imm >> 7) ^ 0x3FFFFFFFFF),
    //        (uint64_t) ((imm >> 10) ^ 0x7FFFFFFFF));
    if (((imm >> 7) ^ 0x3FFFFFFFFF) > 0) {
      val1 = 0;
    }

    if (((imm >> 10) ^ 0x7FFFFFFFF) > 0) {
      val0 = 0;
    }

    if (val1 == 0 || val0 == 0) {
        ahc = ahc | ((uint64_t) 1 << 46);
    }

    if ((val1 == 1 && val0 == 1) || (val1 == 0 && val1 == 0)) {
        ahc = ahc | ((uint64_t) 1 << 45);
    }

    return ahc;
}

inline void print_pacma(uint64_t a, uint64_t b, uint64_t c, uint64_t d)
{
  printf("[PACMA] Addr: %lu Modifier: %lu size: %lu PAC: %lu\n", a, b, c, d);
}

inline void print_pacda(uint64_t a, uint64_t b, uint64_t c)
{
  printf("[PACDA] Addr: %lu Modifier: %lu PAC: %lu\n", a, b, c);
}

inline void print_xpacm(uint64_t a, uint64_t b)
{
  printf("[XPACM] (%lu) -> (%lu)\n", a, b);
}

inline void print_addr_modifier(uint64_t a, uint64_t b, int c)
{
  if (c == 0)
      printf("[PACMA] Addr: %lu Modifier: %lu\n", a, b);
  else if (c == 1)
      printf("[PACIB] Addr: %lu Modifier: %lu\n", a, b);
  else if (c == 2)
      printf("[AUTIB] Addr: %lu Modifier: %lu\n", a, b);
  else if (c == 3)
      printf("[AUTDA] Addr: %lu Modifier: %lu\n", a, b);
  else if (c == 4)
      printf("[PACDA] Addr: %lu Modifier: %lu\n", a, b);
}

inline void print_error_autib(uint64_t a, uint64_t b)
{
  printf("[AUTIA] Authentication failed! expected: %lu result: %lu\n", a, b);
}

inline void print_error_autda(uint64_t a, uint64_t b)
{
  printf("[AUTDA] Authentication failed! expected: %lu result: %lu\n", a, b);
}

inline void print_error_autm(uint64_t ahc)
{
  printf("[AUTM] Authentication failed! expected: 0 result: %lu\n", ahc);
}

inline void print_error(uint64_t a, uint64_t b)
{
  printf("[AUTIASP] Smashed return address found! expected: %lu result: %lu\n", a, b);
}

inline void print_error2(uint64_t a, uint64_t b)
{
  printf("[RETAA] Smashed return address found! expected: %lu result: %lu\n", a, b);
}

inline void printvp(uint64_t op1, uint64_t op2, uint64_t pac, uint64_t dest)
{
  printf("[PACIASP] op1: %lu op2: %lu pac: %lu dest: %lu\n",
          op1, op2, pac, dest);
}

inline void printva(uint64_t op1, uint64_t op2, uint64_t pac, uint64_t dest)
{
  printf("[AUTIASP] op1: %lu op2: %lu pac: %lu dest: %lu\n",
          op1, op2, pac, dest);
}

//u64 qarma_64_dec(u64 k[2], u64 c, u64 tweakey)
//{
//	key_spec(k, DEC);
//	return qarma_64(c, tweakey);
//}

//int main()
//{
//	u64 p = 0xfb623599da6e8127;
//	u64 T = 0x477d469dec0b8762;
//	u64 k[2] = {0x84be85ce9804e94b, 0xec2802d4e0a488e9};
//	u64 c = qarma_64_enc(k, p);
//	u64 np = qarma_64_dec(k, c, T);
//
//	printf("Cipher text: %lx == %lx  %c\n", c, 0xbcaf6c89de930765, (c == 0xbcaf6c89de930765) ? 'y' : 'n');
//	printf("Plain text: %lx == %lx  %c\n", np, 0xfb623599da6e8127, (np == 0xfb623599da6e8127) ? 'y' : 'n');
//}
//yh+end
}
#endif
