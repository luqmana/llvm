//===-- MipsFrameLowering.cpp - Mips Frame Information --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Mips implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "MipsFrameLowering.h"
#include "MCTargetDesc/MipsBaseInfo.h"
#include "MipsAnalyzeImmediate.h"
#include "MipsInstrInfo.h"
#include "MipsMachineFunction.h"
#include "MipsTargetMachine.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Target/TargetOptions.h"

using namespace llvm;


//===----------------------------------------------------------------------===//
//
// Stack Frame Processing methods
// +----------------------------+
//
// The stack is allocated decrementing the stack pointer on
// the first instruction of a function prologue. Once decremented,
// all stack references are done thought a positive offset
// from the stack/frame pointer, so the stack is considering
// to grow up! Otherwise terrible hacks would have to be made
// to get this stack ABI compliant :)
//
//  The stack frame required by the ABI (after call):
//  Offset
//
//  0                 ----------
//  4                 Args to pass
//  .                 saved $GP  (used in PIC)
//  .                 Alloca allocations
//  .                 Local Area
//  .                 CPU "Callee Saved" Registers
//  .                 saved FP
//  .                 saved RA
//  .                 FPU "Callee Saved" Registers
//  StackSize         -----------
//
// Offset - offset from sp after stack allocation on function prologue
//
// The sp is the stack pointer subtracted/added from the stack size
// at the Prologue/Epilogue
//
// References to the previous stack (to obtain arguments) are done
// with offsets that exceeds the stack size: (stacksize+(4*(num_arg-1))
//
// Examples:
// - reference to the actual stack frame
//   for any local area var there is smt like : FI >= 0, StackOffset: 4
//     sw REGX, 4(SP)
//
// - reference to previous stack frame
//   suppose there's a load to the 5th arguments : FI < 0, StackOffset: 16.
//   The emitted instruction will be something like:
//     lw REGX, 16+StackSize(SP)
//
// Since the total stack size is unknown on LowerFormalArguments, all
// stack references (ObjectOffset) created to reference the function
// arguments, are negative numbers. This way, on eliminateFrameIndex it's
// possible to detect those references and the offsets are adjusted to
// their real location.
//
//===----------------------------------------------------------------------===//

const MipsFrameLowering *MipsFrameLowering::create(MipsTargetMachine &TM,
                                                   const MipsSubtarget &ST) {
  if (TM.getSubtargetImpl()->inMips16Mode())
    return llvm::createMips16FrameLowering(ST);

  return llvm::createMipsSEFrameLowering(ST);
}

// hasFP - Return true if the specified function should have a dedicated frame
// pointer register.  This is true if the function has variable sized allocas or
// if frame pointer elimination is disabled.
bool MipsFrameLowering::hasFP(const MachineFunction &MF) const {
  const MachineFrameInfo *MFI = MF.getFrameInfo();
  return MF.getTarget().Options.DisableFramePointerElim(MF) ||
      MFI->hasVarSizedObjects() || MFI->isFrameAddressTaken();
}

uint64_t MipsFrameLowering::estimateStackSize(const MachineFunction &MF) const {
  const MachineFrameInfo *MFI = MF.getFrameInfo();
  const TargetRegisterInfo &TRI = *MF.getTarget().getRegisterInfo();

  int64_t Offset = 0;

  // Iterate over fixed sized objects.
  for (int I = MFI->getObjectIndexBegin(); I != 0; ++I)
    Offset = std::max(Offset, -MFI->getObjectOffset(I));

  // Conservatively assume all callee-saved registers will be saved.
  for (const uint16_t *R = TRI.getCalleeSavedRegs(&MF); *R; ++R) {
    unsigned Size = TRI.getMinimalPhysRegClass(*R)->getSize();
    Offset = RoundUpToAlignment(Offset + Size, Size);
  }

  unsigned MaxAlign = MFI->getMaxAlignment();

  // Check that MaxAlign is not zero if there is a stack object that is not a
  // callee-saved spill.
  assert(!MFI->getObjectIndexEnd() || MaxAlign);

  // Iterate over other objects.
  for (unsigned I = 0, E = MFI->getObjectIndexEnd(); I != E; ++I)
    Offset = RoundUpToAlignment(Offset + MFI->getObjectSize(I), MaxAlign);

  // Call frame.
  if (MFI->adjustsStack() && hasReservedCallFrame(MF))
    Offset = RoundUpToAlignment(Offset + MFI->getMaxCallFrameSize(),
                                std::max(MaxAlign, getStackAlignment()));

  return RoundUpToAlignment(Offset, getStackAlignment());
}

// The stack limit in the TCB is set to this manyu bytes above the actual
// stack limit.
static const uint64_t kSplitStackAvailable = 256;

// helper function for loading immediate
static void loadImm(MachineBasicBlock *MBB, DebugLoc DL,
                    const TargetInstrInfo &TII,
                    unsigned Reg, int64_t Imm) {
    if (isInt<16>(Imm)) {
      // addiu $Reg, $zero, Imm
      BuildMI(MBB, DL, TII.get(Mips::ADDiu), Reg)
              .addReg(Mips::ZERO).addImm(Imm);
    } else {
      // lui $Reg, (Imm >> 16)
      BuildMI(MBB, DL, TII.get(Mips::LUi), Reg)
              .addImm(Imm >> 16);
      // ori $Reg, $Reg, (Imm & 0xffff)
      BuildMI(MBB, DL, TII.get(Mips::ORi), Reg)
              .addReg(Reg).addImm(Imm & 0xffff);
    }
}

// Adjust function prologue to enable split stack.
void
MipsFrameLowering::adjustForSegmentedStacks(MachineFunction &MF) const {
  const TargetMachine &TM = MF.getTarget();
  const MipsSubtarget *ST = &TM.getSubtarget<MipsSubtarget>();

  // Doesn't support vararg function.
  if (MF.getFunction()->isVarArg())
    report_fatal_error("Segmented stacks do not support vararg functions.");
  // Doesn't support other than linux o32 with pic relocation.
  if (!ST->isLinux() || !ST->isABI_O32() || TM.getRelocationModel() != Reloc::PIC_)
    report_fatal_error("Segmented statks not supported on this platfrom.");

  MachineBasicBlock &prologueMBB = MF.front();
  MachineFrameInfo* MFI = MF.getFrameInfo();
  const TargetInstrInfo &TII = *TM.getInstrInfo();
  MipsFunctionInfo* MipsFI = MF.getInfo<MipsFunctionInfo>();
  DebugLoc DL;

  // Use t6 and t7 as scratch register.
  unsigned ScratchReg0 = Mips::T6;
  unsigned ScratchReg1 = Mips::T7;
  // Use the private field of tcb head.
  unsigned TlsOffset = 0x7004;

  MachineBasicBlock* allocMBB = MF.CreateMachineBasicBlock();
  MachineBasicBlock* checkMBB = MF.CreateMachineBasicBlock();

  for (MachineBasicBlock::livein_iterator i = prologueMBB.livein_begin(),
         e = prologueMBB.livein_end(); i != e; ++i) {
    allocMBB->addLiveIn(*i);
    checkMBB->addLiveIn(*i);
  }

  MF.push_front(allocMBB);
  MF.push_front(checkMBB);

  uint64_t StackSize = MFI->getStackSize();

  // If the front-end requested a fixed stack segment size, use that.
  const Function *Fn = MF.getFunction();
  if (Fn->getAttributes().hasAttribute(AttributeSet::FunctionIndex,
                                       Attribute::FixedStackSegment)) {
    StackSize = TM.Options.FixedStackSegmentSize;
  }

  // When the frame size is less than 256 we just compare the stack
  // boundary directly to the value of the stack pointer, per gcc.
  bool CompareStackPointer = StackSize < kSplitStackAvailable;

  if (!MipsFI->globalBaseRegSet()) {
    // lui $v0, %hi(_gp_disp)
    BuildMI(checkMBB, DL, TII.get(Mips::LUi), Mips::V0)
            .addExternalSymbol("_gp_disp", MipsII::MO_ABS_HI);
    // addiu $v0, $v0, %lo(_gp_disp)
    BuildMI(checkMBB, DL, TII.get(Mips::ADDiu), Mips::V0)
            .addReg(Mips::V0)
            .addExternalSymbol("_gp_disp", MipsII::MO_ABS_LO);
    allocMBB->addLiveIn(Mips::V0);
    allocMBB->addLiveIn(Mips::T9);
  }

  if (CompareStackPointer) {
    // addu $SR1, $sp, $zero
    BuildMI(checkMBB, DL, TII.get(Mips::ADDu), ScratchReg1)
            .addReg(Mips::SP).addReg(Mips::ZERO);
  } else {
    // li $SR1, -StackSize
    loadImm(checkMBB, DL, TII, ScratchReg1, -StackSize);
    // addu $SR1, $sp, $SR1
    BuildMI(checkMBB, DL, TII.get(Mips::ADDu), ScratchReg1)
            .addReg(Mips::SP).addReg(ScratchReg1);
  }

  // rdhwr $v1, $sp
  BuildMI(checkMBB, DL, TII.get(Mips::RDHWR), Mips::V1)
          .addReg(Mips::HWR29);

  // addiu $v1, $v1, -TlsOffset
  BuildMI(checkMBB, DL, TII.get(Mips::ADDiu), Mips::V1)
          .addReg(Mips::V1).addImm((-TlsOffset) & 0xffff);

  // Get stack limit.
  // lw $SR0, 0($v1)
  BuildMI(checkMBB, DL, TII.get(Mips::LW), ScratchReg0)
          .addReg(Mips::V1).addImm(0);

  // Compare stack limit with stack size requested.
  // sltu $v1, $SR0, $SR1
  BuildMI(checkMBB, DL, TII.get(Mips::SLTu), Mips::V1)
          .addReg(ScratchReg0).addReg(ScratchReg1);

  // This jump is taken if StackLimit < (SP - stack required).
  // bne $v1, $zero, $prologueMBB
  BuildMI(checkMBB, DL, TII.get(Mips::BNE))
          .addReg(Mips::V1).addReg(Mips::ZERO).addMBB(&prologueMBB);

  // Calling __morestack(StackSize, Size of stack arguments).
  // __morestack knows that the stack size requested is in SR0(t6)
  // and amount size of stack arguments is in SR1(t7).

  // Pass first argument for the __morestack by Scratch Register #0.
  // The amount size of stack required
  // li $SR0, StackSize
  loadImm(allocMBB, DL, TII, ScratchReg0, StackSize);

  // Pass second argument for the __morestack by Scratch Register #1.
  // The amount size of stack consumed to save function arguments.
  // li $SR1, ArgumentStackSize
  loadImm(allocMBB, DL, TII, ScratchReg1, MipsFI->getArgumentStackSize());

  // addiu $sp, $sp, -12
  BuildMI(allocMBB, DL, TII.get(Mips::ADDiu), Mips::SP)
          .addReg(Mips::SP).addImm(-12);
  // Save return address of this function.
  // sw $ra, 8($sp)
  BuildMI(allocMBB, DL, TII.get(Mips::SW))
          .addReg(Mips::RA).addReg(Mips::SP).addImm(8);
  // Save function address and _gp_disp for PIC.
  // sw $t9, 4($sp)
  BuildMI(allocMBB, DL, TII.get(Mips::SW))
          .addReg(Mips::T9).addReg(Mips::SP).addImm(4);
  // sw $v0, 0($sp)
  BuildMI(allocMBB, DL, TII.get(Mips::SW))
          .addReg(Mips::V0).addReg(Mips::SP).addImm(0);

  // addu $gp, $v0, $t9
  BuildMI(allocMBB, DL, TII.get(Mips::ADDu), Mips::GP)
          .addReg(Mips::V0).addReg(Mips::T9);
  // lw $t9, %call16(__morestack)($gp)
  BuildMI(allocMBB, DL, TII.get(Mips::LW), Mips::T9)
          .addReg(Mips::GP)
          .addExternalSymbol("__morestack", MipsII::MO_GOT_CALL);
  // jalr $t9
  BuildMI(allocMBB, DL, TII.get(Mips::JALR), Mips::RA)
          .addReg(Mips::T9);

  // Restore return address of this original function.
  // lw $ra, 8($sp)
  BuildMI(allocMBB, DL, TII.get(Mips::LW), Mips::RA)
          .addReg(Mips::SP).addImm(8);
  // addiu $sp, $sp, 12
  BuildMI(allocMBB, DL, TII.get(Mips::ADDiu), Mips::SP)
          .addReg(Mips::SP).addImm(12);

  // Return from this function.
  // jr $ra
  BuildMI(allocMBB, DL, TII.get(Mips::JR))
          .addReg(Mips::RA);

  // Organizing MBB lists
  allocMBB->addSuccessor(&prologueMBB);

  checkMBB->addSuccessor(allocMBB);
  checkMBB->addSuccessor(&prologueMBB);

#ifdef XDEBUG
  MF.verify();
#endif
}
