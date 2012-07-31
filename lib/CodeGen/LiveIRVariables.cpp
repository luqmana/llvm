//===- LiveIRVariables.cpp - Live Variable Analysis for IR ----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the LiveIRVariables analysis pass.  For each instruction
// in the function, this pass calculates the set of values that are immediately
// dead after the instruction (i.e., the instruction calculates the value, but
// it is never used) and the set of values that are used by the instruction, but
// are never used after the instruction (i.e., they are killed).
//
// This class computes live variable information for each instruction and
// argument in a function using a sparse implementation based on SSA form.  It
// uses the dominance properties of SSA form to efficiently compute liveness of
// values, by walking the graph of uses back to the initial definition of each
// value.
//
// Currently it is used to track liveness of garbage collector roots.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "liveness"

#include "llvm/Function.h"
#include "llvm/Instructions.h"
#include "llvm/Pass.h"
#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/LiveIRVariables.h"
#include "llvm/Support/CFG.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

char LiveIRVariables::ID = 0;
INITIALIZE_PASS_BEGIN(LiveIRVariables, "liveness", "Live IR Variables", false,
                      true)
INITIALIZE_PASS_END(LiveIRVariables, "liveness", "Live IR Variables", false,
                    true)

void
LiveIRVariables::getAnalysisUsage(AnalysisUsage &AU) const {
  FunctionPass::getAnalysisUsage(AU);
  AU.setPreservesAll();
}

bool
LiveIRVariables::runOnFunction(Function &F) {
  DEBUG(dbgs() << "********** LIVE IR VARIABLES **********\n");

  DFSOrdering.reset();

  ComputeDFSOrdering(F);
  ComputeLiveSets(F);

  DEBUG(dump(F, true));

  return false;
}

void
LiveIRVariables::ComputeDFSOrdering(const Function &F) {
  for (df_iterator<const BasicBlock *> DI = df_begin(&F.getEntryBlock()),
       DE = df_end(&F.getEntryBlock()); DI != DE; ++DI) {
    DFSOrdering.insert(*DI);
  }
}

void
LiveIRVariables::ComputeLiveSets(const Function &F) {
  // Calculate live variable information in depth first order on the
  // CFG of the function. This guarantees that we will see the
  // definition of a virtual register before its uses due to dominance
  // properties of SSA (except for PHI nodes, which are treated as a
  // special case).
  for (df_iterator<const BasicBlock *> DI = df_begin(&F.getEntryBlock()),
       DE = df_end(&F.getEntryBlock()); DI != DE; ++DI) {
    const BasicBlock &BB = **DI;

    // Loop over all of the instructions, processing them.
    for (BasicBlock::const_iterator II = BB.begin(),
         IE = BB.end(); II != IE; ++II) {
      const Instruction &I = *II;

      // Process all of the operands of the instruction, unless it is
      // a PHI node. In this case, ONLY process the value defined by
      // the PHI node, not any of the uses. They will be handled in
      // other basic blocks.
      if (!isa<PHINode>(&I)) {
        for (User::const_op_iterator OI = I.op_begin(),
             OE = I.op_end(); OI != OE; ++OI) {
          const Value &V = **OI;

          if (isa<Instruction>(V) || isa<Argument>(V)) {
            HandleUse(V, I);
          }
        }
      }

      // Process value defined by instruction.
      HandleDef(I);
    }
  }
}

void
LiveIRVariables::HandleUse(const Value &Op, const Instruction &I) {
  const BasicBlock &BB = *I.getParent();
  unsigned BBNum = DFSOrdering.idFor(&BB) - 1;

  LivenessInfo &LInfo = getLivenessInfo(Op);

  // Check to see if this basic block is one of the killing blocks.  If so,
  // remove it.
  for (unsigned i = 0, e = LInfo.Kills.size(); i != e; ++i)
    if (LInfo.Kills[i]->getParent() == &BB) {
      LInfo.Kills.erase(LInfo.Kills.begin()+i);  // Erase entry
      break;
    }

  // This situation can occur:
  //
  //     ,------.
  //     |      |
  //     |      v
  //     |   t2 = phi ... t1 ...
  //     |      |
  //     |      v
  //     |   t1 = ...
  //     |  ... = ... t1 ...
  //     |      |
  //     `------'
  //
  // where there is a use in a PHI node that's a predecessor to the defining
  // block. We don't want to mark all predecessors as having the value "alive"
  // in this case.
  if (&BB == &getDefiningBlock(Op)) return;

  // Add a new kill entry for this basic block. If this value is
  // already marked as alive in this basic block, that means it is
  // alive in at least one of the successor blocks, it's not a kill.
  if (!LInfo.AliveBlocks.test(BBNum)) {
    LInfo.Kills.push_back(&I);
  }

  // Update all dominating blocks to mark them as "known live".
  for (const_pred_iterator PI = pred_begin(&BB),
         PE = pred_end(&BB); PI != PE; ++PI) {
    MarkAliveInBlock(LInfo, getDefiningBlock(Op), **PI);
  }
}

void
LiveIRVariables::HandleDef(const Instruction &I) {
  LivenessInfo &LInfo = getLivenessInfo(I);

  if (LInfo.AliveBlocks.empty())
    // If value is not alive in any block, then defaults to dead.
    LInfo.Kills.push_back(&I);
}

LiveIRVariables::LivenessInfo &
LiveIRVariables::getLivenessInfo(const Value &V) {
  if (!Liveness.count(&V)) {
    Liveness.insert(std::pair<const Value *, LiveIRVariables::LivenessInfo>(&V, LivenessInfo()));
  }
  return Liveness[&V];
}

const BasicBlock &
LiveIRVariables::getDefiningBlock(const Value &V) {
  if (isa<Argument>(V))
    return cast<Argument>(V).getParent()->getEntryBlock();
  if (isa<Instruction>(V))
    return *cast<Instruction>(V).getParent();

  llvm_unreachable("Only arguments and instructions have definition sites!");
}

void
LiveIRVariables::MarkAliveInBlock(LivenessInfo &LInfo,
                                  const BasicBlock &DefBlock,
                                  const BasicBlock &BB) {
  std::vector<const BasicBlock *> WorkList;
  MarkAliveInBlock(LInfo, DefBlock, BB, WorkList);

  while (!WorkList.empty()) {
    const BasicBlock &Pred = *WorkList.back();
    WorkList.pop_back();
    MarkAliveInBlock(LInfo, DefBlock, Pred, WorkList);
  }
}

void
LiveIRVariables::MarkAliveInBlock(LivenessInfo& LInfo,
                                  const BasicBlock &DefBlock,
                                  const BasicBlock &BB,
                                  std::vector<const BasicBlock *> &WorkList) {
  const Function &F = *BB.getParent();
  unsigned BBNum = DFSOrdering.idFor(&BB) - 1;

  // Check to see if this basic block is one of the killing blocks.  If so,
  // remove it.
  for (unsigned i = 0, e = LInfo.Kills.size(); i != e; ++i) {
    if (LInfo.Kills[i]->getParent() == &BB) {
      LInfo.Kills.erase(LInfo.Kills.begin() + i);  // Erase entry
      break;
    }
  }

  if (&BB == &DefBlock) return;  // Terminate recursion

  if (LInfo.AliveBlocks.test(BBNum)) {
    return;  // We already know the block is live
  }

  // Mark the variable known alive in this block
  LInfo.AliveBlocks.set(BBNum);

  assert(&BB != &F.getEntryBlock() && "Can't find reaching def for value");
  WorkList.insert(WorkList.end(), pred_begin(&BB), pred_end(&BB));
}

const Instruction *
LiveIRVariables::LivenessInfo::findKill(const BasicBlock *BB) {
  for (unsigned i = 0, e = Kills.size(); i != e; ++i) {
    if (Kills[i]->getParent() == BB) {
      return Kills[i];
    }
  }
  return NULL;
}

bool
LiveIRVariables::LivenessInfo::isLiveIn(const Value &V, const BasicBlock &Def,
                                        const BasicBlock &BB, unsigned BBNum) {
  // V is live-through.
  if (AliveBlocks.test(BBNum)) {
    return true;
  }

  // Values defined in BB cannot be live in.
  if (&Def == &BB) {
    return false;
  }

  // V was not defined in BB, was it killed here?
  return findKill(&BB);
}

bool
LiveIRVariables::isLiveIn(const Value &V, const BasicBlock &BB) {
  return getLivenessInfo(V).isLiveIn(V, getDefiningBlock(V),
                                     BB, DFSOrdering.idFor(&BB) - 1);
}

bool
LiveIRVariables::isLiveOut(const Value &V, const BasicBlock &BB) {
  LiveIRVariables::LivenessInfo &LI = getLivenessInfo(V);

  // Loop over all of the successors of the basic block, checking to see if
  // the value is either live in the block, or if it is killed in the block.
  SmallVector<const BasicBlock*, 8> OpSuccBlocks;
  for (succ_const_iterator SI = succ_begin(&BB),
       SE = succ_end(&BB); SI != SE; ++SI) {
    const BasicBlock *SuccBB = *SI;

    // Is it alive in this successor?
    unsigned SuccIdx = DFSOrdering.idFor(SuccBB) - 1;
    if (LI.AliveBlocks.test(SuccIdx))
      return true;
    OpSuccBlocks.push_back(SuccBB);
  }

  // Check to see if this value is live because there is a use in a successor
  // that kills it.
  switch (OpSuccBlocks.size()) {
  case 1: {
    const BasicBlock *SuccBB = OpSuccBlocks[0];
    for (unsigned i = 0, e = LI.Kills.size(); i != e; ++i)
      if (LI.Kills[i]->getParent() == SuccBB)
        return true;
    break;
  }
  case 2: {
    const BasicBlock *SuccBB1 = OpSuccBlocks[0], *SuccBB2 = OpSuccBlocks[1];
    for (unsigned i = 0, e = LI.Kills.size(); i != e; ++i)
      if (LI.Kills[i]->getParent() == SuccBB1 ||
          LI.Kills[i]->getParent() == SuccBB2)
        return true;
    break;
  }
  default:
    std::sort(OpSuccBlocks.begin(), OpSuccBlocks.end());
    for (unsigned i = 0, e = LI.Kills.size(); i != e; ++i)
      if (std::binary_search(OpSuccBlocks.begin(), OpSuccBlocks.end(),
                             LI.Kills[i]->getParent()))
        return true;
  }
  return false;
}

void LiveIRVariables::dump(Function &F, bool IncludeDead) {
  dbgs() << "Function: " << F.getName() << "\n";
  for (Function::iterator BBI = F.begin(), BBE = F.end(); BBI != BBE; ++BBI) {
    unsigned BBNum = DFSOrdering.idFor(BBI) - 1;
    dbgs() << "Basic Block " << BBNum << ":\n";
    BBI->dump();
  }

  dbgs() << "Liveness:\n";
  for (Function::arg_iterator AI = F.arg_begin(),
                              AE = F.arg_end(); AI != AE; ++AI) {
    dbgs() << "Value: " << *AI << "\n";
    for (Function::iterator BBI = F.begin(),
                            BBE = F.end(); BBI != BBE; ++BBI) {
      bool LiveIn = isLiveIn(*AI, *BBI), LiveOut = isLiveOut(*AI, *BBI);
      if (!LiveIn && !LiveOut && !IncludeDead)
        continue;
      dbgs() << "  is " << (LiveIn ? "" : "NOT ") << "live-in ";
      dbgs() << (LiveIn != LiveOut ? "but " : "and ");
      dbgs() << (LiveOut ? "" : "NOT ") << "live-out at basic block ";
      dbgs() << (DFSOrdering.idFor(&*BBI) - 1) << "\n";
    }
  }

  for (Function::iterator BBAI = F.begin(),
                          BBAE = F.end(); BBAI != BBAE; ++BBAI) {
    for (BasicBlock::iterator II = BBAI->begin(),
                              IE = BBAI->end(); II != IE; ++II) {
      dbgs() << "Value: " << *II << "\n";
      for (Function::iterator BBBI = F.begin(),
                              BBBE = F.end(); BBBI != BBBE; ++BBBI) {
        bool LiveIn = isLiveIn(*II, *BBBI), LiveOut = isLiveOut(*II, *BBBI);
        if (!LiveIn && !LiveOut && !IncludeDead)
          continue;
        dbgs() << "  is " << (LiveIn ? "" : "NOT ") << "live-in ";
        dbgs() << (LiveIn != LiveOut ? "but " : "and ");
        dbgs() << (LiveOut ? "" : "NOT ") << "live-out at basic block ";
        dbgs() << (DFSOrdering.idFor(&*BBBI) - 1) << "\n";
      }
    }
  }
}

