//===- LiveIRVariables.h - Live Variable Analysis for IR --------*- C++ -*-===//
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

#ifndef LLVM_ANALYSIS_LIVEIRVARIABLES_H
#define LLVM_ANALYSIS_LIVEIRVARIABLES_H

#include "llvm/InitializePasses.h"
#include "llvm/Pass.h"
#include "llvm/ADT/SparseBitVector.h"
#include "llvm/ADT/UniqueVector.h"
#include "llvm/ADT/ValueMap.h"
#include <vector>

namespace llvm {

class BasicBlock;
class Function;

/// Analysis pass providing liveness information.
class LiveIRVariables : public FunctionPass {
public:
  static char ID;

  LiveIRVariables() : FunctionPass(ID) {
    initializeLiveIRVariablesPass(*PassRegistry::getPassRegistry());
  }

  void getAnalysisUsage(AnalysisUsage &AU) const;
  bool runOnFunction(Function &F);

  /// isLiveIn - Returns true if V is live-in to BB or false otherwise. This
  /// means that V is live through BB, or it is killed in BB. If V is only used
  /// by PHI nodes in BB, it is not considered live-in.
  bool isLiveIn(const Value &V, const BasicBlock &BB);

  /// isLiveOut - Returns true if V is live-out from BB or false otherwise,
  /// ignoring PHI nodes. This means that the value is either killed by a
  /// successor block or passed through one.
  bool isLiveOut(const Value &V, const BasicBlock &BB);

  /// Dumps the liveness information for all instructions, for debugging.
  /// IncludeDead should be set to true if even dead variables should be
  /// written.
  void dump(Function &F, bool IncludeDead = false);

private:
  /// LivenessInfo - This represents the regions where a value is live in the
  /// program.  We represent this with three different pieces of information:
  /// the set of blocks in which the instruction is live throughout, the set of
  /// blocks in which the instruction is actually used, and the set of non-PHI
  /// instructions that are the last users of the value.
  ///
  /// In the common case where a value is defined and killed in the same block,
  /// There is one killing instruction, and AliveBlocks is empty.
  ///
  /// Otherwise, the value is live out of the block.  If the value is live
  /// throughout any blocks, these blocks are listed in AliveBlocks.  Blocks
  /// where the liveness range ends are not included in AliveBlocks, instead
  /// being captured by the Kills set.  In these blocks, the value is live into
  /// the block (unless the value is defined and killed in the same block) and
  /// lives until the specified instruction.  Note that there cannot ever be a
  /// value whose Kills set contains two instructions from the same basic block.
  ///
  /// PHI nodes complicate things a bit.  If a PHI node is the last user of a
  /// value in one of its predecessor blocks, it is not listed in the kills set,
  /// but does include the predecessor block in the AliveBlocks set (unless that
  /// block also defines the value).  This leads to the (perfectly sensical)
  /// situation where a value is defined in a block, and the last use is a PHI
  /// node in the successor.  In this case, AliveBlocks is empty (the value is
  /// not live across any  blocks) and Kills is empty (PHI nodes are not
  /// included). This is sensical because the value must be live to the end of
  /// the block, but is not live in any successor blocks.
  struct LivenessInfo {
    /// AliveBlocks - Set of blocks in which this value is alive completely
    /// through.  This is a bit set which uses the basic block number as an
    /// index.
    SparseBitVector<> AliveBlocks;

    /// Kills - List of Instruction's which are the last use of this value (kill
    /// it) in their basic block.
    std::vector<const Instruction *> Kills;

    /// findKill - Find a kill instruction in BB. Return NULL if none is found.
    const Instruction *findKill(const BasicBlock *BB);

    /// isLiveIn - Is V live in to BB? This means that V is live through
    /// BB, or it is killed in BB. If V is only used by PHI instructions in
    /// BB, it is not considered live in.
    bool isLiveIn(const Value &V, const BasicBlock &Def,
                  const BasicBlock &BB, unsigned BBNum);

    void dump() const;
  };

  /// DFSOrdering - Maps basic blocks to numeric IDs in DFS order. Used to make
  /// certain bit vector operations on block IDs efficient.
  UniqueVector<const BasicBlock *> DFSOrdering;

  /// Liveness - Tracks the liveness information about every instruction in the
  /// function.
  ValueMap<const Value *, LivenessInfo> Liveness;

  /// ComputeDFSOrdering - Precomputes the DFSOrdering map for use later.
  void ComputeDFSOrdering(const Function &F);

  /// ComputeLiveSets - Computes the liveness information for each instruction
  /// in the function F.
  void ComputeLiveSets(const Function &F);

  /// HandleUse - Propagate the liveness information for the operand V used by
  /// instruction I.
  void HandleUse(const Value &V, const Instruction &I);

  /// HandleDef - Initialize the liveness information for the value defined by
  /// the instruction I. If no later instructions use I, then I will be
  /// considered dead.
  void HandleDef(const Instruction &I);

  /// HandleArg - Post-processing step to ensure arguments are properly marked
  /// live-in at the entry block if they are used anywhere in the function.
  void HandleArg(const Argument &A);

  /// getLivenessInfo - Returns (creating if necessary) the liveness info entry
  /// for value V.
  LivenessInfo &getLivenessInfo(const Value &V);

  /// getDefiningBlock - Returns the block containing V if V is an instruction,
  /// or the entry basic block if V is an argument. Throws an error for other
  /// subclasses of Value.
  const BasicBlock &getDefiningBlock(const Value &V);

  /// MarkAliveInBlock - Propogates liveness information LI upward through the
  /// predecessor blocks of BB until reaching the defining block DefBlock.
  void MarkAliveInBlock(LivenessInfo &LI, const BasicBlock &DefBlock,
                        const BasicBlock &BB);

  /// MarkAliveInBlock - Helper method for propogation of liveness information.
  void MarkAliveInBlock(LivenessInfo &LI, const BasicBlock &DefBlock,
                        const BasicBlock &BB,
                        std::vector<const BasicBlock *> &WorkList);
};

} // end namespace llvm

#endif

