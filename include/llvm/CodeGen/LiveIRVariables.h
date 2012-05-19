//===- LiveIRVariables.h - IR variable liveness -----------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the SSA-based liveness analysis described in this
// paper:
//
//   Boissinot, Hack, Grund, de Dinechin, Rastello, "Fast Liveness Checking for
//   SSA-Form Programs," INRIA Research Report No. RR-2007-45 (2007).
//
// Currently it is used to track liveness of garbage collector roots.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_ANALYSIS_LIVEIRVARIABLES_H
#define LLVM_ANALYSIS_LIVEIRVARIABLES_H

#include "llvm/InitializePasses.h"
#include "llvm/Pass.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/SmallMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/UniqueVector.h"
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

  /// Returns true if the given value is live-in to the given block and false
  /// otherwise.
  bool isLiveIn(Value &V, BasicBlock &BB);

  /// Dumps the liveness information for all instructions, for debugging.
  /// IncludeDead should be set to true if even dead variables should be
  /// written.
  void dump(Function &F, bool IncludeDead = false);

private:
  UniqueVector<BasicBlock *> DFSOrdering;
  DenseSet< std::pair<BasicBlock *, BasicBlock *> > BackEdges;
  std::vector<unsigned> IncomingEdges;          // Number of incoming edges.
  std::vector<BitVector> ReducedReachability;   // R_q in the paper
  std::vector<BitVector> ReachableBackEdges;    // T_q in the paper

  void computeDFSOrdering(BasicBlock &BB);
  void computeBackAndIncomingEdges(Function &F);
  void computeTopologicalOrdering(Function &F,
                                  UniqueVector<BasicBlock *> &Ordering);
  void computeReducedReachability(Function &F);
  void computeReachableBackEdges(Function &F);
};

} // end namespace llvm

#endif

