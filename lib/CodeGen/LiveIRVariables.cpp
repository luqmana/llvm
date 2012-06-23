//===- LiveIRVariables.cpp - IR variable liveness ---------------*- C++ -*-===//
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

#define DEBUG_TYPE "liveness"

#include "llvm/Function.h"
#include "llvm/Pass.h"
#include "llvm/ADT/SetVector.h"
#include "llvm/ADT/SmallMap.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Analysis/Dominators.h"
#include "llvm/CodeGen/LiveIRVariables.h"
#include "llvm/Support/CFG.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

char LiveIRVariables::ID = 0;
INITIALIZE_PASS_BEGIN(LiveIRVariables, "liveness", "Live IR Variables", false,
                      true)
INITIALIZE_PASS_DEPENDENCY(DominatorTree);
INITIALIZE_PASS_END(LiveIRVariables, "liveness", "Live IR Variables", false,
                    true)

void LiveIRVariables::getAnalysisUsage(AnalysisUsage &AU) const {
  FunctionPass::getAnalysisUsage(AU);
  AU.addRequired<DominatorTree>();
  AU.setPreservesAll();
}

bool LiveIRVariables::runOnFunction(Function &F) {
  DEBUG(dbgs() << "********** LIVE IR VARIABLES **********\n");

  DFSOrdering.reset();
  BackEdges.clear();
  IncomingEdges.clear();
  ReducedReachability.clear();
  ReachableBackEdges.clear();

  computeDFSOrdering(F.getEntryBlock());
  computeBackAndIncomingEdges(F);
  computeReducedReachability(F);
  computeReachableBackEdges(F);

  //DEBUG(dump(F));

  return false;
}

void LiveIRVariables::computeDFSOrdering(BasicBlock &BB) {
  if (DFSOrdering.idFor(&BB) != 0)
    return;

  DEBUG(dbgs() << "Basic block " << DFSOrdering.size() << ":\n" << BB);

  DFSOrdering.insert(&BB);
  for (succ_iterator SI = succ_begin(&BB), SE = succ_end(&BB); SI != SE; ++SI)
    computeDFSOrdering(**SI);
}

void LiveIRVariables::computeBackAndIncomingEdges(Function &F) {
  IncomingEdges.resize(F.size(), 0);

  SmallSet<BasicBlock *, 256> BlocksSeen;
  SmallSet<BasicBlock *, 256> PathToNode;
  SmallVector<BasicBlock *, 256> WorkList;
  WorkList.push_back(&F.getEntryBlock());

  while (!WorkList.empty()) {
    BasicBlock *BB = WorkList.back();
    if (BlocksSeen.count(BB)) {
      PathToNode.erase(BB);
      WorkList.pop_back();
      continue;
    }
    BlocksSeen.insert(BB);
    PathToNode.insert(BB);

    for (succ_iterator SI = succ_begin(BB),
                       SE = succ_end(BB); SI != SE; ++SI) {
      if (PathToNode.count(*SI)) {
        BackEdges.insert(std::make_pair(BB, *SI));
        continue;
      }

      ++IncomingEdges[DFSOrdering.idFor(*SI) - 1];

      if (BlocksSeen.count(*SI)) {
        continue;
      }
      WorkList.push_back(*SI);
    }
  }

#ifndef NDEBUG
  DEBUG(dbgs() << "Computed incoming edges:\n");
  for (unsigned i = 0, ie = IncomingEdges.size(); i != ie; ++i) {
    DEBUG(dbgs() << "Basic block " << i << " has " << IncomingEdges[i]);
    DEBUG(dbgs() << " incoming edges.\n");
  }

  DEBUG(dbgs() << "Back edges:\n");
  for (Function::iterator BBI = F.begin(), BBE = F.end(); BBI != BBE; ++BBI) {
    for (succ_iterator SI = succ_begin(&*BBI),
                       SE = succ_end(&*BBI); SI != SE; ++SI) {
      if (BackEdges.count(std::make_pair(&*BBI, *SI)))
        DEBUG(dbgs() << DFSOrdering.idFor(&*BBI)-1 << " -> " << DFSOrdering.idFor(*SI)-1 << "\n");
    }
  }
#endif
}

// Topologically sorts the basic blocks in the function and writes the ordering
// into the supplied unique vector. The back and incoming edges must have been
// computed first.
void LiveIRVariables::computeTopologicalOrdering(Function &F,
                                       UniqueVector<BasicBlock *> &Ordering) {
  assert(IncomingEdges.size() == F.size() &&
         "Incoming edges not computed yet!");

  SmallVector<unsigned, 256> ProcessedIncomingEdges;
  ProcessedIncomingEdges.resize(F.size(), 0);

  SmallVector<BasicBlock *, 256> WorkList;
  WorkList.push_back(&F.getEntryBlock());

  while (!WorkList.empty()) {
    BasicBlock *BB = WorkList.back();
    WorkList.pop_back();

    DEBUG(dbgs() << "Assigning topological order " << Ordering.size());
    DEBUG(dbgs() << " to basic block with DFS order ");
    DEBUG(dbgs() << (DFSOrdering.idFor(BB) - 1) << "\n");

    Ordering.insert(BB);

    for (succ_iterator SI = succ_begin(BB),
                       SE = succ_end(BB); SI != SE; ++SI) {
      if (BackEdges.count(std::make_pair(BB, *SI)))
        continue;

      unsigned DFSID = DFSOrdering.idFor(*SI) - 1;
      unsigned ProcessedEdges = ++ProcessedIncomingEdges[DFSID];
      if (ProcessedEdges == IncomingEdges[DFSID])
        WorkList.push_back(*SI);
    }
  }
}

// Computes reduced reachability. A basic block B is reduced reachable from a
// basic block A if A has a path to B that passes through no blocks that
// dominate A.
void LiveIRVariables::computeReducedReachability(Function &F) {
  // Compute a topological ordering.
  UniqueVector<BasicBlock *> TopologicalOrdering;
  computeTopologicalOrdering(F, TopologicalOrdering);

  // Initialize the reduced reachability matrix.
  ReducedReachability.resize(DFSOrdering.size());

  // Iterate over the basic blocks in reverse order, building up the reduced
  // reachability matrix.
  for (unsigned i = TopologicalOrdering.size() - 1; i != (unsigned)-1; --i) {
    BasicBlock *BB = TopologicalOrdering[i + 1];

    unsigned BBIndex = DFSOrdering.idFor(BB) - 1;
    BitVector &BV = ReducedReachability[BBIndex];
    BV.resize(DFSOrdering.size());
    BV[BBIndex] = true;

    for (succ_iterator SI = succ_begin(BB),
                       SE = succ_end(BB); SI != SE; ++SI) {
      if (TopologicalOrdering.idFor(*SI) - 1 < i)
        continue;   // Ignore back edges.

      unsigned SuccessorIndex = DFSOrdering.idFor(*SI) - 1;
      BV.set(SuccessorIndex);
      BV |= ReducedReachability[SuccessorIndex];
    }
  }

#ifndef NDEBUG
  for (unsigned i = 0, ie = ReducedReachability.size(); i != ie; ++i) {
    DEBUG(dbgs() << "Reduced reachability of " << i << ":");
    BitVector &BV = ReducedReachability[i];
    for (unsigned j = 0, je = BV.size(); j != je; ++j) {
      if (BV[j])
        DEBUG(dbgs() << " " << j);
    }
    DEBUG(dbgs() << "\n");
  }
#endif
}

void LiveIRVariables::computeReachableBackEdges(Function &F) {
  ReachableBackEdges.resize(F.size());
  for (unsigned i = 0, ie = DFSOrdering.size(); i != ie; ++i) {
    BitVector &TargetBV = ReachableBackEdges[i];
    TargetBV.resize(F.size());
    TargetBV[i] = true;

    BitVector &ReachableBV = ReducedReachability[i];
    for (unsigned s = 0, se = ReachableBV.size(); s != se; ++s) {
      // The source must be reachable.
      if (!ReachableBV[s])
        continue;

      BasicBlock *SBB = DFSOrdering[s + 1];
      for (unsigned t = 0, te = ReachableBV.size(); t != te; ++t) {
        // The target must not be reachable.
        if (ReachableBV[t])
          continue;

        // And there must be a back edge from the source to the target.
        BasicBlock *TBB = DFSOrdering[t + 1];
        if (!BackEdges.count(std::make_pair(SBB, TBB)))
          continue;

        assert(t <= i && "Theorem 3 was violated!");
        TargetBV |= ReachableBackEdges[t];
      }
    }

#ifndef NDEBUG
    DEBUG(dbgs() << "Back edge targets for " << i << ":");
    for (unsigned j = 0, je = TargetBV.size(); j != je; ++j) {
      if (TargetBV[j])
        DEBUG(dbgs() << " " << j);
    }
    DEBUG(dbgs() << "\n");
#endif
  }
}

BasicBlock &LiveIRVariables::getDefiningBlock(Value &V) {
  if (isa<Argument>(V))
    return cast<Argument>(V).getParent()->getEntryBlock();
  if (isa<Instruction>(V))
    return *cast<Instruction>(V).getParent();

  llvm_unreachable("Only arguments and instructions have definition sites!");
}

bool LiveIRVariables::isLiveIn(Value &V, BasicBlock &BB) {
  DominatorTree &DT = getAnalysis<DominatorTree>();
  BasicBlock &DefBB = getDefiningBlock(V);

  BitVector &BackEdges = ReachableBackEdges[DFSOrdering.idFor(&BB) - 1];
  for (int i = BackEdges.find_first(); i != -1; i = BackEdges.find_next(i)) {
    BasicBlock &ReachableBB = *DFSOrdering[i + 1];

    // Ignore back edge targets that leave the dominance tree of def(V) and
    // reenter it.
    if (!DT.properlyDominates(&DefBB, &ReachableBB))
      continue;

    BitVector &ReachableBlocks = ReducedReachability[i];
    for (int j = ReachableBlocks.find_first(); j != -1;
         j = ReachableBlocks.find_next(j)) {
      // FIXME: Precompute this to speed this up.
      if (V.isUsedInBasicBlock(DFSOrdering[j + 1]))
        return true;
    }
  }

  return false;
}

bool LiveIRVariables::isBackEdgeTarget(BasicBlock &BB) {
  for (pred_iterator PI = pred_begin(&BB),
                     PE = pred_end(&BB); PI != PE; ++PI) {
    if (BackEdges.count(std::make_pair(*PI, &BB)))
      return true;
  }
  return false;
}

bool LiveIRVariables::isLiveOut(Value &V, BasicBlock &BB) {
  BasicBlock &DefBB = getDefiningBlock(V);
  if (&DefBB == &BB) {
    // If the value is defined within this basic block, just look for any use
    // outside it.
    for (Value::use_iterator UI = V.use_begin(),
                             UE = V.use_end(); UI != UE; ++UI) {
      if (isa<Instruction>(*UI) && cast<Instruction>(*UI)->getParent() != &BB)
        return true;
    }
    return false;
  }

  DominatorTree &DT = getAnalysis<DominatorTree>();
  if (!DT.properlyDominates(&DefBB, &BB))
    return false;

  unsigned BBID = DFSOrdering.idFor(&BB) - 1;
  BitVector &BackEdges = ReachableBackEdges[BBID];
  bool BBIsBackEdgeTarget = isBackEdgeTarget(BB);

  for (int i = BackEdges.find_first(); i != -1; i = BackEdges.find_next(i)) {
    BasicBlock &ReachableBB = *DFSOrdering[i + 1];

    // Ignore back edge targets that leave the dominance tree of def(V) and
    // reenter it.
    if (!DT.properlyDominates(&DefBB, &ReachableBB))
      continue;

    BitVector &ReachableBlocks = ReducedReachability[i];
    for (int j = ReachableBlocks.find_first(); j != -1;
         j = ReachableBlocks.find_next(j)) {
      if ((unsigned)j == BBID && j == i && !BBIsBackEdgeTarget)
        continue;   // Skip trivial paths.
      // FIXME: Precompute this to speed this up.
      if (V.isUsedInBasicBlock(DFSOrdering[j + 1]))
        return true;
    }
  }

  return false;
}

void LiveIRVariables::dump(Function &F, bool IncludeDead) {
  for (Function::iterator BBAI = F.begin(),
                          BBAE = F.end(); BBAI != BBAE; ++BBAI) {
    for (BasicBlock::iterator II = BBAI->begin(),
                              IE = BBAI->end(); II != IE; ++II) {
      for (Function::iterator BBBI = F.begin(),
                              BBBE = F.end(); BBBI != BBBE; ++BBBI) {
        bool LiveIn = isLiveIn(*II, *BBBI), LiveOut = isLiveOut(*II, *BBBI);
        if (!LiveIn && !LiveOut && !IncludeDead)
          continue;
        dbgs() << "Value is " << (LiveIn ? "" : "NOT ") << "live-in ";
        dbgs() << (LiveIn != LiveOut ? "but " : "and ");
        dbgs() << (LiveOut ? "" : "NOT ") << "live-out at ";
        dbgs() << (DFSOrdering.idFor(&*BBBI) - 1) << ":\n" << *II << "\n";
      }
    }
  }
}

