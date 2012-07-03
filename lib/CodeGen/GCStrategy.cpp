//===-- GCStrategy.cpp - Garbage collection infrastructure -----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements target- and collector-independent garbage collection
// infrastructure.
//
// GCMachineCodeAnalysis identifies the GC safe points in the machine code.
// Roots are identified in SelectionDAGISel.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "gc-strategy"

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/CodeGen/GCStrategy.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/IntrinsicInst.h"
#include "llvm/Module.h"
#include "llvm/Analysis/Dominators.h"
#include "llvm/CodeGen/LiveIRVariables.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/Target/TargetFrameLowering.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetRegisterInfo.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

namespace llvm {
  class DominatorTree;
  class LiveIRVariables;
} // end namespace llvm

namespace {

  /// LowerIntrinsics - This pass rewrites calls to the llvm.gcread or
  /// llvm.gcwrite intrinsics, replacing them with simple loads and stores as
  /// directed by the GCStrategy. It also performs automatic root initialization
  /// and custom intrinsic lowering.
  class LowerIntrinsics : public FunctionPass {
    typedef DenseMap< Value *, std::vector<Value *> > GCRootMapType;

    DominatorTree *DT;
    LiveIRVariables *LV;

    static bool NeedsDefaultLoweringPass(const GCStrategy &C);
    static bool NeedsCustomLoweringPass(const GCStrategy &C);
    static bool CouldBecomeSafePoint(Instruction *I);
    bool PerformDefaultLowering(Function &F, GCStrategy &Coll);
    static bool InsertRootInitializers(Function &F,
                                       Instruction **Roots, unsigned Count);
    void AutomaticallyRootValue(AllocaInst &AI, Type *Ty,
                                ArrayRef<Value *> Indices);
    void InsertAutomaticGCRoots(Function &F, GCStrategy &S);
    void InsertGCRegisterRoots(Function &F, GCStrategy &S);
    bool IsGCRoot(Value *V);
    void AddGCRoot(Value &V, GCRootMapType &GCRoots);
    bool IsRootLiveAt(Value *Root, BasicBlock::iterator II,
                      GCRootMapType &GCRoots);
    void FindGCRegisterRoots(Function &F, GCRootMapType &GCRoots);

  public:
    static char ID;

    LowerIntrinsics();
    const char *getPassName() const;
    void getAnalysisUsage(AnalysisUsage &AU) const;

    bool doInitialization(Module &M);
    bool runOnFunction(Function &F);
  };

  class GCMachineCodeFixup : public MachineFunctionPass {
  public:
    static char ID;

    GCMachineCodeFixup();
    void getAnalysisUsage(AnalysisUsage &AU) const;
    bool runOnMachineFunction(MachineFunction &MF);
  };

  /// GCMachineCodeAnalysis - This is a target-independent pass over the machine
  /// function representation to identify safe points for the garbage collector
  /// in the machine code. It inserts labels at safe points and populates a
  /// GCMetadata record for each function.
  class GCMachineCodeAnalysis : public MachineFunctionPass {
    const TargetMachine *TM;
    GCFunctionInfo *FI;
    MachineModuleInfo *MMI;
    const TargetInstrInfo *TII;

    void FindSafePoints(MachineFunction &MF);
    void VisitCallPoint(MachineBasicBlock::iterator MI);
    MCSymbol *InsertLabel(MachineBasicBlock &MBB,
                          MachineBasicBlock::iterator MI,
                          DebugLoc DL) const;

    void FindStackOffsets(MachineFunction &MF);
    void FindRegisterRoots(MachineFunction &MF);
    void FindCalleeSavedRegisters(MachineFunction &MF);

  public:
    static char ID;

    GCMachineCodeAnalysis();
    void getAnalysisUsage(AnalysisUsage &AU) const;

    bool runOnMachineFunction(MachineFunction &MF);
  };

}

// -----------------------------------------------------------------------------

GCStrategy::GCStrategy() :
  NeededSafePoints(0),
  CustomReadBarriers(false),
  CustomWriteBarriers(false),
  CustomRoots(false),
  CustomSafePoints(false),
  InitRoots(true),
  UsesMetadata(false)
{}

GCStrategy::~GCStrategy() {
  for (iterator I = begin(), E = end(); I != E; ++I)
    delete *I;

  Functions.clear();
}

bool GCStrategy::initializeCustomLowering(Module &M) { return false; }

bool GCStrategy::performCustomLowering(Function &F) {
  dbgs() << "gc " << getName() << " must override performCustomLowering.\n";
  llvm_unreachable("must override performCustomLowering");
}


bool GCStrategy::findCustomSafePoints(GCFunctionInfo& FI, MachineFunction &F) {
  dbgs() << "gc " << getName() << " must override findCustomSafePoints.\n";
  llvm_unreachable(0);
}


GCFunctionInfo *GCStrategy::insertFunctionInfo(const Function &F) {
  GCFunctionInfo *FI = new GCFunctionInfo(F, *this);
  Functions.push_back(FI);
  return FI;
}

// -----------------------------------------------------------------------------

INITIALIZE_PASS_BEGIN(LowerIntrinsics, "gc-lowering", "GC Lowering",
                      false, false)
INITIALIZE_PASS_DEPENDENCY(GCModuleInfo)
INITIALIZE_PASS_END(LowerIntrinsics, "gc-lowering", "GC Lowering", false, false)

FunctionPass *llvm::createGCLoweringPass() {
  return new LowerIntrinsics();
}

char LowerIntrinsics::ID = 0;

LowerIntrinsics::LowerIntrinsics()
  : FunctionPass(ID) {
    initializeLowerIntrinsicsPass(*PassRegistry::getPassRegistry());
  }

const char *LowerIntrinsics::getPassName() const {
  return "Lower Garbage Collection Instructions";
}

void LowerIntrinsics::getAnalysisUsage(AnalysisUsage &AU) const {
  FunctionPass::getAnalysisUsage(AU);
  AU.addRequired<GCModuleInfo>();
  AU.addRequired<DominatorTree>();
  AU.addRequired<LiveIRVariables>();
}

/// doInitialization - If this module uses the GC intrinsics, find them now.
bool LowerIntrinsics::doInitialization(Module &M) {
  // FIXME: This is rather antisocial in the context of a JIT since it performs
  //        work against the entire module. But this cannot be done at
  //        runFunction time (initializeCustomLowering likely needs to change
  //        the module).
  GCModuleInfo *MI = getAnalysisIfAvailable<GCModuleInfo>();
  assert(MI && "LowerIntrinsics didn't require GCModuleInfo!?");
  for (Module::iterator I = M.begin(), E = M.end(); I != E; ++I)
    if (!I->isDeclaration() && I->hasGC())
      MI->getFunctionInfo(*I); // Instantiate the GC strategy.

  bool MadeChange = false;
  for (GCModuleInfo::iterator I = MI->begin(), E = MI->end(); I != E; ++I)
    if (NeedsCustomLoweringPass(**I))
      if ((*I)->initializeCustomLowering(M))
        MadeChange = true;

  return MadeChange;
}

bool LowerIntrinsics::InsertRootInitializers(Function &F, Instruction **Roots,
                                                          unsigned Count) {
  // Scroll past alloca instructions.
  BasicBlock::iterator IP = F.getEntryBlock().begin();
  while (isa<AllocaInst>(IP)) ++IP;

  // Search for initializers in the initial BB.
  SmallPtrSet<AllocaInst*,16> InitedRoots;
  for (; !CouldBecomeSafePoint(IP); ++IP)
    if (StoreInst *SI = dyn_cast<StoreInst>(IP))
      if (AllocaInst *AI =
          dyn_cast<AllocaInst>(SI->getOperand(1)->stripPointerCasts()))
        InitedRoots.insert(AI);

  // Add root initializers.
  bool MadeChange = false;

  for (Instruction **II = Roots, **IE = Roots + Count; II != IE; ++II) {
    // Trace back through GEPs to find the actual alloca.
    Instruction *I = *II;
    while (GetElementPtrInst *GEP = dyn_cast<GetElementPtrInst>(I))
      I = cast<Instruction>(GEP->getPointerOperand());

    AllocaInst *AI = cast<AllocaInst>(I);
    if (!InitedRoots.count(AI)) {
      Type *ElemTy = cast<PointerType>((*II)->getType())->getElementType();
      PointerType *PElemTy = cast<PointerType>(ElemTy);
      StoreInst* SI = new StoreInst(ConstantPointerNull::get(PElemTy), *II);
      SI->insertAfter(*II);
      MadeChange = true;
    }
  }

  return MadeChange;
}

bool LowerIntrinsics::NeedsDefaultLoweringPass(const GCStrategy &C) {
  // Default lowering is necessary only if read or write barriers have a default
  // action. The default for roots is no action.
  return !C.customWriteBarrier()
      || !C.customReadBarrier()
      || C.initializeRoots();
}

bool LowerIntrinsics::NeedsCustomLoweringPass(const GCStrategy &C) {
  // Custom lowering is only necessary if enabled for some action.
  return C.customWriteBarrier()
      || C.customReadBarrier()
      || C.customRoots();
}

/// CouldBecomeSafePoint - Predicate to conservatively determine whether the
/// instruction could introduce a safe point.
bool LowerIntrinsics::CouldBecomeSafePoint(Instruction *I) {
  // The natural definition of instructions which could introduce safe points
  // are:
  //
  //   - call, invoke (AfterCall, BeforeCall)
  //   - phis (Loops)
  //   - invoke, ret, unwind (Exit)
  //
  // However, instructions as seemingly inoccuous as arithmetic can become
  // libcalls upon lowering (e.g., div i64 on a 32-bit platform), so instead
  // it is necessary to take a conservative approach.

  if (isa<AllocaInst>(I) || isa<GetElementPtrInst>(I) ||
      isa<StoreInst>(I) || isa<LoadInst>(I))
    return false;

  // llvm.gcroot is safe because it doesn't do anything at runtime.
  if (CallInst *CI = dyn_cast<CallInst>(I))
    if (Function *F = CI->getCalledFunction())
      if (unsigned IID = F->getIntrinsicID())
        if (IID == Intrinsic::gcroot)
          return false;

  return true;
}

/// runOnFunction - Replace gcread/gcwrite intrinsics with loads and stores.
/// Leave gcroot intrinsics; the code generator needs to see those.
bool LowerIntrinsics::runOnFunction(Function &F) {
  // Quick exit for functions that do not use GC.
  if (!F.hasGC())
    return false;

  DT = getAnalysisIfAvailable<DominatorTree>();
  LV = getAnalysisIfAvailable<LiveIRVariables>();
  if (!DT || !LV)
    return false;

  GCFunctionInfo &FI = getAnalysis<GCModuleInfo>().getFunctionInfo(F);
  GCStrategy &S = FI.getStrategy();

  bool MadeChange = false;

  if (NeedsDefaultLoweringPass(S))
    MadeChange |= PerformDefaultLowering(F, S);

  bool UseCustomLoweringPass = NeedsCustomLoweringPass(S);
  if (UseCustomLoweringPass)
    MadeChange |= S.performCustomLowering(F);

  // Custom lowering may modify the CFG, so dominators must be recomputed.
  if (UseCustomLoweringPass) {
    if (DominatorTree *DT = getAnalysisIfAvailable<DominatorTree>())
      DT->DT->recalculate(F);
  }

  return MadeChange;
}

bool LowerIntrinsics::IsGCRoot(Value *V) {
  Type *Ty = V->getType();
  return isa<PointerType>(Ty) && cast<PointerType>(Ty)->getAddressSpace() != 0;
}

void LowerIntrinsics::AutomaticallyRootValue(AllocaInst &AI, Type *Ty,
                                             ArrayRef<Value *> Indices) {
  Module *M = AI.getParent()->getParent()->getParent();
  LLVMContext &C = M->getContext();
  Type *Int32Ty = Type::getInt32Ty(C);

  switch (Ty->getTypeID()) {
  case Type::PointerTyID: {
    if (!cast<PointerType>(Ty)->getAddressSpace() == 1)
      break;

    // Create the GEP, if necessary.
    Instruction *BaseInst = &AI;
    if (Indices.size() > 1) {
      BaseInst = GetElementPtrInst::Create(&AI, Indices);
      BaseInst->insertAfter(&AI);
    }

    // Cast the value to an i8** to make a type-compatible intrinsic call.
    Type *PtrTy = PointerType::get(Type::getInt8PtrTy(C), 0);
    Instruction *GCRootArg = new BitCastInst(BaseInst, PtrTy);
    GCRootArg->insertAfter(BaseInst);

    // Create an intrinsic call.
    Value *Args[2];
    Args[0] = GCRootArg;
    Args[1] = ConstantPointerNull::get(Type::getInt8PtrTy(C));
    Function *GCRootFn = Intrinsic::getDeclaration(M, Intrinsic::gcroot);
    CallInst *Call = CallInst::Create(GCRootFn, Args);
    Call->insertAfter(GCRootArg);
    break;
  }

  case Type::StructTyID:
  case Type::ArrayTyID:
  case Type::VectorTyID: {
    SmallVector<Value *, 8> NewIndices(Indices.begin(), Indices.end());
    NewIndices.push_back(ConstantInt::get(Int32Ty, 0));
    for (unsigned i = 0; i < Ty->getNumContainedTypes(); ++i) {
      NewIndices[NewIndices.size() - 1] = ConstantInt::get(Int32Ty, i);
      AutomaticallyRootValue(AI, Ty->getContainedType(i), NewIndices);
    }
    break;
  }

  default:
    break;
  }
}

void LowerIntrinsics::InsertAutomaticGCRoots(Function &F, GCStrategy &S) {
  if (!S.usesAutomaticRoots())
    return;

  LLVMContext &C = F.getParent()->getContext();
  for (Function::iterator BBI = F.begin(), BBE = F.end(); BBI != BBE; ++BBI) {
    for (BasicBlock::iterator II = BBI->begin(),
                              IE = BBI->end(); II != IE; ++II) {
      AllocaInst *AI = dyn_cast<AllocaInst>(&*II);
      if (!AI)
        continue;

      Value *Index = ConstantInt::get(Type::getInt32Ty(C), 0);
      ArrayRef<Value *> Indices(&Index, 1);
      AutomaticallyRootValue(*AI, AI->getAllocatedType(), Indices);
    }
  }
}

void LowerIntrinsics::AddGCRoot(Value &V, GCRootMapType &GCRoots) {
  Value *Origin = V.stripInBoundsOffsets();
  if (!GCRoots.count(Origin)) {
    std::vector<Value *> Vals;
    Vals.push_back(&V);
    GCRoots[Origin] = Vals;
  } else {
    GCRoots[Origin].push_back(&V);
  }
}

void LowerIntrinsics::FindGCRegisterRoots(Function &F,
                                          GCRootMapType &GCRoots) {
  // Start with arguments.
  for (Function::arg_iterator AI = F.arg_begin(),
                              AE = F.arg_end(); AI != AE; ++AI) {
    if (IsGCRoot(&*AI))
      AddGCRoot(*AI, GCRoots);
  }

  // Gather up values defined in basic blocks as well.
  for (Function::iterator BBI = F.begin(), BBE = F.end(); BBI != BBE; ++BBI) {
    for (BasicBlock::iterator II = BBI->begin(),
                              IE = BBI->end(); II != IE; ++II) {
      if (IsGCRoot(&*II))
        AddGCRoot(*II, GCRoots);
    }
  }
}

bool LowerIntrinsics::IsRootLiveAt(Value *Root, BasicBlock::iterator II,
                                   GCRootMapType &GCRoots) {
  if (!isa<Argument>(Root) && !isa<Instruction>(Root))
    return false;

  std::vector<Value *> &Ptrs = GCRoots[Root];
  for (std::vector<Value *>::iterator RI = Ptrs.begin(),
                                      RE = Ptrs.end(); RI != RE; ++RI) {
    // Quick bail-out for calls that aren't even in the dominance subtree
    // of the root.
    if (isa<Instruction>(*RI) &&
        !DT->dominates(cast<Instruction>(*RI), &*II))
      continue;

    // We now need to determine whether the root is live. Since our
    // liveness works on basic blocks, we need a little special handling
    // here.
    //
    // First, we check to see whether there's a use after the call site.
    // If there is, the root is clearly live.
    SmallSet<Instruction *, 16> ImmediateSuccessors;
    BasicBlock::iterator SI = II, SE = II->getParent()->end();
    for (++SI; SI != SE; ++SI)
      ImmediateSuccessors.insert(&*SI);

    bool IsUsedAfterCall = false;
    for (Value::use_iterator UI = (*RI)->use_begin(),
                             UE = (*RI)->use_end(); UI != UE; ++UI) {
      if (isa<Instruction>(*UI) &&
          ImmediateSuccessors.count(cast<Instruction>(*UI))) {
        IsUsedAfterCall = true;
        break;
      }
    }

    // If the root isn't used in this basic block, then we check to see
    // whether it's live-out of this block. If not, it's dead and we skip
    // it.
    if (!IsUsedAfterCall && !LV->isLiveOut(**RI, *II->getParent()))
      continue;

    return true;
  }

  return false;
}

void LowerIntrinsics::InsertGCRegisterRoots(Function &F, GCStrategy &S) {
  GCRootMapType GCRoots;
  FindGCRegisterRoots(F, GCRoots);

  // For each live root at a call site, insert a call to llvm.gcregroot.
  Function *GCRegRootFn = Intrinsic::getDeclaration(F.getParent(),
                                                    Intrinsic::gcregroot);
  Type *PtrTy = Type::getInt8PtrTy(F.getParent()->getContext());
  for (Function::iterator BBI = F.begin(), BBE = F.end(); BBI != BBE; ++BBI) {
    for (BasicBlock::iterator II = BBI->begin(),
                              IE = BBI->end(); II != IE; ++II) {
      if (!isa<CallInst>(&*II) && !isa<InvokeInst>(&*II))
        continue;

      // Skip intrinsic calls. This prevents infinite loops.
      if (isa<CallInst>(&*II)) {
        Function *Fn = cast<CallInst>(&*II)->getCalledFunction();
        if (Fn && Fn->isIntrinsic())
          continue;
      }

      // Find all the live GC roots, and create gcregroot intrinsics for them.
      SmallVector<Instruction *, 8> GeneratedInsts;
      for (GCRootMapType::iterator RI = GCRoots.begin(),
                                   RE = GCRoots.end(); RI != RE; ++RI) {
        Value *Origin = RI->first;
        if (!IsRootLiveAt(Origin, II, GCRoots))
          continue;

        // Ok, we need to root this value. First, cast the value to an i8* to
        // make a type-compatible intrinsic call.
        Instruction *GCRegRootArg = new BitCastInst(Origin, PtrTy);
        GeneratedInsts.push_back(GCRegRootArg);

        // Create the intrinsic call.
        Value *Args[1];
        Args[0] = GCRegRootArg;
        GeneratedInsts.push_back(CallInst::Create(GCRegRootFn, Args));
      }

      // Insert the gcregroot intrinsic calls after the call.
      if (isa<CallInst>(&*II)) {
        for (SmallVector<Instruction *, 8>::reverse_iterator CII =
             GeneratedInsts.rbegin(), CIE = GeneratedInsts.rend();
             CII != CIE; ++CII) {
          (*CII)->insertAfter(&*II);
        }
      } else {  // Invoke instruction
        BasicBlock *NormalDest = cast<InvokeInst>(&*II)->getNormalDest();
        Instruction &InsertPt = NormalDest->front();
        for (SmallVector<Instruction *, 8>::iterator CII =
             GeneratedInsts.begin(), CIE = GeneratedInsts.end();
             CII != CIE; ++CII) {
          (*CII)->insertBefore(&InsertPt);
        }
      }
    }
  }
}

bool LowerIntrinsics::PerformDefaultLowering(Function &F, GCStrategy &S) {
  InsertAutomaticGCRoots(F, S);
  InsertGCRegisterRoots(F, S);

  bool LowerWr = !S.customWriteBarrier();
  bool LowerRd = !S.customReadBarrier();
  bool InitRoots = S.initializeRoots();

  SmallVector<Instruction *, 32> Roots;

  bool MadeChange = false;
  for (Function::iterator BB = F.begin(), BE = F.end(); BB != BE; ++BB) {
    for (BasicBlock::iterator II = BB->begin(), IE = BB->end(); II != IE;) {
      if (IntrinsicInst *CI = dyn_cast<IntrinsicInst>(II++)) {
        Function *F = CI->getCalledFunction();
        switch (F->getIntrinsicID()) {
        case Intrinsic::gcwrite:
          if (LowerWr) {
            // Replace a write barrier with a simple store.
            Value *St = new StoreInst(CI->getArgOperand(0),
                                      CI->getArgOperand(2), CI);
            CI->replaceAllUsesWith(St);
            CI->eraseFromParent();
          }
          break;
        case Intrinsic::gcread:
          if (LowerRd) {
            // Replace a read barrier with a simple load.
            Value *Ld = new LoadInst(CI->getArgOperand(1), "", CI);
            Ld->takeName(CI);
            CI->replaceAllUsesWith(Ld);
            CI->eraseFromParent();
          }
          break;
        case Intrinsic::gcroot:
          if (InitRoots) {
            // Initialize the GC root, but do not delete the intrinsic. The
            // backend needs the intrinsic to flag the stack slot.
            Value *V = CI->getArgOperand(0)->stripPointerCastsOnly();
            Roots.push_back(cast<Instruction>(V));
          }
          break;
        default:
          continue;
        }

        MadeChange = true;
      }
    }
  }

  if (Roots.size())
    MadeChange |= InsertRootInitializers(F, Roots.begin(), Roots.size());

  return MadeChange;
}

// -----------------------------------------------------------------------------

char GCMachineCodeFixup::ID = 0;
char &llvm::GCMachineCodeFixupID = GCMachineCodeFixup::ID;

INITIALIZE_PASS(GCMachineCodeFixup, "gc-fixup",
                "Fix Machine Code for Garbage Collection", false, false)

GCMachineCodeFixup::GCMachineCodeFixup()
  : MachineFunctionPass(ID) {}

void GCMachineCodeFixup::getAnalysisUsage(AnalysisUsage &AU) const {
  MachineFunctionPass::getAnalysisUsage(AU);
  AU.setPreservesAll();
  AU.addRequired<GCModuleInfo>();
}

bool GCMachineCodeFixup::runOnMachineFunction(MachineFunction &MF) {
  // Quick exit for functions that do not use GC.
  if (!MF.getFunction()->hasGC())
    return false;

  const TargetMachine &TM = MF.getTarget();
  const TargetInstrInfo *TII = TM.getInstrInfo();
  GCModuleInfo &GMI = getAnalysis<GCModuleInfo>();
  GCFunctionInfo &GCFI = GMI.getFunctionInfo(*MF.getFunction());

  for (MachineFunction::iterator MBBI = MF.begin(),
                                 MBBE = MF.end(); MBBI != MBBE; ++MBBI) {
    for (MachineBasicBlock::iterator MII = MBBI->begin(),
                                     MIE = MBBI->end(); MII != MIE;) {
      if (!MII->isGCRegRoot() || !MII->getOperand(0).isReg()) {
        ++MII;
        continue;
      }

      // Trace the register back to its location at the site of the call (either
      // a physical reg or a frame index).
      bool TracingReg = true;
      unsigned TracedReg = MII->getOperand(0).getReg();
      int FrameIndex;

      MachineBasicBlock::iterator PrevII = MII;
      for (--PrevII;; --PrevII) {
        if (PrevII->isGCRegRoot() && PrevII->getOperand(0).isReg())
          break;
        if (PrevII->isCall())
          break;

        int FI;

        // Trace back through register reloads.
        unsigned Reg =
          TM.getInstrInfo()->isLoadFromStackSlotPostFE(&*PrevII, FI);
        if (Reg) {
          // This is a reload. If we're tracing this register, start tracing the
          // frame index instead.
          if (TracingReg && TracedReg == Reg) {
            TracingReg = false;
            FrameIndex = FI;
          }
          continue;
        }

        // Trace back through spills.
        if (TM.getInstrInfo()->isStoreToStackSlotPostFE(&*PrevII, FI))
          continue;

        // Trace back through register-to-register copies.
        if (PrevII->isCopy()) {
          if (TracingReg && TracedReg == PrevII->getOperand(0).getReg())
            TracedReg = PrevII->getOperand(1).getReg();
          continue;
        }

        // Trace back through non-register GC_REG_ROOT instructions.
        if (PrevII->isGCRegRoot() && !PrevII->getOperand(0).isReg())
          continue;

        DEBUG(dbgs() << "Bad instruction: " << *PrevII);
        llvm_unreachable("GC_REG_ROOT found in an unexpected location!");
      }

      // Now we've reached either a call or another GC_REG_ROOT instruction.
      // Move the GC_REG_ROOT instruction we're considering to the right place,
      // and rewrite it if necessary.
      //
      // Also, tell the GCFunctionInfo about the frame index, since this is
      // our only chance -- the frame indices will be deleted by the time
      // GCMachineCodeAnalysis runs.
      ++PrevII;
      unsigned RootIndex = MII->getOperand(1).getImm();
      MachineInstr *NewMI;
      if (TracingReg) {
        MachineInstrBuilder MIB = BuildMI(MF, MII->getDebugLoc(),
                                          TII->get(TargetOpcode::GC_REG_ROOT));
        MIB.addReg(TracedReg).addImm(RootIndex);
        NewMI = MIB;
      } else {
        NewMI = TII->emitFrameIndexGCRegRoot(MF, FrameIndex, RootIndex,
                                             MII->getDebugLoc());
        GCFI.spillRegRoot(RootIndex, FrameIndex);
      }

      MBBI->insert(PrevII, NewMI);

      MachineBasicBlock::iterator NextII = MII;
      ++NextII;
      MII->eraseFromParent();
      MII = NextII;
    }
  }

  return true;
}

// -----------------------------------------------------------------------------

char GCMachineCodeAnalysis::ID = 0;
char &llvm::GCMachineCodeAnalysisID = GCMachineCodeAnalysis::ID;

INITIALIZE_PASS(GCMachineCodeAnalysis, "gc-analysis",
                "Analyze Machine Code For Garbage Collection", false, false)

GCMachineCodeAnalysis::GCMachineCodeAnalysis()
  : MachineFunctionPass(ID) {}

void GCMachineCodeAnalysis::getAnalysisUsage(AnalysisUsage &AU) const {
  MachineFunctionPass::getAnalysisUsage(AU);
  AU.setPreservesAll();
  AU.addRequired<MachineModuleInfo>();
  AU.addRequired<GCModuleInfo>();
}

MCSymbol *GCMachineCodeAnalysis::InsertLabel(MachineBasicBlock &MBB,
                                             MachineBasicBlock::iterator MI,
                                             DebugLoc DL) const {
  MCSymbol *Label = MBB.getParent()->getContext().CreateTempSymbol();
  BuildMI(MBB, MI, DL, TII->get(TargetOpcode::GC_LABEL)).addSym(Label);
  return Label;
}

void GCMachineCodeAnalysis::VisitCallPoint(MachineBasicBlock::iterator CI) {
  // Find the return address (next instruction), too, so as to bracket the call
  // instruction.
  MachineBasicBlock::iterator RAI = CI;
  ++RAI;

  if (FI->getStrategy().needsSafePoint(GC::PreCall)) {
    MCSymbol* Label = InsertLabel(*CI->getParent(), CI, CI->getDebugLoc());
    FI->addSafePoint(GC::PreCall, Label, CI->getDebugLoc());
  }

  if (FI->getStrategy().needsSafePoint(GC::PostCall)) {
    MCSymbol* Label = InsertLabel(*CI->getParent(), RAI, CI->getDebugLoc());
    FI->addSafePoint(GC::PostCall, Label, CI->getDebugLoc());
  }
}

void GCMachineCodeAnalysis::FindSafePoints(MachineFunction &MF) {
  for (MachineFunction::iterator BBI = MF.begin(),
                                 BBE = MF.end(); BBI != BBE; ++BBI)
    for (MachineBasicBlock::iterator MI = BBI->begin(),
                                     ME = BBI->end(); MI != ME; ++MI)
      if (MI->isCall())
        VisitCallPoint(MI);
}

void GCMachineCodeAnalysis::FindStackOffsets(MachineFunction &MF) {
  const TargetFrameLowering *TFI = TM->getFrameLowering();
  assert(TFI && "TargetRegisterInfo not available!");

  for (unsigned i = 0; i < FI->roots_size(); ++i) {
    GCRoot &RI = FI->getRoot(i);
    if (RI.isStack())
      RI.Loc.StackOffset += TFI->getFrameIndexOffset(MF, RI.Num);
  }
}

void GCMachineCodeAnalysis::FindRegisterRoots(MachineFunction &MF) {
  const TargetFrameLowering *TFI = TM->getFrameLowering();
  assert(TFI && "TargetRegisterInfo not available!");

  unsigned PointIndex = 0;
  for (MachineFunction::iterator BBI = MF.begin(),
                                 BBE = MF.end(); BBI != BBE; ++BBI) {
    for (MachineBasicBlock::iterator MI = BBI->begin(),
                                     ME = BBI->end(); MI != ME; ++MI) {
      if (MI->isGCLabel())
        PointIndex = FI->getPointIndex(MI->getOperand(0).getMCSymbol());
      if (!MI->isGCRegRoot())
        continue;

      unsigned RootIndex = MI->getOperand(MI->getNumOperands() - 1).getImm();
      if (FI->isRootGlobal(RootIndex))
        continue;

      FI->setLive(PointIndex, RootIndex, true);

      if (!FI->getRoot(RootIndex).isReg())
        continue;
      GCRootLoc Loc;
      Loc.PhysReg = MI->getOperand(0).getReg();
      FI->setRootLoc(RootIndex, Loc);
    }
  }
}

void GCMachineCodeAnalysis::FindCalleeSavedRegisters(MachineFunction &MF) {
  const std::vector<CalleeSavedInfo> CSInfo =
      MF.getFrameInfo()->getCalleeSavedInfo();
  const TargetFrameLowering *TFI = TM->getFrameLowering();
  for (std::vector<CalleeSavedInfo>::const_iterator CSIB = CSInfo.begin(),
                                                    CSIE = CSInfo.end();
                                                    CSIB != CSIE; ++CSIB) {
    int StackOffset = TFI->getFrameIndexOffset(MF, CSIB->getFrameIdx());
    FI->addCalleeSavedReg(CSIB->getReg(), CSIB->getFrameIdx(), StackOffset);
  }
}

bool GCMachineCodeAnalysis::runOnMachineFunction(MachineFunction &MF) {
  // Quick exit for functions that do not use GC.
  if (!MF.getFunction()->hasGC())
    return false;

  FI = &getAnalysis<GCModuleInfo>().getFunctionInfo(*MF.getFunction());
  if (!FI->getStrategy().needsSafePoints())
    return false;

  TM = &MF.getTarget();
  MMI = &getAnalysis<MachineModuleInfo>();
  TII = TM->getInstrInfo();

  // Find the size of the stack frame.
  FI->setFrameSize(MF.getFrameInfo()->getStackSize());

  // Find all safe points.
  if (FI->getStrategy().customSafePoints()) {
    FI->getStrategy().findCustomSafePoints(*FI, MF);
  } else {
    FindSafePoints(MF);
  }

  // Create the liveness vector.
  FI->finalizeRoots();

  // Populate the list of callee-saved registers.
  FindCalleeSavedRegisters(MF);

  // Find the register locations for all register roots, and compute their
  // liveness.
  FindRegisterRoots(MF);

  // Find the stack offsets for all stack roots, and mark them live everywhere
  // (for now).
  FindStackOffsets(MF);

  return false;
}
