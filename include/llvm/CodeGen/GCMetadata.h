//===-- GCMetadata.h - Garbage collector metadata ---------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the GCFunctionInfo and GCModuleInfo classes, which are
// used as a communication channel from the target code generator to the target
// garbage collectors. This interface allows code generators and garbage
// collectors to be developed independently.
//
// The GCFunctionInfo class logs the data necessary to build a type accurate
// stack map. The code generator outputs:
//
//   - Safe points as specified by the GCStrategy's NeededSafePoints.
//   - Stack offsets for GC roots, as specified by calls to llvm.gcroot
//
// As a refinement, liveness analysis calculates the set of live roots at each
// safe point. Liveness analysis is not presently performed by the code
// generator, so all roots are assumed live.
//
// GCModuleInfo simply collects GCFunctionInfo instances for each Function as
// they are compiled. This accretion is necessary for collectors which must emit
// a stack map for the compilation unit as a whole. Therefore, GCFunctionInfo
// outlives the MachineFunction from which it is derived and must not refer to
// any code generator data structures.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CODEGEN_GCMETADATA_H
#define LLVM_CODEGEN_GCMETADATA_H

#include "llvm/Pass.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/Support/DebugLoc.h"
#include <utility>

namespace llvm {
  class AllocaInst;
  class AsmPrinter;
  class GCStrategy;
  class Constant;
  class MCSymbol;
  class TargetData;
  class Value;

  namespace GC {
    /// PointKind - The type of a collector-safe point.
    ///
    enum PointKind {
      Loop,    ///< Instr is a loop (backwards branch).
      Return,  ///< Instr is a return instruction.
      PreCall, ///< Instr is a call instruction.
      PostCall ///< Instr is the return address of a call.
    };
  }

  /// GCPoint - Metadata for a collector-safe point in machine code.
  ///
  struct GCPoint {
    GC::PointKind Kind; ///< The kind of the safe point.
    MCSymbol *Label;    ///< A label.
    DebugLoc Loc;

    GCPoint(GC::PointKind K, MCSymbol *L, DebugLoc DL)
        : Kind(K), Label(L), Loc(DL) {}
  };

  /// GCRootLoc - The location of an object managed by the garbage collector.
  union GCRootLoc {
    int StackOffset;    //< Offset from the stack pointer.
    int PhysReg;        //< A physical register.
  };

  /// GCRoot - Metadata for a pointer to an object managed by the garbage
  /// collector.
  struct GCRoot {
    int Num;            //< Positive integers indicate a frame index; -1
                        //  indicates a register.
    GCRootLoc Loc;      //< The location of the object.
    const Constant *Metadata;//< Metadata straight from the call to llvm.gcroot.

    inline bool isStack() const { return Num >= 0; }
    inline bool isReg() const { return !isStack(); }

    GCRoot(int N, unsigned Offset, const Constant *MD)
      : Num(N), Metadata(MD) {
      Loc.StackOffset = Offset;
    }
  };

  /// GCCalleeSavedInfo - Metadata for a callee-saved register.
  struct GCCalleeSavedInfo {
    unsigned Reg;     //< The physical register.
    int Num;          //< The frame index.
    int StackOffset;  //< Offset from the stack pointer.

    GCCalleeSavedInfo(unsigned R, int N, int SO)
      : Reg(R), Num(N), StackOffset(SO) {}
  };


  /// GCFunctionInfo - Garbage collection metadata for a single function.
  ///
  class GCFunctionInfo {
  public:
    class live_iterator {
      GCFunctionInfo &GCFI;
      unsigned PointIndex;
      unsigned RootIndex;

      inline void advance() {
        for (++RootIndex; RootIndex < GCFI.roots_size(); ++RootIndex) {
          if (GCFI.isLive(PointIndex, RootIndex))
            break;
        }
      }

    public:
      live_iterator(GCFunctionInfo &FI, unsigned PI, unsigned RI)
          : GCFI(FI), PointIndex(PI), RootIndex(RI) {
        if (RootIndex < GCFI.roots_size() &&
            !GCFI.isLive(PointIndex, RootIndex)) {
          advance();
        }
      }

      inline live_iterator& operator++() { advance(); return *this; }

      inline const GCRoot &operator*() { return GCFI.getRoot(RootIndex); }
      inline const GCRoot *operator->() { return &operator*(); }

      inline bool operator==(const live_iterator &Other) {
        return &GCFI == &Other.GCFI && PointIndex == Other.PointIndex &&
               RootIndex == Other.RootIndex;
      }
      inline bool operator!=(const live_iterator &Other) {
        return !operator==(Other);
      }
    };

    typedef std::vector<GCPoint>::iterator iterator;
    typedef std::vector<GCRoot>::iterator roots_iterator;
    typedef std::vector<GCCalleeSavedInfo>::iterator callee_saved_iterator;

  private:
    const Function &F;
    GCStrategy &S;
    uint64_t FrameSize;
    std::vector<GCRoot> Roots;
    std::vector<GCPoint> SafePoints;

    // The bit vector is the more compact representation where >3.2% of roots
    // are live per safe point (1.5% on 64-bit hosts).
    BitVector Liveness;

    // A bit vector that describes whether each root is global or local.
    // Local roots have their liveness described by the Liveness bit vector,
    // while global roots are live everywhere.
    BitVector GlobalRoots;

    // A mapping from safe point symbols to indices.
    DenseMap<MCSymbol *, unsigned> SafePointSymbols;

    // A list of callee-saved registers in this function.
    std::vector<GCCalleeSavedInfo> CSInfo;

  public:
    GCFunctionInfo(const Function &F, GCStrategy &S);
    ~GCFunctionInfo();

    /// getFunction - Return the function to which this metadata applies.
    ///
    const Function &getFunction() const { return F; }

    /// getStrategy - Return the GC strategy for the function.
    ///
    GCStrategy &getStrategy() { return S; }

    /// addGlobalRoot - Registers a root that lives on the stack and is live
    /// everywhere. Num is the stack object ID for the alloca (if the code
    /// generator is using MachineFrameInfo), while Offset is the offset in
    /// bytes from that stack object.
    void addGlobalRoot(int Num, unsigned Offset, const Constant *Metadata) {
      assert(Liveness.size() == 0 && "Can't add roots after finalization!");
      unsigned RootIndex = Roots.size();
      Roots.push_back(GCRoot(Num, Offset, Metadata));

      GlobalRoots.resize(Roots.size(), false);
      GlobalRoots[RootIndex] = true;
    }

    /// addRegRoot - Registers a root that lives in a register and returns its
    /// index. The actual physical register will be filled in after register
    /// allocation.
    unsigned addRegRoot(const Constant *Metadata) {
      assert(Liveness.size() == 0 && "Can't add roots after finalization!");
      unsigned RootIndex = Roots.size();
      Roots.push_back(GCRoot(-1, 0, Metadata));
      return RootIndex;
    }

    /// addSafePoint - Notes the existence of a safe point. Num is the ID of the
    /// label just prior to the safe point (if the code generator is using
    /// MachineModuleInfo).
    void addSafePoint(GC::PointKind Kind, MCSymbol *Label, DebugLoc DL) {
      assert(Liveness.size() == 0 &&
             "Can't add safe points after roots have been finalized!");
      unsigned PointIndex = SafePoints.size();
      SafePoints.push_back(GCPoint(Kind, Label, DL));
      SafePointSymbols[Label] = PointIndex;
    }

    /// getPoint - Returns the safe point with the given index.
    GCPoint &getPoint(unsigned PointIndex) {
      assert(PointIndex < SafePoints.size() &&
             "No point with that index exists!");
      return SafePoints[PointIndex];
    }

    /// getPointIndex - Returns the index of the safe point with the given
    /// label.
    unsigned getPointIndex(MCSymbol *Label) {
      assert(SafePointSymbols.find(Label) != SafePointSymbols.end() &&
             "No safe point with that symbol exists!");
      return SafePointSymbols[Label];
    }

    /// getRoot - Returns the root with the given index.
    inline GCRoot &getRoot(unsigned RootIndex) {
      assert(RootIndex < Roots.size() && "Invalid root index!");
      return Roots[RootIndex];
    }

    /// finalizeRoots - Creates the liveness bit vector. After this call, no
    /// more roots or safe points can be added.
    void finalizeRoots() {
      Liveness.resize(Roots.size() * SafePoints.size(), false);
      GlobalRoots.resize(Roots.size(), false);
    }

    /// isRootGlobal - Returns true if the given root is a global root or false
    /// otherwise.
    bool isRootGlobal(unsigned RootIndex) const {
      return GlobalRoots[RootIndex];
    }

    /// isLive - Returns true if the given root is live at the supplied safe
    /// point or false otherwise.
    bool isLive(unsigned PointIndex, unsigned RootIndex) const {
      assert(Liveness.size() != 0 &&
             "Liveness is not available until roots have been finalized!");
      assert(RootIndex < Roots.size() && "Invalid root index!");
      assert(PointIndex < SafePoints.size() && "Invalid safe point index!");
      return GlobalRoots[RootIndex] ||
        Liveness[PointIndex * Roots.size() + RootIndex];
    }

    /// setLive - Adjusts the liveness of the given root at the supplied safe
    /// point.
    void setLive(unsigned PointIndex, unsigned RootIndex, bool Live) {
      assert(Liveness.size() != 0 &&
             "Liveness is not available until roots have been finalized!");
      assert(!GlobalRoots[RootIndex] &&
             "Cannot adjust the liveness of a global root!");
      assert(RootIndex < Roots.size() && "Invalid root index!");
      assert(PointIndex < SafePoints.size() && "Invalid safe point index!");
      Liveness[PointIndex * Roots.size() + RootIndex] = Live;
    }

    /// spillRegRoot - Moves the given register root to the stack and assigns
    /// it the supplied frame index.
    void spillRegRoot(unsigned RootIndex, int FrameIndex) {
      assert(RootIndex < Roots.size() && "Invalid root index!");
      assert(FrameIndex >= 0 && "Invalid frame index!");
      Roots[RootIndex].Num = FrameIndex;
    }

    /// setRootLoc - Sets the final location of the given register root.
    inline void setRootLoc(unsigned RootIndex, GCRootLoc Loc) {
      assert(RootIndex < Roots.size() && "Invalid root index!");
      Roots[RootIndex].Loc = Loc;
    }

    /// findGCRootOrigin - Traces the argument to llvm.gcroot back to its
    /// origin (through any pointer casts or GEPs) and returns the original
    /// value as well as the byte offset of the pointer of interest within it.
    static std::pair<const AllocaInst *, unsigned>
    findGCRootOrigin(const TargetData *TD, const Value *V);

    /// getFrameSize/setFrameSize - Records the function's frame size.
    ///
    uint64_t getFrameSize() const { return FrameSize; }
    void setFrameSize(uint64_t S) { FrameSize = S; }

    /// addCalleeSavedReg - Records information about a callee-saved register.
    void addCalleeSavedReg(unsigned Reg, int FI, int SO) {
      CSInfo.push_back(GCCalleeSavedInfo(Reg, FI, SO));
    }

    /// begin/end - Iterators for safe points.
    ///
    iterator begin() { return SafePoints.begin(); }
    iterator end()   { return SafePoints.end();   }
    size_t size() const { return SafePoints.size(); }

    /// roots_begin/roots_end - Iterators for all roots in the function.
    ///
    roots_iterator roots_begin() { return Roots.begin(); }
    roots_iterator roots_end  () { return Roots.end();   }
    size_t roots_size() const { return Roots.size(); }

    /// live_begin/live_end - Iterators for live roots at a given safe point.
    ///
    live_iterator live_begin(unsigned PointIndex) {
      return live_iterator(*this, PointIndex, 0);
    }
    live_iterator live_end(unsigned PointIndex) {
      return live_iterator(*this, PointIndex, roots_size());
    }
    size_t live_size(unsigned PointIndex) {
      size_t Count = 0;
      for (unsigned RI = 0; RI < Roots.size(); ++RI) {
        if (isLive(PointIndex, RI))
          ++Count;
      }
      return Count;
    }

    /// callee_saved_begin/callee_saved_end - Iterators for callee-saved
    /// registers.
    callee_saved_iterator callee_saved_begin() { return CSInfo.begin(); }
    callee_saved_iterator callee_saved_end()   { return CSInfo.end(); }
    size_t callee_saved_size() { return CSInfo.size(); }
  };


  /// GCModuleInfo - Garbage collection metadata for a whole module.
  ///
  class GCModuleInfo : public ImmutablePass {
    typedef StringMap<GCStrategy*> strategy_map_type;
    typedef std::vector<GCStrategy*> list_type;
    typedef DenseMap<const Function*,GCFunctionInfo*> finfo_map_type;

    strategy_map_type StrategyMap;
    list_type StrategyList;
    finfo_map_type FInfoMap;

    GCStrategy *getOrCreateStrategy(const Module *M, const std::string &Name);

  public:
    typedef list_type::const_iterator iterator;

    static char ID;

    GCModuleInfo();
    ~GCModuleInfo();

    /// clear - Resets the pass. The metadata deleter pass calls this.
    ///
    void clear();

    /// begin/end - Iterators for used strategies.
    ///
    iterator begin() const { return StrategyList.begin(); }
    iterator end()   const { return StrategyList.end();   }

    /// get - Look up function metadata.
    ///
    GCFunctionInfo &getFunctionInfo(const Function &F);
  };

}

#endif
