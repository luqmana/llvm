//===-- GenericGCPrinter.cpp - Generic frametable emitter -----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements printing the assembly code for a generic frametable.
//
//===----------------------------------------------------------------------===//

#include "llvm/Constant.h"
#include "llvm/Constants.h"
#include "llvm/GlobalVariable.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/GCs.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/GCMetadataPrinter.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/Target/Mangler.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Target/TargetLoweringObjectFile.h"
#include "llvm/Target/TargetMachine.h"

using namespace llvm;

namespace {
  class GenericGCMetadataPrinter : public GCMetadataPrinter {
    void writeFunctionMetadata(AsmPrinter &AP, GCFunctionInfo &FI);
  public:
    void beginAssembly(AsmPrinter &AP);
    void finishAssembly(AsmPrinter &AP);
  };
}

static GCMetadataPrinterRegistry::Add<GenericGCMetadataPrinter>
Y("generic", "generic collector");

void llvm::linkGenericGCPrinter() {}

void GenericGCMetadataPrinter::writeFunctionMetadata(AsmPrinter &AP,
                                                     GCFunctionInfo &FI) {
  unsigned PtrSize = AP.TM.getTargetData()->getPointerSize();

  StringRef Name = FI.getFunction().getName();
  AP.OutStreamer.AddComment("GC metadata for " + Name);

  // Emit the symbol.
  SmallString<128> FnMetadataName;
  AP.Mang->getNameWithPrefix(FnMetadataName, "_gc_metadata_" + Name);
  MCSymbol *FnMetadataSym = AP.OutContext.GetOrCreateSymbol(FnMetadataName);
  AP.OutStreamer.EmitSymbolAttribute(FnMetadataSym, MCSA_Global);
  AP.OutStreamer.EmitLabel(FnMetadataSym);

  // Emit the number of safe points.
  unsigned PointCount = FI.size();
  AP.EmitInt32(PointCount);

  // Emit the safe point addresses.
  SmallVector<MCSymbol *, 8> Symbols;
  for (GCFunctionInfo::iterator PI = FI.begin(),
                                PE = FI.end(); PI != PE; ++PI) {
    MCSymbol *Sym = AP.OutContext.CreateTempSymbol();
    AP.OutStreamer.EmitSymbolValue(PI->Label, PtrSize);
    AP.OutStreamer.EmitSymbolValue(Sym, PtrSize);
    Symbols.push_back(Sym);
  }

  // Emit each safe point.
  for (unsigned i = 0; i < PointCount; ++i) {
    AP.OutStreamer.EmitLabel(Symbols[i]);

    unsigned RegRootCount = 0, StackRootCount = 0;
    for (GCFunctionInfo::live_iterator LI = FI.live_begin(i),
                                       LE = FI.live_end(i); LI != LE; ++LI) {
      if (LI->isReg())
        ++RegRootCount;
      else
        ++StackRootCount;
    }

    AP.EmitInt32(StackRootCount);
    AP.EmitInt32(RegRootCount);

    // Write out the locations of all stack roots.
    for (GCFunctionInfo::live_iterator LI = FI.live_begin(i),
                                       LE = FI.live_end(i); LI != LE; ++LI) {
      if (!LI->isReg())
        AP.EmitInt32(LI->Loc.StackOffset);
    }

    // Write out the locations of all register roots.
    for (GCFunctionInfo::live_iterator LI = FI.live_begin(i),
                                       LE = FI.live_end(i); LI != LE; ++LI) {
      if (LI->isReg())
        AP.EmitInt8(LI->Loc.PhysReg);
    }

    // Write out the address spaces for all roots.
    for (GCFunctionInfo::live_iterator LI = FI.live_begin(i),
                                       LE = FI.live_end(i); LI != LE; ++LI) {
      if (LI->Metadata->isNullValue()) {
        AP.EmitInt8(0);
        continue;
      }

      assert(isa<GlobalVariable>(LI->Metadata) &&
             "Generic GC printer requires global variable metadata!");
      const Constant *Init =
          cast<GlobalVariable>(LI->Metadata)->getInitializer();
      assert(isa<ConstantInt>(Init) &&
             "Generic GC printer requires metadata to be an integer!");
      AP.EmitInt8(cast<ConstantInt>(Init)->getZExtValue());
    }
  }
}

void GenericGCMetadataPrinter::beginAssembly(AsmPrinter &AP) {
}

void GenericGCMetadataPrinter::finishAssembly(AsmPrinter &AP) {
  AP.OutStreamer.SwitchSection(AP.getObjFileLowering().getDataSection());
  for (iterator II = begin(), IE = end(); II != IE; ++II)
    writeFunctionMetadata(AP, **II);
}
