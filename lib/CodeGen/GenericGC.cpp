//===-- GenericGC.cpp - Generic GC strategy -------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements lowering for the llvm.gc* intrinsics in a generic way.
// This is useful for simple language runtimes and for unit tests.
//
// The frametable emitter is in GenericGCPrinter.cpp.
//
//===----------------------------------------------------------------------===//

#include "llvm/CodeGen/GCs.h"
#include "llvm/CodeGen/GCStrategy.h"

using namespace llvm;

namespace {
  class GenericGC : public GCStrategy {
  public:
    GenericGC();
  };
}

static GCRegistry::Add<GenericGC>
X("generic", "generic GC");

void llvm::linkGenericGC() {}

GenericGC::GenericGC() {
  NeededSafePoints = 1 << GC::PostCall;
  UsesMetadata = true;
}

