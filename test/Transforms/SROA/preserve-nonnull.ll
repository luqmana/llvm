; RUN: opt < %s -sroa -S | FileCheck %s
;
; Make sure that SROA doesn't lose nonnull metadata
; on loads from allocas that optimized out.

target datalayout = "e-m:e-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

; CHECK-LABEL: yummy_nonnull
; CHECK: %{{.*}} = load float*, float** %0, align 8, !nonnull !0

define float* @yummy_nonnull(float**) {
entry-block:
	%buf = alloca float*

	%_0_i8 = bitcast float** %0 to i8*
	%_buf_i8 = bitcast float** %buf to i8*
	call void @llvm.memcpy.p0i8.p0i8.i64(i8* %_buf_i8, i8* %_0_i8, i64 8, i32 8, i1 false)

	%ret = load float*, float** %buf, align 8, !nonnull !0
	ret float* %ret
}

declare void @llvm.memcpy.p0i8.p0i8.i64(i8* nocapture writeonly, i8* nocapture readonly, i64, i32, i1)

!0 = !{}
