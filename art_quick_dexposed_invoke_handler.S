#include "arch/arm64/asm_support_arm64.S"

/*
 * Macro that sets up the callee save frame to conform with
 * Runtime::CreateCalleeSaveMethod(kSaveAll)
 */
.macro SETUP_SAVE_ALL_CALLEE_SAVE_FRAME
    adrp xIP0, :got:_ZN3art7Runtime9instance_E
    ldr xIP0, [xIP0, #:got_lo12:_ZN3art7Runtime9instance_E]

    // Our registers aren't intermixed - just spill in order.
    ldr xIP0, [xIP0]  // xIP0 = & (art::Runtime * art::Runtime.instance_) .

    // xIP0 = (ArtMethod*) Runtime.instance_.callee_save_methods[kRefAndArgs]  .
    THIS_LOAD_REQUIRES_READ_BARRIER
    // Loads appropriate callee-save-method.
    ldr xIP0, [xIP0, RUNTIME_SAVE_ALL_CALLEE_SAVE_FRAME_OFFSET ]

    sub sp, sp, #176
    .cfi_adjust_cfa_offset 176

    // Ugly compile-time check, but we only have the preprocessor.
#if (FRAME_SIZE_SAVE_ALL_CALLEE_SAVE != 176)
#error "SAVE_ALL_CALLEE_SAVE_FRAME(ARM64) size not as expected."
#endif

    // FP callee-saves
    stp d8, d9,   [sp, #8]
    stp d10, d11, [sp, #24]
    stp d12, d13, [sp, #40]
    stp d14, d15, [sp, #56]

    // Thread register and x19 (callee-save)
    stp xSELF, x19, [sp, #72]
    .cfi_rel_offset x18, 72
    .cfi_rel_offset x19, 80

    // callee-saves
    stp x20, x21, [sp, #88]
    .cfi_rel_offset x20, 88
    .cfi_rel_offset x21, 96

    stp x22, x23, [sp, #104]
    .cfi_rel_offset x22, 104
    .cfi_rel_offset x23, 112

    stp x24, x25, [sp, #120]
    .cfi_rel_offset x24, 120
    .cfi_rel_offset x25, 128

    stp x26, x27, [sp, #136]
    .cfi_rel_offset x26, 136
    .cfi_rel_offset x27, 144

    stp x28, x29, [sp, #152]
    .cfi_rel_offset x28, 152
    .cfi_rel_offset x29, 160

    str xLR, [sp, #168]
    .cfi_rel_offset x30, 168

    // Loads appropriate callee-save-method
    str xIP0, [sp]    // Store ArtMethod* Runtime::callee_save_methods_[kRefsAndArgs]
    // Place sp in Thread::Current()->top_quick_frame.
    mov xIP0, sp
    str xIP0, [xSELF, # THREAD_TOP_QUICK_FRAME_OFFSET]
.endm

.macro SETUP_REFS_AND_ARGS_CALLEE_SAVE_FRAME_INTERNAL
    sub sp, sp, #224
    .cfi_adjust_cfa_offset 224

    // Ugly compile-time check, but we only have the preprocessor.
#if (FRAME_SIZE_REFS_AND_ARGS_CALLEE_SAVE != 224)
#error "REFS_AND_ARGS_CALLEE_SAVE_FRAME(ARM64) size not as expected."
#endif

    // FP args.
    stp d0, d1, [sp, #8]
    stp d2, d3, [sp, #24]
    stp d4, d5, [sp, #40]
    stp d6, d7, [sp, #56]

    // Core args.
    str x1, [sp, 72]
    .cfi_rel_offset x1, 72

    stp x2,  x3, [sp, #80]
    .cfi_rel_offset x2, 80
    .cfi_rel_offset x3, 88

    stp x4,  x5, [sp, #96]
    .cfi_rel_offset x4, 96
    .cfi_rel_offset x5, 104

    stp x6,  x7, [sp, #112]
    .cfi_rel_offset x6, 112
    .cfi_rel_offset x7, 120

    // Callee-saves.
    stp x19, x20, [sp, #128]
    .cfi_rel_offset x19, 128
    .cfi_rel_offset x20, 136

    stp x21, x22, [sp, #144]
    .cfi_rel_offset x21, 144
    .cfi_rel_offset x22, 152

    stp x23, x24, [sp, #160]
    .cfi_rel_offset x23, 160
    .cfi_rel_offset x24, 168

    stp x25, x26, [sp, #176]
    .cfi_rel_offset x25, 176
    .cfi_rel_offset x26, 184

    stp x27, x28, [sp, #192]
    .cfi_rel_offset x27, 192
    .cfi_rel_offset x28, 200

    // x29(callee-save) and LR
    stp x29, xLR, [sp, #208]
    .cfi_rel_offset x29, 208
    .cfi_rel_offset x30, 216

    // Save xSELF to xETR.
    mov xETR, xSELF
.endm

.macro SETUP_REFS_AND_ARGS_CALLEE_SAVE_FRAME_WITH_METHOD_IN_X0
    SETUP_REFS_AND_ARGS_CALLEE_SAVE_FRAME_INTERNAL
    str x0, [sp, #0]  // Store ArtMethod* to bottom of stack.
    // Place sp in Thread::Current()->top_quick_frame.
    mov xIP0, sp
    str xIP0, [xSELF, # THREAD_TOP_QUICK_FRAME_OFFSET]
.endm

// TODO: Probably no need to restore registers preserved by aapcs64.
.macro RESTORE_REFS_AND_ARGS_CALLEE_SAVE_FRAME
    // Restore xSELF.
    mov xSELF, xETR

    // FP args.
    ldp d0, d1, [sp, #8]
    ldp d2, d3, [sp, #24]
    ldp d4, d5, [sp, #40]
    ldp d6, d7, [sp, #56]

    // Core args.
    ldr x1, [sp, 72]
    .cfi_restore x1

    ldp x2,  x3, [sp, #80]
    .cfi_restore x2
    .cfi_restore x3

    ldp x4,  x5, [sp, #96]
    .cfi_restore x4
    .cfi_restore x5

    ldp x6,  x7, [sp, #112]
    .cfi_restore x6
    .cfi_restore x7

    // Callee-saves.
    ldp x19, x20, [sp, #128]
    .cfi_restore x19
    .cfi_restore x20

    ldp x21, x22, [sp, #144]
    .cfi_restore x21
    .cfi_restore x22

    ldp x23, x24, [sp, #160]
    .cfi_restore x23
    .cfi_restore x24

    ldp x25, x26, [sp, #176]
    .cfi_restore x25
    .cfi_restore x26

    ldp x27, x28, [sp, #192]
    .cfi_restore x27
    .cfi_restore x28

    // x29(callee-save) and LR
    ldp x29, xLR, [sp, #208]
    .cfi_restore x29
    .cfi_restore x30

    add sp, sp, #224
    .cfi_adjust_cfa_offset -224
.endm

/*
 * Macro that set calls through to artDeliverPendingExceptionFromCode, where the pending
 * exception is Thread::Current()->exception_
 */
.macro DELIVER_PENDING_EXCEPTION
    SETUP_SAVE_ALL_CALLEE_SAVE_FRAME
    mov x0, xSELF

    // Point of no return.
    b artDeliverPendingExceptionFromCode  // artDeliverPendingExceptionFromCode(Thread*)
    brk 0  // Unreached
.endm

     .extern artQuickDexposedInvokeHandler
ENTRY art_quick_dexposed_invoke_handler
    SETUP_REFS_AND_ARGS_CALLEE_SAVE_FRAME_WITH_METHOD_IN_X0
    mov     x2, xSELF                   // pass Thread::Current
    mov     x3, sp                      // pass SP
    bl      artQuickDexposedInvokeHandler  // (Method* proxy method, receiver, Thread*, SP)
    // Use xETR as xSELF might be scratched by native function above.
    ldr     x2, [xETR, THREAD_EXCEPTION_OFFSET]
    cbnz    x2, .Lexception_in_proxy    // success if no exception is pending
    RESTORE_REFS_AND_ARGS_CALLEE_SAVE_FRAME // Restore frame
    fmov    d0, x0                      // Store result in d0 in case it was float or double
    ret                                 // return on success
.Lexception_in_proxy:
    RESTORE_REFS_AND_ARGS_CALLEE_SAVE_FRAME
    DELIVER_PENDING_EXCEPTION
END art_quick_dexposed_invoke_handler
