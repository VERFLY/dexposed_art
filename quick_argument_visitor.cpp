/*
 * Original work Copyright (c) 2005-2008, The Android Open Source Project
 * Modified work Copyright (c) 2013, rovo89 and Tungstwenty
 * Modified work Copyright (c) 2015, Alibaba Mobile Infrastructure (Android) Team
 * Modified work Copyright (c) 2016, verfly
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "callee_save_frame.h"
#include "entrypoints/entrypoint_utils-inl.h"

namespace art {

    // Visits the arguments as saved to the stack by a Runtime::kRefAndArgs callee save frame.
    class QuickArgumentVisitor {
        // Number of bytes for each out register in the caller method's frame.
        static constexpr size_t kBytesStackArgLocation = 4;
        // Frame size in bytes of a callee-save frame for RefsAndArgs.
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_FrameSize =
            GetCalleeSaveFrameSize(kRuntimeISA, Runtime::kRefsAndArgs);
#if defined(__arm__)
        // The callee save frame is pointed to by SP.
        // | argN       |  |
        // | ...        |  |
        // | arg4       |  |
        // | arg3 spill |  |  Caller's frame
        // | arg2 spill |  |
        // | arg1 spill |  |
        // | Method*    | ---
        // | LR         |
        // | ...        |    4x6 bytes callee saves
        // | R3         |
        // | R2         |
        // | R1         |
        // | S15        |
        // | :          |
        // | S0         |
        // |            |    4x2 bytes padding
        // | Method*    |  <- sp
        static constexpr bool kSplitPairAcrossRegisterAndStack = kArm32QuickCodeUseSoftFloat;
        static constexpr bool kAlignPairRegister = !kArm32QuickCodeUseSoftFloat;
        static constexpr bool kQuickSoftFloatAbi = kArm32QuickCodeUseSoftFloat;
        static constexpr bool kQuickDoubleRegAlignedFloatBackFilled = !kArm32QuickCodeUseSoftFloat;
        static constexpr size_t kNumQuickGprArgs = 3;
        static constexpr size_t kNumQuickFprArgs = kArm32QuickCodeUseSoftFloat ? 0 : 16;
        static constexpr bool kGprFprLockstep = false;
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_Fpr1Offset =
            arm::ArmCalleeSaveFpr1Offset(Runtime::kRefsAndArgs);  // Offset of first FPR arg.
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_Gpr1Offset =
            arm::ArmCalleeSaveGpr1Offset(Runtime::kRefsAndArgs);  // Offset of first GPR arg.
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_LrOffset =
            arm::ArmCalleeSaveLrOffset(Runtime::kRefsAndArgs);  // Offset of return address.
        static size_t GprIndexToGprOffset(uint32_t gpr_index) {
            return gpr_index * GetBytesPerGprSpillLocation(kRuntimeISA);
        }
#elif defined(__aarch64__)
        // The callee save frame is pointed to by SP.
        // | argN       |  |
        // | ...        |  |
        // | arg4       |  |
        // | arg3 spill |  |  Caller's frame
        // | arg2 spill |  |
        // | arg1 spill |  |
        // | Method*    | ---
        // | LR         |
        // | X29        |
        // |  :         |
        // | X19        |
        // | X7         |
        // | :          |
        // | X1         |
        // | D7         |
        // |  :         |
        // | D0         |
        // |            |    padding
        // | Method*    |  <- sp
        static constexpr bool kSplitPairAcrossRegisterAndStack = false;
        static constexpr bool kAlignPairRegister = false;
        static constexpr bool kQuickSoftFloatAbi = false;  // This is a hard float ABI.
        static constexpr bool kQuickDoubleRegAlignedFloatBackFilled = false;
        static constexpr size_t kNumQuickGprArgs = 7;  // 7 arguments passed in GPRs.
        static constexpr size_t kNumQuickFprArgs = 8;  // 8 arguments passed in FPRs.
        static constexpr bool kGprFprLockstep = false;
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_Fpr1Offset =
            arm64::Arm64CalleeSaveFpr1Offset(Runtime::kRefsAndArgs);  // Offset of first FPR arg.
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_Gpr1Offset =
            arm64::Arm64CalleeSaveGpr1Offset(Runtime::kRefsAndArgs);  // Offset of first GPR arg.
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_LrOffset =
            arm64::Arm64CalleeSaveLrOffset(Runtime::kRefsAndArgs);  // Offset of return address.
        static size_t GprIndexToGprOffset(uint32_t gpr_index) {
            return gpr_index * GetBytesPerGprSpillLocation(kRuntimeISA);
        }
#elif defined(__mips__) && !defined(__LP64__)
        // The callee save frame is pointed to by SP.
        // | argN       |  |
        // | ...        |  |
        // | arg4       |  |
        // | arg3 spill |  |  Caller's frame
        // | arg2 spill |  |
        // | arg1 spill |  |
        // | Method*    | ---
        // | RA         |
        // | ...        |    callee saves
        // | A3         |    arg3
        // | A2         |    arg2
        // | A1         |    arg1
        // | A0/Method* |  <- sp
        static constexpr bool kSplitPairAcrossRegisterAndStack = true;
        static constexpr bool kAlignPairRegister = false;
        static constexpr bool kQuickSoftFloatAbi = true;  // This is a soft float ABI.
        static constexpr bool kQuickDoubleRegAlignedFloatBackFilled = false;
        static constexpr size_t kNumQuickGprArgs = 3;  // 3 arguments passed in GPRs.
        static constexpr size_t kNumQuickFprArgs = 0;  // 0 arguments passed in FPRs.
        static constexpr bool kGprFprLockstep = false;
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_Fpr1Offset = 0;  // Offset of first FPR arg.
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_Gpr1Offset = 16;  // Offset of first GPR arg.
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_LrOffset = 60;  // Offset of return address.
        static size_t GprIndexToGprOffset(uint32_t gpr_index) {
            return gpr_index * GetBytesPerGprSpillLocation(kRuntimeISA);
        }
#elif defined(__mips__) && defined(__LP64__)
        // The callee save frame is pointed to by SP.
        // | argN       |  |
        // | ...        |  |
        // | arg4       |  |
        // | arg3 spill |  |  Caller's frame
        // | arg2 spill |  |
        // | arg1 spill |  |
        // | Method*    | ---
        // | RA         |
        // | ...        |    callee saves
        // | F7         |    f_arg7
        // | F6         |    f_arg6
        // | F5         |    f_arg5
        // | F4         |    f_arg4
        // | F3         |    f_arg3
        // | F2         |    f_arg2
        // | F1         |    f_arg1
        // | F0         |    f_arg0
        // | A7         |    arg7
        // | A6         |    arg6
        // | A5         |    arg5
        // | A4         |    arg4
        // | A3         |    arg3
        // | A2         |    arg2
        // | A1         |    arg1
        // |            |    padding
        // | A0/Method* |  <- sp
        // NOTE: for Mip64, when A0 is skipped, F0 is also skipped.
        static constexpr bool kSplitPairAcrossRegisterAndStack = false;
        static constexpr bool kAlignPairRegister = false;
        static constexpr bool kQuickSoftFloatAbi = false;
        static constexpr bool kQuickDoubleRegAlignedFloatBackFilled = false;
        // These values are set to zeros because GPR and FPR register
        // assignments for Mips64 are interleaved, which the current VisitArguments()
        // function does not support.
        static constexpr size_t kNumQuickGprArgs = 7;  // 7 arguments passed in GPRs.
        static constexpr size_t kNumQuickFprArgs = 7;  // 7 arguments passed in FPRs.
        static constexpr bool kGprFprLockstep = true;

        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_Fpr1Offset = 24;  // Offset of first FPR arg (F1).
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_Gpr1Offset = 80;  // Offset of first GPR arg (A1).
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_LrOffset = 200;  // Offset of return address.
        static size_t GprIndexToGprOffset(uint32_t gpr_index) {
            return gpr_index * GetBytesPerGprSpillLocation(kRuntimeISA);
        }
#elif defined(__i386__)
        // The callee save frame is pointed to by SP.
        // | argN        |  |
        // | ...         |  |
        // | arg4        |  |
        // | arg3 spill  |  |  Caller's frame
        // | arg2 spill  |  |
        // | arg1 spill  |  |
        // | Method*     | ---
        // | Return      |
        // | EBP,ESI,EDI |    callee saves
        // | EBX         |    arg3
        // | EDX         |    arg2
        // | ECX         |    arg1
        // | XMM3        |    float arg 4
        // | XMM2        |    float arg 3
        // | XMM1        |    float arg 2
        // | XMM0        |    float arg 1
        // | EAX/Method* |  <- sp
        static constexpr bool kSplitPairAcrossRegisterAndStack = false;
        static constexpr bool kAlignPairRegister = false;
        static constexpr bool kQuickSoftFloatAbi = false;  // This is a hard float ABI.
        static constexpr bool kQuickDoubleRegAlignedFloatBackFilled = false;
        static constexpr size_t kNumQuickGprArgs = 3;  // 3 arguments passed in GPRs.
        static constexpr size_t kNumQuickFprArgs = 4;  // 4 arguments passed in FPRs.
        static constexpr bool kGprFprLockstep = false;
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_Fpr1Offset = 4;  // Offset of first FPR arg.
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_Gpr1Offset = 4 + 4*8;  // Offset of first GPR arg.
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_LrOffset = 28 + 4*8;  // Offset of return address.
        static size_t GprIndexToGprOffset(uint32_t gpr_index) {
            return gpr_index * GetBytesPerGprSpillLocation(kRuntimeISA);
        }
#elif defined(__x86_64__)
        // The callee save frame is pointed to by SP.
        // | argN            |  |
        // | ...             |  |
        // | reg. arg spills |  |  Caller's frame
        // | Method*         | ---
        // | Return          |
        // | R15             |    callee save
        // | R14             |    callee save
        // | R13             |    callee save
        // | R12             |    callee save
        // | R9              |    arg5
        // | R8              |    arg4
        // | RSI/R6          |    arg1
        // | RBP/R5          |    callee save
        // | RBX/R3          |    callee save
        // | RDX/R2          |    arg2
        // | RCX/R1          |    arg3
        // | XMM7            |    float arg 8
        // | XMM6            |    float arg 7
        // | XMM5            |    float arg 6
        // | XMM4            |    float arg 5
        // | XMM3            |    float arg 4
        // | XMM2            |    float arg 3
        // | XMM1            |    float arg 2
        // | XMM0            |    float arg 1
        // | Padding         |
        // | RDI/Method*     |  <- sp
        static constexpr bool kSplitPairAcrossRegisterAndStack = false;
        static constexpr bool kAlignPairRegister = false;
        static constexpr bool kQuickSoftFloatAbi = false;  // This is a hard float ABI.
        static constexpr bool kQuickDoubleRegAlignedFloatBackFilled = false;
        static constexpr size_t kNumQuickGprArgs = 5;  // 5 arguments passed in GPRs.
        static constexpr size_t kNumQuickFprArgs = 8;  // 8 arguments passed in FPRs.
        static constexpr bool kGprFprLockstep = false;
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_Fpr1Offset = 16;  // Offset of first FPR arg.
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_Gpr1Offset = 80 + 4*8;  // Offset of first GPR arg.
        static constexpr size_t kQuickCalleeSaveFrame_RefAndArgs_LrOffset = 168 + 4*8;  // Offset of return address.
        static size_t GprIndexToGprOffset(uint32_t gpr_index) {
            switch (gpr_index) {
                case 0: return (4 * GetBytesPerGprSpillLocation(kRuntimeISA));
                case 1: return (1 * GetBytesPerGprSpillLocation(kRuntimeISA));
                case 2: return (0 * GetBytesPerGprSpillLocation(kRuntimeISA));
                case 3: return (5 * GetBytesPerGprSpillLocation(kRuntimeISA));
                case 4: return (6 * GetBytesPerGprSpillLocation(kRuntimeISA));
                default:
                        LOG(FATAL) << "Unexpected GPR index: " << gpr_index;
                        return 0;
            }
        }
#else
#error "Unsupported architecture"
#endif

        public:
        // Special handling for proxy methods. Proxy methods are instance methods so the
        // 'this' object is the 1st argument. They also have the same frame layout as the
        // kRefAndArgs runtime method. Since 'this' is a reference, it is located in the
        // 1st GPR.
        static mirror::Object* GetProxyThisObject(ArtMethod** sp)
            SHARED_LOCKS_REQUIRED(Locks::mutator_lock_) {
                CHECK((*sp)->IsProxyMethod());
                CHECK_EQ(kQuickCalleeSaveFrame_RefAndArgs_FrameSize, (*sp)->GetFrameSizeInBytes());
                CHECK_GT(kNumQuickGprArgs, 0u);
                constexpr uint32_t kThisGprIndex = 0u;  // 'this' is in the 1st GPR.
                size_t this_arg_offset = kQuickCalleeSaveFrame_RefAndArgs_Gpr1Offset +
                    GprIndexToGprOffset(kThisGprIndex);
                uint8_t* this_arg_address = reinterpret_cast<uint8_t*>(sp) + this_arg_offset;
                return reinterpret_cast<StackReference<mirror::Object>*>(this_arg_address)->AsMirrorPtr();
            }

        static ArtMethod* GetCallingMethod(ArtMethod** sp)
            SHARED_LOCKS_REQUIRED(Locks::mutator_lock_) {
                DCHECK((*sp)->IsCalleeSaveMethod());
                uint8_t* previous_sp = reinterpret_cast<uint8_t*>(sp) +
                    kQuickCalleeSaveFrame_RefAndArgs_FrameSize;
                return *reinterpret_cast<ArtMethod**>(previous_sp);
            }

        // For the given quick ref and args quick frame, return the caller's PC.
        static uintptr_t GetCallingPc(ArtMethod** sp) SHARED_LOCKS_REQUIRED(Locks::mutator_lock_) {
            DCHECK((*sp)->IsCalleeSaveMethod());
            uint8_t* lr = reinterpret_cast<uint8_t*>(sp) + kQuickCalleeSaveFrame_RefAndArgs_LrOffset;
            return *reinterpret_cast<uintptr_t*>(lr);
        }

        QuickArgumentVisitor(ArtMethod** sp, bool is_static, const char* shorty,
                uint32_t shorty_len) SHARED_LOCKS_REQUIRED(Locks::mutator_lock_) :
            is_static_(is_static), shorty_(shorty), shorty_len_(shorty_len),
            gpr_args_(reinterpret_cast<uint8_t*>(sp) + kQuickCalleeSaveFrame_RefAndArgs_Gpr1Offset),
            fpr_args_(reinterpret_cast<uint8_t*>(sp) + kQuickCalleeSaveFrame_RefAndArgs_Fpr1Offset),
            stack_args_(reinterpret_cast<uint8_t*>(sp) + kQuickCalleeSaveFrame_RefAndArgs_FrameSize
                    + sizeof(ArtMethod*)),  // Skip ArtMethod*.
            gpr_index_(0), fpr_index_(0), fpr_double_index_(0), stack_index_(0),
            cur_type_(Primitive::kPrimVoid), is_split_long_or_double_(false) {
                static_assert(kQuickSoftFloatAbi == (kNumQuickFprArgs == 0),
                        "Number of Quick FPR arguments unexpected");
                static_assert(!(kQuickSoftFloatAbi && kQuickDoubleRegAlignedFloatBackFilled),
                        "Double alignment unexpected");
                // For register alignment, we want to assume that counters(fpr_double_index_) are even if the
                // next register is even.
                static_assert(!kQuickDoubleRegAlignedFloatBackFilled || kNumQuickFprArgs % 2 == 0,
                        "Number of Quick FPR arguments not even");
                DCHECK_EQ(Runtime::Current()->GetClassLinker()->GetImagePointerSize(), sizeof(void*));
            }

        virtual ~QuickArgumentVisitor() {}

        virtual void Visit() = 0;

        Primitive::Type GetParamPrimitiveType() const {
            return cur_type_;
        }

        uint8_t* GetParamAddress() const {
            if (!kQuickSoftFloatAbi) {
                Primitive::Type type = GetParamPrimitiveType();
                if (UNLIKELY((type == Primitive::kPrimDouble) || (type == Primitive::kPrimFloat))) {
                    if (type == Primitive::kPrimDouble && kQuickDoubleRegAlignedFloatBackFilled) {
                        if (fpr_double_index_ + 2 < kNumQuickFprArgs + 1) {
                            return fpr_args_ + (fpr_double_index_ * GetBytesPerFprSpillLocation(kRuntimeISA));
                        }
                    } else if (fpr_index_ + 1 < kNumQuickFprArgs + 1) {
                        return fpr_args_ + (fpr_index_ * GetBytesPerFprSpillLocation(kRuntimeISA));
                    }
                    return stack_args_ + (stack_index_ * kBytesStackArgLocation);
                }
            }
            if (gpr_index_ < kNumQuickGprArgs) {
                return gpr_args_ + GprIndexToGprOffset(gpr_index_);
            }
            return stack_args_ + (stack_index_ * kBytesStackArgLocation);
        }

        bool IsSplitLongOrDouble() const {
            if ((GetBytesPerGprSpillLocation(kRuntimeISA) == 4) ||
                    (GetBytesPerFprSpillLocation(kRuntimeISA) == 4)) {
                return is_split_long_or_double_;
            } else {
                return false;  // An optimization for when GPR and FPRs are 64bit.
            }
        }

        bool IsParamAReference() const {
            return GetParamPrimitiveType() == Primitive::kPrimNot;
        }

        bool IsParamALongOrDouble() const {
            Primitive::Type type = GetParamPrimitiveType();
            return type == Primitive::kPrimLong || type == Primitive::kPrimDouble;
        }

        uint64_t ReadSplitLongParam() const {
            // The splitted long is always available through the stack.
            return *reinterpret_cast<uint64_t*>(stack_args_
                    + stack_index_ * kBytesStackArgLocation);
        }

        void IncGprIndex() {
            gpr_index_++;
            if (kGprFprLockstep) {
                fpr_index_++;
            }
        }

        void IncFprIndex() {
            fpr_index_++;
            if (kGprFprLockstep) {
                gpr_index_++;
            }
        }

        void VisitArguments() SHARED_LOCKS_REQUIRED(Locks::mutator_lock_) {
            // (a) 'stack_args_' should point to the first method's argument
            // (b) whatever the argument type it is, the 'stack_index_' should
            //     be moved forward along with every visiting.
            gpr_index_ = 0;
            fpr_index_ = 0;
            if (kQuickDoubleRegAlignedFloatBackFilled) {
                fpr_double_index_ = 0;
            }
            stack_index_ = 0;
            if (!is_static_) {  // Handle this.
                cur_type_ = Primitive::kPrimNot;
                is_split_long_or_double_ = false;
                Visit();
                stack_index_++;
                if (kNumQuickGprArgs > 0) {
                    IncGprIndex();
                }
            }
            for (uint32_t shorty_index = 1; shorty_index < shorty_len_; ++shorty_index) {
                cur_type_ = Primitive::GetType(shorty_[shorty_index]);
                switch (cur_type_) {
                    case Primitive::kPrimNot:
                    case Primitive::kPrimBoolean:
                    case Primitive::kPrimByte:
                    case Primitive::kPrimChar:
                    case Primitive::kPrimShort:
                    case Primitive::kPrimInt:
                        is_split_long_or_double_ = false;
                        Visit();
                        stack_index_++;
                        if (gpr_index_ < kNumQuickGprArgs) {
                            IncGprIndex();
                        }
                        break;
                    case Primitive::kPrimFloat:
                        is_split_long_or_double_ = false;
                        Visit();
                        stack_index_++;
                        if (kQuickSoftFloatAbi) {
                            if (gpr_index_ < kNumQuickGprArgs) {
                                IncGprIndex();
                            }
                        } else {
                            if (fpr_index_ + 1 < kNumQuickFprArgs + 1) {
                                IncFprIndex();
                                if (kQuickDoubleRegAlignedFloatBackFilled) {
                                    // Double should not overlap with float.
                                    // For example, if fpr_index_ = 3, fpr_double_index_ should be at least 4.
                                    fpr_double_index_ = std::max(fpr_double_index_, RoundUp(fpr_index_, 2));
                                    // Float should not overlap with double.
                                    if (fpr_index_ % 2 == 0) {
                                        fpr_index_ = std::max(fpr_double_index_, fpr_index_);
                                    }
                                }
                            }
                        }
                        break;
                    case Primitive::kPrimDouble:
                    case Primitive::kPrimLong:
                        if (kQuickSoftFloatAbi || (cur_type_ == Primitive::kPrimLong)) {
                            if (cur_type_ == Primitive::kPrimLong && kAlignPairRegister && gpr_index_ == 0) {
                                // Currently, this is only for ARM, where the first available parameter register
                                // is R1. So we skip it, and use R2 instead.
                                IncGprIndex();
                            }
                            is_split_long_or_double_ = (GetBytesPerGprSpillLocation(kRuntimeISA) == 4) &&
                                ((gpr_index_ + 1) == kNumQuickGprArgs);
                            if (!kSplitPairAcrossRegisterAndStack && is_split_long_or_double_) {
                                // We don't want to split this. Pass over this register.
                                gpr_index_++;
                                is_split_long_or_double_ = false;
                            }
                            Visit();
                            if (kBytesStackArgLocation == 4) {
                                stack_index_+= 2;
                            } else {
                                CHECK_EQ(kBytesStackArgLocation, 8U);
                                stack_index_++;
                            }
                            if (gpr_index_ < kNumQuickGprArgs) {
                                IncGprIndex();
                                if (GetBytesPerGprSpillLocation(kRuntimeISA) == 4) {
                                    if (gpr_index_ < kNumQuickGprArgs) {
                                        IncGprIndex();
                                    }
                                }
                            }
                        } else {
                            is_split_long_or_double_ = (GetBytesPerFprSpillLocation(kRuntimeISA) == 4) &&
                                ((fpr_index_ + 1) == kNumQuickFprArgs) && !kQuickDoubleRegAlignedFloatBackFilled;
                            Visit();
                            if (kBytesStackArgLocation == 4) {
                                stack_index_+= 2;
                            } else {
                                CHECK_EQ(kBytesStackArgLocation, 8U);
                                stack_index_++;
                            }
                            if (kQuickDoubleRegAlignedFloatBackFilled) {
                                if (fpr_double_index_ + 2 < kNumQuickFprArgs + 1) {
                                    fpr_double_index_ += 2;
                                    // Float should not overlap with double.
                                    if (fpr_index_ % 2 == 0) {
                                        fpr_index_ = std::max(fpr_double_index_, fpr_index_);
                                    }
                                }
                            } else if (fpr_index_ + 1 < kNumQuickFprArgs + 1) {
                                IncFprIndex();
                                if (GetBytesPerFprSpillLocation(kRuntimeISA) == 4) {
                                    if (fpr_index_ + 1 < kNumQuickFprArgs + 1) {
                                        IncFprIndex();
                                    }
                                }
                            }
                        }
                        break;
                    default:
                        LOG(FATAL) << "Unexpected type: " << cur_type_ << " in " << shorty_;
                }
            }
        }

        protected:
        const bool is_static_;
        const char* const shorty_;
        const uint32_t shorty_len_;

        private:
        uint8_t* const gpr_args_;  // Address of GPR arguments in callee save frame.
        uint8_t* const fpr_args_;  // Address of FPR arguments in callee save frame.
        uint8_t* const stack_args_;  // Address of stack arguments in caller's frame.
        uint32_t gpr_index_;  // Index into spilled GPRs.
        // Index into spilled FPRs.
        // In case kQuickDoubleRegAlignedFloatBackFilled, it may index a hole while fpr_double_index_
        // holds a higher register number.
        uint32_t fpr_index_;
        // Index into spilled FPRs for aligned double.
        // Only used when kQuickDoubleRegAlignedFloatBackFilled. Next available double register indexed in
        // terms of singles, may be behind fpr_index.
        uint32_t fpr_double_index_;
        uint32_t stack_index_;  // Index into arguments on the stack.
        // The current type of argument during VisitArguments.
        Primitive::Type cur_type_;
        // Does a 64bit parameter straddle the register and stack arguments?
        bool is_split_long_or_double_;
    };


    // Visits arguments on the stack placing them into the shadow frame.
    class BuildQuickShadowFrameVisitor FINAL : public QuickArgumentVisitor {
        public:
            BuildQuickShadowFrameVisitor(ArtMethod** sp, bool is_static, const char* shorty,
                    uint32_t shorty_len, ShadowFrame* sf, size_t first_arg_reg) :
                QuickArgumentVisitor(sp, is_static, shorty, shorty_len), sf_(sf), cur_reg_(first_arg_reg) {}

            void Visit() SHARED_LOCKS_REQUIRED(Locks::mutator_lock_) OVERRIDE;

        private:
            ShadowFrame* const sf_;
            uint32_t cur_reg_;

            DISALLOW_COPY_AND_ASSIGN(BuildQuickShadowFrameVisitor);
    };


    void BuildQuickShadowFrameVisitor::Visit() {
        Primitive::Type type = GetParamPrimitiveType();
        switch (type) {
            case Primitive::kPrimLong:  // Fall-through.
            case Primitive::kPrimDouble:
                if (IsSplitLongOrDouble()) {
                    sf_->SetVRegLong(cur_reg_, ReadSplitLongParam());
                } else {
                    sf_->SetVRegLong(cur_reg_, *reinterpret_cast<jlong*>(GetParamAddress()));
                }
                ++cur_reg_;
                break;
            case Primitive::kPrimNot: {
                                          StackReference<mirror::Object>* stack_ref =
                                              reinterpret_cast<StackReference<mirror::Object>*>(GetParamAddress());
                                          sf_->SetVRegReference(cur_reg_, stack_ref->AsMirrorPtr());
                                      }
                                      break;
            case Primitive::kPrimBoolean:  // Fall-through.
            case Primitive::kPrimByte:     // Fall-through.
            case Primitive::kPrimChar:     // Fall-through.
            case Primitive::kPrimShort:    // Fall-through.
            case Primitive::kPrimInt:      // Fall-through.
            case Primitive::kPrimFloat:
                                      sf_->SetVReg(cur_reg_, *reinterpret_cast<jint*>(GetParamAddress()));
                                      break;
            case Primitive::kPrimVoid:
                                      LOG(FATAL) << "UNREACHABLE";
                                      UNREACHABLE();
        }
        ++cur_reg_;
    }


    // Visits arguments on the stack placing them into the args vector, Object* arguments are converted
    // to jobjects.
    class BuildQuickArgumentVisitor FINAL : public QuickArgumentVisitor {
        public:
            BuildQuickArgumentVisitor(ArtMethod** sp, bool is_static, const char* shorty, uint32_t shorty_len,
                    ScopedObjectAccessUnchecked* soa, std::vector<jvalue>* args) :
                QuickArgumentVisitor(sp, is_static, shorty, shorty_len), soa_(soa), args_(args) {}

            void Visit() SHARED_LOCKS_REQUIRED(Locks::mutator_lock_) OVERRIDE;

            void FixupReferences() SHARED_LOCKS_REQUIRED(Locks::mutator_lock_);

        private:
            ScopedObjectAccessUnchecked* const soa_;
            std::vector<jvalue>* const args_;
            // References which we must update when exiting in case the GC moved the objects.
            std::vector<std::pair<jobject, StackReference<mirror::Object>*>> references_;

            DISALLOW_COPY_AND_ASSIGN(BuildQuickArgumentVisitor);
    };


    void BuildQuickArgumentVisitor::Visit() {
        jvalue val;
        Primitive::Type type = GetParamPrimitiveType();
        switch (type) {
            case Primitive::kPrimNot: {
                                          StackReference<mirror::Object>* stack_ref =
                                              reinterpret_cast<StackReference<mirror::Object>*>(GetParamAddress());
                                          val.l = soa_->AddLocalReference<jobject>(stack_ref->AsMirrorPtr());
                                          references_.push_back(std::make_pair(val.l, stack_ref));
                                          break;
                                      }
            case Primitive::kPrimLong:  // Fall-through.
            case Primitive::kPrimDouble:
                                      if (IsSplitLongOrDouble()) {
                                          val.j = ReadSplitLongParam();
                                      } else {
                                          val.j = *reinterpret_cast<jlong*>(GetParamAddress());
                                      }
                                      break;
            case Primitive::kPrimBoolean:  // Fall-through.
            case Primitive::kPrimByte:     // Fall-through.
            case Primitive::kPrimChar:     // Fall-through.
            case Primitive::kPrimShort:    // Fall-through.
            case Primitive::kPrimInt:      // Fall-through.
            case Primitive::kPrimFloat:
                                      val.i = *reinterpret_cast<jint*>(GetParamAddress());
                                      break;
            case Primitive::kPrimVoid:
                                      LOG(FATAL) << "UNREACHABLE";
                                      UNREACHABLE();
        }
        args_->push_back(val);
    }

    void BuildQuickArgumentVisitor::FixupReferences() {
        // Fixup any references which may have changed.
        for (const auto& pair : references_) {
            pair.second->Assign(soa_->Decode<mirror::Object*>(pair.first));
            soa_->Env()->DeleteLocalRef(pair.first);
        }
    }

}  // namespace art
