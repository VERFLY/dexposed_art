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

#ifndef DEXPOSED_H
#define DEXPOSED_H

#include <jni.h>
#include <art_method.h>
#include <mirror/abstract_method.h>
#include <mirror/object.h>
#include <mirror/array.h>
#include <mirror/class.h>
#include <mirror/class-inl.h>
#include <mirror/method.h>
#include <mirror/object_array-inl.h>
#include <mirror/object-inl.h>
#include <well_known_classes.h>
#include <class_linker.h>
#include <primitive.h>
#include <reflection.h>
#include <runtime.h>
#include <scoped_thread_state_change.h>
#include <handle.h>
#include <util.h>
#include <stack.h>
#include <jni_internal.h>
#include <dex_file.h>

using art::ArtMethod;
using art::mirror::AbstractMethod;
using art::mirror::Array;
using art::mirror::ObjectArray;
using art::mirror::Class;
using art::mirror::Object;
using art::mirror::Constructor;
using art::PrettyMethod;
using art::Runtime;

using art::WellKnownClasses;
using art::Primitive;
using art::BoxPrimitive;
using art::UnboxPrimitiveForResult;
using art::UnboxPrimitiveForField;
using art::JValue;
using art::ScopedObjectAccess;
using art::ScopedObjectAccessUnchecked;
using art::ScopedObjectAccessAlreadyRunnable;
using art::StackHandleScope;
using art::Handle;
using art::StackReference;
using art::ClassLinker;
using art::ScopedJniEnvLocalRefState;

#define DEXPOSED_CLASS "com/taobao/android/dexposed/DexposedBridge"
#define DEXPOSED_ADDITIONAL_CLASS "com/taobao/android/dexposed/DexposedBridge$AdditionalHookInfo"
#define DEXPOSED_CLASS_DOTS "com.taobao.android.dexposed.DexposedBridge"

//#define PLATFORM_SDK_VERSION 21

#define SHARED_LOCKS_REQUIRED(...) THREAD_ANNOTATION_ATTRIBUTE__(shared_locks_required(__VA_ARGS__))

namespace art {
    struct DexposedHookInfo {
        jobject reflectedMethod;
        jobject additionalInfo;
        ArtMethod* originalMethod;
    };

    static bool dexposedIsHooked(ArtMethod* method);

    static void com_taobao_android_dexposed_DexposedBridge_hookMethodNative(JNIEnv* env, jclass clazz, jobject javaMethod, jobject declaredClassIndirect, jint slot, jobject additionalInfoIndirect);

    static int register_com_taobao_android_dexposed_DexposedBridge(JNIEnv* env);

} // namespace android

#endif  // DEXPOSED_H
