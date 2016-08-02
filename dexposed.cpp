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

#include "dexposed.h"

#include <utils/Log.h>
#include <android_runtime/AndroidRuntime.h>
#include <stdio.h>
#include <sys/mman.h>
#include <cutils/properties.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <entrypoints/entrypoint_utils.h>

#include "quick_argument_visitor.cpp"


namespace art {

	jclass dexposed_class = NULL;
	jmethodID dexposed_handle_hooked_method = NULL;
	jclass additionalhookinfo_class = NULL;
	jfieldID  additionalhookinfo_shorty_field = NULL;

	void logMethod(const char* tag, ArtMethod* method) {
		LOG(INFO) << "dexposed:" << tag << " " << method << " " << PrettyMethod(method);
	}

	bool dexposedOnVmCreated(JNIEnv* env, const char*) {

		dexposed_class = env->FindClass(DEXPOSED_CLASS);
		dexposed_class = reinterpret_cast<jclass>(env->NewGlobalRef(dexposed_class));

		if (dexposed_class == NULL) {
			LOG(ERROR) << "dexposed: Error while loading Dexposed class " << DEXPOSED_CLASS;
			env->ExceptionClear();
			return false;
		}

		additionalhookinfo_class = env->FindClass(DEXPOSED_ADDITIONAL_CLASS);
		additionalhookinfo_class = reinterpret_cast<jclass>(env->NewGlobalRef(additionalhookinfo_class));
		if (additionalhookinfo_class == NULL) {
			LOG(ERROR) << "dexposed: Error while loading Dexposed class " << DEXPOSED_ADDITIONAL_CLASS;
			env->ExceptionClear();
			return false;
		}

		LOG(INFO) << "dexposed: now initializing, Found Dexposed class " << DEXPOSED_CLASS;
		if (register_com_taobao_android_dexposed_DexposedBridge(env) != JNI_OK) {
			LOG(ERROR) << "dexposed: Could not register natives for " << DEXPOSED_CLASS;
			env->ExceptionClear();
			return false;
		}

		return true;
	}

	static jboolean initNative(JNIEnv* env, jclass) {

		LOG(INFO) << "dexposed: initNative";

		dexposed_handle_hooked_method =
				env->GetStaticMethodID(dexposed_class, "handleHookedMethod",
						"(Ljava/lang/reflect/Member;ILjava/lang/Object;Ljava/lang/Object;[Ljava/lang/Object;)Ljava/lang/Object;");
		if (dexposed_handle_hooked_method == NULL) {
			LOG(ERROR) << "dexposed: Could not find method " << DEXPOSED_CLASS << ".handleHookedMethod()";
			env->ExceptionClear();
			return false;
		}

		return true;
	}

	extern "C" JNIEXPORT jint JNI_OnLoad(JavaVM* vm, void*)
	{
		JNIEnv* env = NULL;
		jint result = -1;

		if (vm->GetEnv((void**) &env, JNI_VERSION_1_6) != JNI_OK) {
			return result;
		}

		int keepLoadingDexposed = dexposedOnVmCreated(env, NULL);
		if(keepLoadingDexposed)
			initNative(env, NULL);

		return JNI_VERSION_1_6;
	}

	extern "C" void art_quick_dexposed_invoke_handler();
	static inline const void* GetQuickDexposedInvokeHandler() {
		return reinterpret_cast<void*>(art_quick_dexposed_invoke_handler);
	}

	JValue InvokeXposedHandleHookedMethod(ScopedObjectAccessAlreadyRunnable& soa, const char* shorty,
	                                    jobject rcvr_jobj, jmethodID method,
	                                    std::vector<jvalue>& args)
		SHARED_LOCKS_REQUIRED(Locks::mutator_lock_) {

		LOG(INFO) << "dexposed: InvokeXposedHandleHookedMethod";
        // Build argument array possibly triggering GC.
        soa.Self()->AssertThreadSuspensionIsAllowable();
        jobjectArray args_jobj = nullptr;
        const JValue zero;
        int32_t target_sdk_version = Runtime::Current()->GetTargetSdkVersion();
        // Do not create empty arrays unless needed to maintain Dalvik bug compatibility.
        if (args.size() > 0 || (target_sdk_version > 0 && target_sdk_version <= 21)) {
            args_jobj = soa.Env()->NewObjectArray(args.size(), WellKnownClasses::java_lang_Object, nullptr);
            if (args_jobj == nullptr) {
                CHECK(soa.Self()->IsExceptionPending());
                return zero;
            }
            for (size_t i = 0; i < args.size(); ++i) {
                if (shorty[i + 1] == 'L') {
                    jobject val = args.at(i).l;
                    soa.Env()->SetObjectArrayElement(args_jobj, i, val);
                } else {
                    JValue jv;
                    jv.SetJ(args.at(i).j);
                    mirror::Object* val = BoxPrimitive(Primitive::GetType(shorty[i + 1]), jv);
                    if (val == nullptr) {
                        CHECK(soa.Self()->IsExceptionPending());
                        return zero;
                    }
                    soa.Decode<mirror::ObjectArray<mirror::Object>* >(args_jobj)->Set<false>(i, val);
                }
            }
        }

#if PLATFORM_SDK_VERSION < 22
        const DexposedHookInfo *hookInfo =
                (DexposedHookInfo *) (soa.DecodeMethod(method)->GetNativeMethod());
#else
        const DexposedHookInfo *hookInfo =
                (DexposedHookInfo *) (soa.DecodeMethod(method)->GetEntryPointFromJni());
#endif

        jvalue invocation_args[5];
        invocation_args[0].l = hookInfo->reflectedMethod;
        invocation_args[1].i = 1;
        invocation_args[2].l = hookInfo->additionalInfo;
        invocation_args[3].l = rcvr_jobj;
        invocation_args[4].l = args_jobj;
        jobject result = soa.Env()->CallStaticObjectMethodA(dexposed_class,
                                             dexposed_handle_hooked_method,
                                             invocation_args);


        // Unbox the result if necessary and return it.
        if (UNLIKELY(soa.Self()->IsExceptionPending())) {
            return zero;
        } else {
            if (shorty[0] == 'V' || (shorty[0] == 'L' && result == nullptr)) {
                return zero;
            }
            // This can cause thread suspension.
            mirror::Class* result_type = soa.DecodeMethod(method)->GetReturnType();
            mirror::Object* result_ref = soa.Decode<mirror::Object*>(result);
            JValue result_unboxed;
            if (!UnboxPrimitiveForResult(result_ref, result_type, &result_unboxed)) {
                DCHECK(soa.Self()->IsExceptionPending());
                return zero;
            }
            return result_unboxed;
        }
	}

	// Handler for invocation on proxy methods. On entry a frame will exist for the proxy object method
	// which is responsible for recording callee save registers. We explicitly place into jobjects the
	// incoming reference arguments (so they survive GC). We invoke the invocation handler, which is a
	// field within the proxy object, which will box the primitive arguments and deal with error cases.
	extern "C" uint64_t artQuickDexposedInvokeHandler(ArtMethod* proxy_method,
			Object* receiver, Thread* self, ArtMethod** sp)
	SHARED_LOCKS_REQUIRED(Locks::mutator_lock_) {

		const bool is_static = proxy_method->IsStatic();

		LOG(INFO) << "dexposed: artQuickDexposedInvokeHandler isStatic:" << is_static;

		// Ensure we don't get thread suspension until the object arguments are safely in jobjects.
		const char* old_cause = self->StartAssertNoThreadSuspension(
				"Adding to IRT proxy object arguments");

        // Register the top of the managed stack, making stack crawlable.
        DCHECK_EQ((*sp), proxy_method) << PrettyMethod(proxy_method);
        DCHECK_EQ(proxy_method->GetFrameSizeInBytes(),
            Runtime::Current()->GetCalleeSaveMethod(Runtime::kRefsAndArgs)->GetFrameSizeInBytes()) << PrettyMethod(proxy_method);
		self->VerifyStack();
		// Start new JNI local reference state.
		JNIEnvExt* env = self->GetJniEnv();
		ScopedObjectAccessUnchecked soa(env);
		ScopedJniEnvLocalRefState env_state(env);
		// Create local ref. copies of proxy method and the receiver.
		jobject rcvr_jobj = is_static ? nullptr : soa.AddLocalReference<jobject>(receiver);

		// Placing arguments into args vector and remove the receiver.
		ArtMethod* non_proxy_method = proxy_method->GetInterfaceMethodIfProxy(sizeof(void*));

		std::vector < jvalue > args;

#if PLATFORM_SDK_VERSION < 22
        const DexposedHookInfo *hookInfo =
                (DexposedHookInfo *) (proxy_method->GetNativeMethod());
#else
        const DexposedHookInfo *hookInfo =
                (DexposedHookInfo *) (proxy_method->GetEntryPointFromJni());
#endif


		uint32_t shorty_len = 0;
		const char* shorty = proxy_method->GetShorty(&shorty_len);
		//const char* shorty = hookInfo->shorty;
		//shorty_len = strlen(shorty);

		for(uint32_t i=0; i<shorty_len; ++i){
			LOG(INFO) << "dexposed: artQuickDexposedInvokeHandler " << "shorty[" << i << "]:" << shorty[i];
		}

		BuildQuickArgumentVisitor local_ref_visitor(sp, is_static, shorty, shorty_len, &soa, &args);
		local_ref_visitor.VisitArguments();
		if (!is_static) {
			DCHECK_GT(args.size(), 0U) << PrettyMethod(proxy_method);
			args.erase(args.begin());
		}
		LOG(INFO) << "dexposed: artQuickDexposedInvokeHandler args.size:" << args.size();
	    jmethodID proxy_methodid = soa.EncodeMethod(proxy_method);
	    self->EndAssertNoThreadSuspension(old_cause);
	    JValue result = InvokeXposedHandleHookedMethod(soa, shorty, rcvr_jobj, proxy_methodid, args);
	    local_ref_visitor.FixupReferences();
	    return result.GetJ();
	}

	static void EnableXposedHook(JNIEnv* env, ArtMethod* art_method, jobject additional_info)
	  SHARED_LOCKS_REQUIRED(Locks::mutator_lock_) {

	  LOG(INFO) << "dexposed: >>> EnableXposedHook " << art_method << " " << PrettyMethod(art_method);
	  if (dexposedIsHooked(art_method)) {
		// Already hooked
		return;
	  }

	  ScopedObjectAccess soa(env);

	  // Create a backup of the ArtMethod object
      auto* cl = Runtime::Current()->GetClassLinker();
      ArtMethod* backup_method = cl->AllocArtMethodArray(soa.Self(), 1);
      backup_method->CopyFrom(art_method, cl->GetImagePointerSize());
	  backup_method->SetAccessFlags(backup_method->GetAccessFlags() /*| kAccXposedOriginalMethod*/);
	  // Create a Method/Constructor object for the backup ArtMethod object
        mirror::AbstractMethod* reflect_method;
        if (art_method->IsConstructor()) {
            reflect_method = mirror::Constructor::CreateFromArtMethod(soa.Self(), backup_method);
        } else {
            reflect_method = mirror::Method::CreateFromArtMethod(soa.Self(), backup_method);
        }
        reflect_method->SetAccessible<false>(true);
        
	  // Save extra information in a separate structure, stored instead of the native method
	  DexposedHookInfo* hookInfo = reinterpret_cast<DexposedHookInfo*>(calloc(1, sizeof(DexposedHookInfo)));
	  hookInfo->reflectedMethod = soa.Vm()->AddGlobalRef(soa.Self(), reflect_method);
	  hookInfo->additionalInfo = soa.Env()->NewGlobalRef(additional_info);
	  hookInfo->originalMethod = backup_method;

#if PLATFORM_SDK_VERSION < 22
        art_method->SetNativeMethod(reinterpret_cast<uint8_t *>(hookInfo));
#else
        art_method->SetEntryPointFromJni(reinterpret_cast<void *>(hookInfo));
#endif

	  art_method->SetEntryPointFromQuickCompiledCode(GetQuickDexposedInvokeHandler());
//	  art_method->SetEntryPointFromInterpreter(art::artInterpreterToCompiledCodeBridge);
	  // Adjust access flags
	  art_method->SetAccessFlags((art_method->GetAccessFlags() & ~kAccNative) /*| kAccXposedHookedMethod*/);
	}

	static void com_taobao_android_dexposed_DexposedBridge_hookMethodNative(
			JNIEnv* env, jclass, jobject java_method, jobject, jint,
			jobject additional_info) {

		ScopedObjectAccess soa(env);
		art::Thread* self = art::Thread::Current();

        ArtMethod* artMethod = ArtMethod::FromReflectedMethod(soa, java_method);

	    LOG(INFO) << "dexposed: >>> hookMethodNative " << artMethod << " " << PrettyMethod(artMethod);
	    EnableXposedHook(env, artMethod, additional_info);
	}

	static bool dexposedIsHooked(ArtMethod* method) {
		return (method->GetEntryPointFromQuickCompiledCode())
				== (void *) GetQuickDexposedInvokeHandler();
	}

	extern "C" jobject com_taobao_android_dexposed_DexposedBridge_invokeOriginalMethodNative(
			JNIEnv* env, jclass, jobject java_method, jint, jobject, jobject,
			jobject thiz, jobject args)
	SHARED_LOCKS_REQUIRED(Locks::mutator_lock_) {
	    LOG(INFO) << "dexposed: >>> invokeOriginalMethodNative";
        ScopedObjectAccess soa(env);
#if PLATFORM_SDK_VERSION >= 21
		return art::InvokeMethod(soa, java_method, thiz, args, true);
#else
		return art::InvokeMethod(soa, java_method, thiz, args);
#endif
	}

	extern "C" jobject com_taobao_android_dexposed_DexposedBridge_invokeSuperNative(
			JNIEnv* env, jclass, jobject thiz, jobject args, jobject java_method, jobject, jobject,
			jint slot, jboolean check)

	SHARED_LOCKS_REQUIRED(Locks::mutator_lock_) {

		LOG(INFO) << "dexposed: >>> invokeSuperNative";

		ScopedObjectAccess soa(env);
		ArtMethod* method = ArtMethod::FromReflectedMethod(soa, java_method);

		// Find the actual implementation of the virtual method.
		ArtMethod* mm = method->FindOverriddenMethod(sizeof(void*));

        mirror::AbstractMethod* mm_abstract;
        if (mm->IsConstructor()) {
            mm_abstract = mirror::Constructor::CreateFromArtMethod(soa.Self(), mm);
        } else {
            mm_abstract = mirror::Method::CreateFromArtMethod(soa.Self(), mm);
        }
        mm_abstract->SetAccessible<false>(true);
        
		jobject reflect_method = env->AllocObject(WellKnownClasses::java_lang_reflect_Method);
		env->SetObjectField(reflect_method,
						WellKnownClasses::java_lang_reflect_AbstractMethod_artMethod,
						env->NewGlobalRef(soa.AddLocalReference<jobject>(mm_abstract)));
#if PLATFORM_SDK_VERSION >= 21
		return art::InvokeMethod(soa, reflect_method, thiz, args, true);
#else
		return art::InvokeMethod(soa, reflect_method, thiz, args);
#endif
	}

	static const JNINativeMethod dexposedMethods[] =
	{
		{ "hookMethodNative", "(Ljava/lang/reflect/Member;Ljava/lang/Class;ILjava/lang/Object;)V",
							(void*) com_taobao_android_dexposed_DexposedBridge_hookMethodNative },
		{ "invokeOriginalMethodNative",
				  "(Ljava/lang/reflect/Member;I[Ljava/lang/Class;Ljava/lang/Class;Ljava/lang/Object;[Ljava/lang/Object;)Ljava/lang/Object;",
							(void*) com_taobao_android_dexposed_DexposedBridge_invokeOriginalMethodNative },
		{ "invokeSuperNative", "(Ljava/lang/Object;[Ljava/lang/Object;Ljava/lang/reflect/Member;Ljava/lang/Class;[Ljava/lang/Class;Ljava/lang/Class;I)Ljava/lang/Object;",
				(void*) com_taobao_android_dexposed_DexposedBridge_invokeSuperNative},
	};

	static int register_com_taobao_android_dexposed_DexposedBridge(JNIEnv* env) {
		return env->RegisterNatives(dexposed_class, dexposedMethods, sizeof(dexposedMethods) / sizeof(dexposedMethods[0]));
	}
}

