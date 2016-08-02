LOCAL_C_INCLUDES := \
	external/libcxx/include \
	$(LOCAL_C_INCLUDES) \
LOCAL_CFLAGS += -D_USING_LIBCXX
LOCAL_CPPFLAGS += -nostdinc++
LOCAL_LDFLAGS += -nodefaultlibs
LOCAL_LDLIBS += -lm -lc
LOCAL_SHARED_LIBRARIES += libc++
