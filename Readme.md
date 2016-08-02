README
-----------
由于dexposed art近期一直没有更新，因此自己参考xposed最新版本的实现，在Android 6.0.1上做了简单的尝试，抛砖引玉！

* 注意：（1）仅在Android 6.0.1上做过测试；（2）仅支持ARM 64位处理器。

Step 1:
-----------------
* 下载AOSP 6.0.1版本源代码，整体编译一次

Step 2:
-----------
* 由于AOSP 6.0.1将libcxx.mk文件删除，因此将libcxx.mk文件拷贝到ANDROID_SOURCE_CODE/external/libcxx目录下
* 拷贝dexposed_art目录到 ANDROID_SOURCE_CODE/frameworks/base/cmds
* 切换到ANDROID_SOURCE_CODE/frameworks/base/cmds，执行命令'mmm -B dexposed_art'
* 编译成功后, 在ANDROID_SOURCE_CODE/out/target/product/generic/system/lib64/目录下可以找到libdexposed_m.so库

-----------
