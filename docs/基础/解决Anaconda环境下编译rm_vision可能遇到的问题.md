# 解决Anaconda环境下编译rm_vision可能遇到的问题

- ModuleNotFoundError: No module named 'catkin_pkg'
`pip3 install catkin_pkg`

- ModuleNotFoundError: No module named 'em'
  或 AttributeError: module 'em' has no attribute 'BUFFERED_OPT'
  或 AttributeError: module 'em' has no attribute 'Interpreter'
empy 的版本与python版本不适配，可以尝试下载 empy3.3.2版本 [^1]。
`pip install empy==3.3.2`

- ModuleNotFoundError: No module named 'lark'
`pip3 install lark`

- curl库问题

```txt
/usr/bin/ld: /lib/x86_64-linux-gnu/libnetcdf.so.19: undefined reference to `curl_global_cleanup@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_easy_getinfo@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_multi_cleanup@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_multi_perform@CURL_OPENSSL_4'
/usr/bin/ld: /lib/x86_64-linux-gnu/libnetcdf.so.19: undefined reference to `curl_global_init@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_easy_perform@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_mime_init@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_easy_init@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_multi_add_handle@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_multi_remove_handle@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_mime_addpart@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_version_info@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_mime_name@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_mime_filename@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_multi_info_read@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_slist_free_all@CURL_OPENSSL_4'
/usr/bin/ld: /lib/x86_64-linux-gnu/libnetcdf.so.19: undefined reference to `curl_easy_strerror@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_easy_setopt@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_slist_append@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_multi_poll@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_version@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_multi_setopt@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_mime_data@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_mime_free@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_easy_cleanup@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_multi_init@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_mime_data_cb@CURL_OPENSSL_4'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `curl_multi_wait@CURL_OPENSSL_4'
```

使用`locate libcurl.so.4`定位文件位置，可以看到存在以下目录：

```txt
...
/home/iowqi/anaconda3/lib/libcurl.so.4
/home/iowqi/anaconda3/lib/libcurl.so.4.8.0
...
/usr/lib/x86_64-linux-gnu/libcurl.so.4
/usr/lib/x86_64-linux-gnu/libcurl.so.4.7.0
```

将`anaconda3/lib/`下的`libcurl.so.4`删除，之后创建软链接将系统库链接到conda的库中(注意修改版本和路径) [^2]

```bash
rm /home/$USER/anaconda3/lib/libcurl.so.4
ln -s /usr/lib/x86_64-linux-gnu/libcurl.so.4.7.0 /home/$USER/anaconda3/lib/libcurl.so.4
```

- tiff库问题

```txt
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `TIFFReadRGBAStripExt@LIBTIFF_4.0'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `TIFFReadFromUserBuffer@LIBTIFF_4.0'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `TIFFGetStrileOffsetWithErr@LIBTIFF_4.0'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `TIFFGetStrileOffset@LIBTIFF_4.0'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `TIFFGetStrileByteCount@LIBTIFF_4.0'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `TIFFDeferStrileArrayWriting@LIBTIFF_4.0'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `TIFFGetStrileByteCountWithErr@LIBTIFF_4.0'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `TIFFForceStrileArrayWriting@LIBTIFF_4.0'
/usr/bin/ld: /lib/libgdal.so.30: undefined reference to `TIFFReadRGBATileExt@LIBTIFF_4.0'
```

使用`ldd /usr/lib/libgdal.so.30 | grep tif`查看libgdal.so.30对tiff库的依赖关系

```txt
 libgeotiff.so.5 => /lib/x86_64-linux-gnu/libgeotiff.so.5 (0x000073fb42f26000)
 libtiff.so.5 => /lib/x86_64-linux-gnu/libtiff.so.5 (0x000073fb42dc6000)
```

使用`locate libtiff.so.5`定位文件位置，可以看到存在以下目录：

```txt
...
/home/iowqi/anaconda3/lib/libtiff.so.5
...
/usr/lib/x86_64-linux-gnu/libtiff.so.5
/usr/lib/x86_64-linux-gnu/libtiff.so.5.7.0
```

将`anaconda3/lib/`下的`libtiff.so.5`删除，之后创建软链接将系统库链接到conda的库中(注意修改版本和路径) [^2]

```bash
rm /home/$USER/anaconda3/lib/libtiff.so.5
ln -s /usr/lib/x86_64-linux-gnu/libtiff.so.5.7.0 /home/$USER/anaconda3/lib/libtiff.so.5
```

[^1]: [https://blog.csdn.net/qq_34168988/article/details/135099372](https://blog.csdn.net/qq_34168988/article/details/135099372)

[^2]: [https://askubuntu.com/questions/1211782/curl-openssl-4-not-found-required-by-curl](https://askubuntu.com/questions/1211782/curl-openssl-4-not-found-required-by-curl)
