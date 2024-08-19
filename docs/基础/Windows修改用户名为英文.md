# Windows修改用户名为英文

1.右键【任务栏win徽标】->【计算机管理】->【本地用户和组】->【用户】->右键【中文用户名】->【重命名】

***
如果是win10，则是：右键【计算机（一般放在桌面上，如果没有的话桌面右键选择个性化，然后打开桌面图标）】->【管理】->【本地用户和组】->【用户】->右键【中文用户名】->【重命名】
***

!**注意这个英文用户名后续还要使用**!

2.以管理员权限启动命令提示符，输入

``` bat
net user administrator /active:yes
```

如果杀毒软件提示恶意修改，请放行

3.重启电脑，登录的时候选择**Administrator**账户，需要进行一些简单的设置，直接一路确定即可。

4.进入C盘的用户文件夹下，把你原来的用户文件夹名称改成刚刚设定的英文用户名。

5.【Windows+R】->【运行】->输入 regedit ->回车

找到`HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Windows NT\CurrentVersion\Profilelist`，把这个标签页下每个选项都点开看，其中有一个页面对应的ProfileImagePath值是以你用户名命名的文件夹路径，比如路径是`C:\User\小白`。

双击，把这个路径改成你新设的用户名，比如`C:\User\iowqi`

6.重启电脑，进入修改好的用户，

7.**以管理员权限**启动命令提示符，输入

```bat
net user administrator /active:no
```

如果杀毒软件提示恶意修改，请放行。这会关闭Administator账户。
