# Jetson系列发行版升级到22.04

1. 编辑`/etc/update-manager/release-upgrades`中的`Prompt=lts`，将`lts`改为`normal`。
2. 执行`sudo do-release-upgrade`（有关确认询问，一般输入y或enter回车）。

    如果在升级时中断过程，重新执行`sudo do-release-upgrade`即可。
    提示`Please install all available updates for your release before upgrading.`则执行更新命令

    ```bash
    sudo apt update
    sudo apt upgrade
    ```

3. 可能遇到的问题(1): Firefox升级部署snapd过程中卡在 `Waiting for automatic snapd restart ...`

    解决方案：[^1]

    ```bash
    killall firefox
    sudo snap refresh
    sudo service snapd restart
    sudo snap remove firefox (输出 no firefox installed)

    #如果还是等待中：
    sudo systemctl stop snapd.socket snapd.service
    sudo umount /var/lib/snapd/snaps/*.snap
    sudo rm -rvf /var/lib/snapd/*
    sudp rm -vf /etc/systemd/system/snap-*.mount
    sudo rm -vf /etc/systemd/system/snap-*.service
    sudo rm -vf /etc/systemd/system/multi-user.target.wants/snap-*.mount
    sudo rm -vrf /snap/*
    sudo systemctl start snapd.socket
    ```

4. 可能遇到的问题(2): libvulkan版本冲突

    ```bash
    dpkg: error processing archive /var/cache/apt/archives/libvulkan1_1.3.204.1-2_ar
    m64.deb (--unpack):
    trying to overwrite '/usr/lib/aarch64-linux-gnu/libvulkan.so.1.3.204', which is
    also in package nvidia-l4t-libvulkan 35.5.0-20240219203809
    dpkg-deb: error: paste subprocess was killed by signal (Broken pipe)
    Errors were encountered while processing:
    /var/cache/apt/archives/libvulkan1_1.3.204.1-2_arm64.deb
    E: Sub-process /usr/bin/dpkg returned an error code (1)
    ```

    然后会导致

    ```bash
    The following packages have unmet dependencies:
    libvulkan-dev : Depends: libvulkan1 (= 1.3.204.1-2) but 1.2.131.2-1 is installed
    vulkan-tools : Depends: libvulkan1 (>= 1.3.204.0-1) but 1.2.131.2-1 is installed
    E: Unmet dependencies. Try 'apt --fix-broken install' with no packages (or specify a solution).
    ```

    解决方案：[^2]
    使用下面指令，它将以强行覆盖的方式解决 “the trying to overwrite” 错误。

    ```bash
    sudo dpkg -i --force-overwrite /var/cache/apt/archives/libvulkan1_1.3.204.1-2_arm64.deb
    ```

5. 升级完成
![1](../../static/img/Jetson系列发行版升级到22.04/1.png)

[^1]: [https://askubuntu.com/questions/1230390/chromium-installation-error-in-lubuntu-20-04](https://askubuntu.com/questions/1230390/chromium-installation-error-in-lubuntu-20-04)

[^2]: [https://askubuntu.com/questions/1062171/dpkg-deb-error-paste-subprocess-was-killed-by-signal-broken-pipe](https://askubuntu.com/questions/1062171/dpkg-deb-error-paste-subprocess-was-killed-by-signal-broken-pipe)
