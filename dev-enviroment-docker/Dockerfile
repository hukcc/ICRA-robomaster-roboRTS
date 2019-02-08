FROM ros:kinetic-ros-base

RUN apt-get update
RUN apt-get install -y ros-kinetic-opencv3             \
                        ros-kinetic-cv-bridge           \
                        ros-kinetic-image-transport     \
                        ros-kinetic-stage-ros           \
                        ros-kinetic-map-server          \
                        ros-kinetic-laser-geometry      \
                        ros-kinetic-interactive-markers \ 
                        ros-kinetic-tf                  \
                        ros-kinetic-pcl-*               \
                        ros-kinetic-libg2o              \
                        ros-kinetic-rviz                \
                        protobuf-compiler               \
                        libprotobuf-dev                 \
                        libsuitesparse-dev              \
                        libgoogle-glog-dev              \
#basic tools for developers
build-essential libgtk2.0-dev libavcodec-dev libavformat-dev libjpeg.dev libtiff4.dev libswscale-dev libjasper-dev cmake wget acl adwaita-icon-theme at-spi2-core colord colord-data dconf-gsettings-backend dconf-service gconf-service libasound-dev gconf-service-backend gconf2-common glib-networking glib-networking-common glib-networking-services gsettings-desktop-schemas humanity-icon-theme  libatk-bridge2.0-0 libatspi2.0-0 libboost-filesystem1.58.0  libboost-system1.58.0 libcapnp-0.5.3 libcolord2 libcolorhug2  libdbus-glib-1-2 libdconf1 libegl1-mesa libepoxy0 libexif12 libgbm1  libgconf-2-4 libgd3 libgl1-mesa-dri libglapi-mesa libgphoto2-6  libgphoto2-l10n libgphoto2-port12 libgtk-3-0 libgtk-3-bin libgtk-3-common  libgudev-1.0-0 libgusb2 libieee1284-3 libjson-glib-1.0-0  libjson-glib-1.0-common liblcms2-2 libltdl7 libmirclient9 libmircommon7  libmircore1 libmirprotobuf3 libnotify4 libnspr4 libnss3 libnss3-nssdb  libpam-systemd libpolkit-agent-1-0 libpolkit-backend-1-0  libpolkit-gobject-1-0 libprotobuf-lite9v5 libproxy1v5 librest-0.7-0  librsvg2-2 librsvg2-common libsane libsane-common libsecret-1-0  libsecret-common libsoup-gnome2.4-1 libsoup2.4-1 libtxc-dxtn-s2tc0  libusb-1.0-0 libwayland-client0 libwayland-cursor0 libwayland-egl1-mesa  libwayland-server0 libxkbcommon0 libxkbfile1 libxpm4 libxss1 libxtst6  notification-daemon policykit-1 ubuntu-mono xkb-data
RUN wget -P /soft https://vscode.cdn.azure.cn/stable/c6e592b2b5770e40a98cb9c2715a8ef89aec3d74/code_1.30.0-1544567151_amd64.deb
RUN cd /soft && dpkg -i code_1.30.0-1544567151_amd64.deb

# 配置中文语言

ENV LANG zh_CN.UTF-8
ENV LANGUAGE zh_CN.UTF-8
ENV LC_ALL=zh_CN.UTF-8
RUN apt-get -y install language-pack-zh-hans language-pack-zh-hans-base
RUN locale
RUN apt-get -qqy --no-install-recommends install fonts-ipafont-gothic xfonts-100dpi xfonts-75dpi xfonts-cyrillic xfonts-scalable


# 安装文泉驿微米黑字体
RUN apt-get -qqy --no-install-recommends install ttf-wqy-microhei


# 将文泉驿微米黑设置为系统默认字体
RUN ln /etc/fonts/conf.d/65-wqy-microhei.conf /etc/fonts/conf.d/69-language-selector-zh-cn.conf
#USER hukcc
RUN useradd --create-home --no-log-init --shell /bin/bash hukcc
