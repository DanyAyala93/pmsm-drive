# Ubuntu 20.04 LTS is the standard platform for development
FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get -y install cmake=3.16.3-1ubuntu1 \
    && apt-get -y install g++=4:9.3.0-1ubuntu2 \
    && apt-get -y install protobuf-compiler=3.6.1.3-2ubuntu5 \
    && apt-get -y install wget=1.20.3-1ubuntu1 \
    && apt-get -y install zip=3.0-11build1 \
    && apt-get -y install cloc=1.82-1 \
    && apt-get -y install doxygen=1.8.17-0ubuntu2 \
    && apt-get -y install python3=3.8.2-0ubuntu2 \
    && apt-get -y install python3-pip \
    && apt-get -y install git \
    && apt-get -y install libncurses5 \
    && apt-get -y install make=4.2.1-1.2 \
    && apt-get -y install clang=1:10.0-50~exp1

WORKDIR /usr/src/

RUN apt-get update \
	&& git clone https://github.com/catchorg/Catch2.git -b v2.7.0 \
	&& cd Catch2 \
	&& cmake -Bbuild -H. -DBUILD_TESTING=OFF \
	&& cmake --build build/ --target install

RUN ln -sf /usr/bin/llvm-profdata-10 /usr/bin/llvm-profdata
RUN ln -sf /usr/bin/llvm-cov-10 /usr/bin/llvm-cov

WORKDIR /home/dev

# install Bloaty McBloatface: a size profiler for binaries
RUN git clone https://github.com/google/bloaty.git \
    && cmake -Bbloaty bloaty/ \
    && make -j`nproc` -C bloaty/

# MPLAB XC32 compiler
RUN wget https://ww1.microchip.com/downloads/en/DeviceDoc/xc32-v2.50-full-install-linux-installer.run && \
    chmod +x xc32-v2.50-full-install-linux-installer.run && \
    echo "Installing XC32 compiler, this process could take a few minutes, please wait..." && \
    ./xc32-v2.50-full-install-linux-installer.run --mode unattended --unattendedmodeui none --netservername localhost --LicenseType FreeMode

ENV PATH "/opt/microchip/xc32/v2.50/bin:$PATH"
