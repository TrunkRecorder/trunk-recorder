FROM ubuntu:24.04 AS builder

# Install docker for passing the socket to allow for intercontainer exec
RUN apt-get update && \
  apt-get -y upgrade &&\
  export DEBIAN_FRONTEND=noninteractive && \
  apt-get install --no-install-recommends -y \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    git \
    gnuradio-dev \
    gr-osmosdr \
    libosmosdr-dev \
    libairspy-dev \
    libairspyhf-dev \
    libbladerf-dev \
    libboost-all-dev \
    libcurl4-openssl-dev \
    libfreesrp-dev \
    libgmp-dev \
    libhackrf-dev \
    libmirisdr-dev \
    liborc-0.4-dev \
    libpthread-stubs0-dev \
    librtlsdr-dev \
    libsndfile1-dev \
    libsoapysdr-dev \
    libssl-dev \
    libuhd-dev \
    libusb-dev \
    libusb-1.0-0-dev \
    libxtrx-dev \
    pkg-config \
    wget \
    python3-six \
    openssh-client \
    ffmpeg \
    libiio-dev \
    libiio-utils \
    libad9361-dev \
    libgnuradio-iio3.10.9t64

WORKDIR /src

COPY . .

WORKDIR /src/build

RUN cmake .. && make -j$(nproc) && make DESTDIR=/newroot install

#Stage 2 build
FROM ubuntu:24.04
RUN apt-get update && apt-get -y upgrade && apt-get install --no-install-recommends -y ca-certificates gr-funcube gr-iqbal curl wget libboost-log1.83.0 \
    libboost-chrono1.83.0t64 libgnuradio-digital3.10.9t64 libgnuradio-analog3.10.9t64 libgnuradio-filter3.10.9t64 libgnuradio-network3.10.9t64  \
    libgnuradio-uhd3.10.9t64 libgnuradio-osmosdr0.2.0t64 libiio-dev libiio-utils libad9361-dev libgnuradio-iio3.10.9t64 libsoapysdr0.8 \
    soapysdr0.8-module-all libairspyhf1 libfreesrp0 librtlsdr2 libxtrx0 sox fdkaac docker.io && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /usr/share/{doc,man,info} && rm -rf /usr/local/share/{doc,man,info}

COPY --from=builder /newroot /

# Fix the error message level for SmartNet
RUN mkdir -p /etc/gnuradio/conf.d/ && echo 'log_level = info' >> /etc/gnuradio/conf.d/gnuradio-runtime.conf && ldconfig
WORKDIR /app

# GNURadio requires a place to store some files, can only be set via $HOME env var.
ENV HOME=/tmp

#USER nobody
# We need to use this style of CMD or else it will leak zombie processes with Unit Script
# See #539
CMD trunk-recorder --config=/app/config.json
