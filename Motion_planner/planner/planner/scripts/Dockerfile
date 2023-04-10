# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
ARG ROS_DISTRO=noetic
ARG BASE_IMAGE=osrf/ros:${ROS_DISTRO}-desktop-full
FROM $BASE_IMAGE

# opencv version must be consistent with the version of ROS, 
# othervize you may need to rebuild image-transport and other ROS packages


ENV DEBIAN_FRONTEND noninteractive

RUN \
  apt-get -y -q update && \
  DEBIAN_FRONTEND=noninteractive apt-get install -y \
  software-properties-common
  
RUN add-apt-repository ppa:deadsnakes/ppa

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3.9

RUN cd /usr/bin && mv python3 python3_old && ln -s python3.9 python3

RUN \
  apt-get -y -q update && \
  DEBIAN_FRONTEND=noninteractive apt-get install -y \
    cmake \
    gcc \
    g++ \
    build-essential \
    wget \
    curl \
    unzip \
    git \
    git-lfs \
    python3.9-dev \
    autotools-dev \
    m4 \
    libicu-dev \
    build-essential \
    libbz2-dev \
    libasio-dev \
    libeigen3-dev \
    libglew-dev \
    freeglut3-dev \
    expat \
    libcairo2-dev \
    cmake \
    libboost-dev \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    nlohmann-json3-dev \
    python3-pip \
    ffmpeg \
    libhdf5-dev \
    nano \
    htop \
    gdb \
    ros-noetic-cv-bridge \
    ros-noetic-image-geometry \
    ros-noetic-rviz \
    ros-noetic-image-proc \
    protobuf-compiler \
    gfortran \
    libopenblas-dev \
    liblapack-dev \
    gnutls-bin

RUN python3.9 -m pip install numpy --upgrade \
			      torch --upgrade \
			      pillow --upgrade \
			      kiwisolver --upgrade \
			      pytorch_lightning --upgrade \
			      scipy --upgrade \
			      pandas --upgrade \
			      scikit-learn --upgrade \
			      scikit-image --upgrade \
			      cython --upgrade \
			      bokeh --upgrade \
			      sqlalchemy --upgrade \
			      h5py --upgrade \
			      vincent --upgrade \
			      jupyterlab --upgrade \
			      click --upgrade \
			      bitarray --upgrade \
			      psutil --upgrade \
			      tqdm --upgrade \
			      ipywidgets --upgrade \
			      seaborn --upgrade \
			      sympy --upgrade \
			      cloudpickle --upgrade \
			      dill --upgrade \
			      dask --upgrade \
			      bokeh --upgrade \
			      sqlalchemy --upgrade \
			      beautifulsoup4 --upgrade \
			      protobuf --upgrade \
			      xlrd --upgrade \
			      matplotlib --upgrade

RUN git config --global http.postBuffer 1048576000
 
RUN mkdir -p /root/code

# Build and install OMPL
WORKDIR /root/code
RUN git clone https://github.com/ompl/ompl.git && cd ompl && mkdir build && cd build && cmake .. && make -j4 && make install

# Build and install SBPL
WORKDIR /root/code
RUN git clone https://github.com/sbpl/sbpl.git && cd sbpl && git checkout 1.3.1 && mkdir build && cd build && cmake .. && make -j4 && make install

# Set up Node.js 15.x repo
RUN curl -sL https://deb.nodesource.com/setup_15.x | bash -
RUN \
  DEBIAN_FRONTEND=noninteractive apt-get install -y \
    nodejs

# Install ipywidgets for Jupyter Lab
RUN jupyter labextension install @jupyter-widgets/jupyterlab-manager

# Build and install main repo
WORKDIR /root/code
COPY . /root/code/pytorch-motion-planner
RUN mkdir -p /root/code/pytorch-motion-planner/notebooks/benchmark/plots

WORKDIR /root/code/pytorch-motion-planner
RUN python3.9 setup.py install
RUN git submodule update --init --recursive --remote --progress 

RUN chmod u+x benchmark/third_party/bench-mr/scripts/build.sh
RUN bash benchmark/third_party/bench-mr/scripts/build.sh

RUN rm -rf build && mkdir build
WORKDIR /root/code/pytorch-motion-planner/build
RUN cmake -DINSTALL_BENCHMARK=ON -DPYBIND11_PYTHON_VERSION=3.9 ..
RUN make -j4
RUN export PYTHONPATH=$PYTHONPATH:/root/code/pytorch-motion-planner/build/benchmark

WORKDIR /root/code/pytorch-motion-planner
# Use bash as default shell
SHELL ["/bin/bash", "-c"]

EXPOSE 8888

ENTRYPOINT ["jupyter", "lab", "--ip=0.0.0.0", "--allow-root", "--no-browser", "--NotebookApp.token=''"]
