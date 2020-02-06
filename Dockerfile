ARG FROM_IMAGE=ros:melodic

# multi-stage for caching
FROM $FROM_IMAGE AS cache

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# copy overlay source
ENV OVERLAY_WS /opt/overlay_ws
RUN mkdir -p $OVERLAY_WS/src
WORKDIR $OVERLAY_WS
COPY ./ src/kalibr
# COPY ./kalibr.repos ./
# RUN vcs import src < kalibr.repos

# copy manifests for caching
WORKDIR /opt
RUN find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp \
    && find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp

# multi-stage for building
FROM $FROM_IMAGE AS build

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    python-catkin-tools \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# install CI dependencies
RUN apt-get update && apt-get install -q -y \
      ccache \
      lcov \
    && rm -rf /var/lib/apt/lists/*

# copy overlay manifests
ENV OVERLAY_WS /opt/overlay_ws
COPY --from=cache /tmp/overlay_ws $OVERLAY_WS
WORKDIR $OVERLAY_WS

# install overlay dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths \
        src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# copy overlay source
COPY --from=cache $OVERLAY_WS ./

# build overlay source
ARG OVERLAY_MIXINS="release ccache"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin init && \
    catkin config \
        --merge-devel \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
    && catkin build

# source overlay from entrypoint
RUN sed --in-place \
      's|^source .*|source "$OVERLAY_WS/devel/setup.bash"|' \
      /ros_entrypoint.sh