FROM tazlogic/open-mower-base as fetch

ENV DEBIAN_FRONTEND=noninteractive

COPY --link ./ /opt/open_mower_ros
#RUN git clone https://github.com/ClemensElflein/open_mower_ros /opt/open_mower_ros

WORKDIR /opt/open_mower_ros

RUN git submodule update --init --recursive

# Fetch the repo and assemble the list of dependencies. We will pull these in the next step and actually run install on them
# If the package list is the same as last time, the apt install step is cached as well which saves a lot of time.
# Since the list gets sorted, it will be the same each time and the cache will know that by file checksum in the COPY command.
# We can't use this stage as base for the next, because this stage is run every time and would therefore invalidate the cache of the next stage.
FROM fetch as dependencies

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /opt/open_mower_ros

# This creates the sorted list of apt-get install commands.
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src --simulate | \
    sed --expression '1d' | sort | tr -d '\n' | sed --expression 's/  apt-get install//g' > /apt-install_list


# We can't derive this from "dependencies" because "dependencies" will be rebuilt every time, but apt install should only be done if needed
FROM tazlogic/open-mower-base as assemble

ENV DEBIAN_FRONTEND=noninteractive

#Fetch the slic3r built earlier, this only changes if slic3r was changed (probably never)
COPY --link --from=tazlogic/open-mower-slic3r /opt/prebuilt/slic3r_coverage_planner /opt/prebuilt/slic3r_coverage_planner

#Fetch the list of packages, this only changes if new dependencies have been added (only sometimes)
COPY --link --from=dependencies /apt-install_list /apt-install_list
RUN apt-get update && \
    apt-get install --no-install-recommends --yes $(cat /apt-install_list) && \
    rm -rf /var/lib/apt/lists/*

# This will already have the submodules initialized, no need to clone again
COPY --link --from=fetch /opt/open_mower_ros /opt/open_mower_ros

#delete prebuilt libs (so that they won't shadow the preinstalled ones)
RUN rm -rf /opt/open_mower_ros/src/lib/slic3r_coverage_planner /apt-install_list

#RUN git clone https://github.com/ClemensElflein/open_mower_ros /opt/open_mower_ros

WORKDIR /opt/open_mower_ros

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && cd /opt/open_mower_ros/src && catkin_init_workspace && cd .. && source /opt/prebuilt/slic3r_coverage_planner/setup.bash && catkin_make -DCATKIN_BLACKLIST_PACKAGES=slic3r_coverage_planner"

COPY .github/assets/openmower_entrypoint.sh /openmower_entrypoint.sh
COPY .github/assets/start.sh /start.sh
RUN chmod +x /openmower_entrypoint.sh && chmod +x /start.sh

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "source /opt/open_mower_ros/devel/setup.bash" > /root/.bashrc

ENTRYPOINT ["/openmower_entrypoint.sh"]
#CMD ["bash", "-c", "service nginx start; service mosquitto start; roslaunch open_mower open_mower.launch --screen"]
CMD ["bash", "-c", "/start.sh"]
