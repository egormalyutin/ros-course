.PHONY: all clean reset-turtle

all:
	colcon build --symlink-install

clean:
	rm -rf log install build

reset-turtle:
	ros2 service call /reset std_srvs/srv/Empty