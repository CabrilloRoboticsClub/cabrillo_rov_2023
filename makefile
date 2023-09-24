
.PHONY: all clean

all:
	if [ "$$(uname -m)" = "aarch64" ]; then touch src/seahawk_description/COLCON_IGNORE; fi
	colcon build --symlink

devbox-install:
	# Need sudo here for when password authentication is on
	cd setup && sudo ansible-playbook \
		--connection=local \
		--inventory 127.0.0.1, \
		--limit 127.0.0.1 devbox.yaml

rov-install:
	# Need sudo here for when password authentication is on
	cd setup && sudo ansible-playbook \
		--connection=local \
		--inventory 127.0.0.1, \
		--limit 127.0.0.1 rov.yaml

rov:
	. ./install/local_setup.sh && \
		ros2 launch seahawk_rov rov.launch.py

clean:
	-rm -rf build install log 