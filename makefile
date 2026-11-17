
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

rov-install: rov-install-ansible /usr/local/bin/openocd .venv

rov-install-ansible:
	# Need sudo here for when password authentication is on
	cd setup && sudo ansible-playbook \
		--connection=local \
		--inventory 127.0.0.1, \
		--limit 127.0.0.1 rov.yaml

/usr/local/bin/openocd:
	cd openocd && \
	./bootstrap && \
	./configure --enable-sysfsgpio --enable-bcm2835gpio --disable-werror && \
	make -j4 && \
	sudo make install && \
	make clean

.venv:
	uv venv --system-site-packages
	uv sync 
	
rov:
	. ./install/local_setup.sh && \
		ros2 launch seahawk rov.launch.py

deck:
	. ./install/local_setup.sh && \
		ros2 launch seahawk deck.launch.py

clean:
	-rm -rf build install log 