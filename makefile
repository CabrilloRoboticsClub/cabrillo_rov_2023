
.PHONY: all clean

all:
	if [ "$$(uname -m)" = "aarch64" ]; then touch src/seahawk_description/COLCON_IGNORE; fi
	colcon build --symlink

devbox:
	# Need sudo here for when password authentication is on
	cd setup && sudo ansible-playbook \
		--connection=local \
		--inventory 127.0.0.1, \
		--limit 127.0.0.1 devbox.yaml

rov:
	# Need sudo here for when password authentication is on
	cd setup && sudo ansible-playbook \
		--connection=local \
		--inventory 127.0.0.1, \
		--limit 127.0.0.1 rov.yaml

clean:
	-rm -rf build install log 