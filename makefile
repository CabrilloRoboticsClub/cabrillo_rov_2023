
.PHONY: all clean

all:
	colcon build --symlink

devbox:
	# Need sudo here for when password authentication is on
	cd setup && sudo ansible-playbook \
		--connection=local \
		--inventory 127.0.0.1, \
		--limit 127.0.0.1 devbox.yaml

rovsetup:
	# Need sudo here for when password authentication is on
	cd setup && sudo ansible-playbook \
		--connection=local \
		--inventory 127.0.0.1, \
		--limit 127.0.0.1 rov.yaml

clean:
	-rm -rf build install log 