
.PHONY: all clean

all:
	colcon build --symlink

devbox:
	cd setup && ansible-playbook \
		--connection=local \
		--inventory 127.0.0.1, \
		--limit 127.0.0.1 devbox.yaml

clean:
	-rm -rf build install log 