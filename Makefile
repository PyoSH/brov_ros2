.PHONY: shell up down build test packages check

shell:
	./docker/brov-shell

up:
	docker compose up -d

down:
	docker compose down

build: up
	docker compose exec brov bash /workspace/brov_ros2/docker/build_workspace.sh

test: build
	docker compose exec brov bash -lc 'source /workspace/brov_ros2/docker/ros_env.sh && cd "$${BROV_ROS_WS}" && colcon test --event-handlers console_direct+ && colcon test-result --verbose'

packages: build
	docker compose exec brov bash -lc 'source /workspace/brov_ros2/docker/ros_env.sh && ros2 pkg executables | grep "^brov_"'

check: build
	docker compose exec brov bash -lc 'source /workspace/brov_ros2/docker/ros_env.sh && cd /workspace/brov_ros2 && python3 docker/check_environment.py'
