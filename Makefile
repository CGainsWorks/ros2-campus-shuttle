# Convenience wrappers. Everything here is also a plain docker compose command.
.PHONY: help demo bench test calibration sim gazebo-comparison clean

help:
	@echo "make demo              reproduce the headline controller comparison"
	@echo "make test              run every check that does not need ROS"
	@echo "make calibration       reproduce the camera-to-LiDAR calibration"
	@echo "make sim               interactive shell in the ROS 2 + Gazebo image"
	@echo "make gazebo-comparison Gazebo cross-check of the comparison"
	@echo "make clean             remove generated results"

demo bench:
	docker compose run --rm bench

test:
	docker compose run --rm test

calibration:
	docker compose run --rm calibration

sim:
	docker compose --profile sim run --rm sim

gazebo-comparison:
	docker compose --profile sim run --rm gazebo-comparison

clean:
	rm -rf results build install log
