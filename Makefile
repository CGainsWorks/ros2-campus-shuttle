# Convenience wrappers. Everything here is also a plain docker compose command.
.PHONY: help demo bench comparison test calibration sim gazebo-comparison readme-check clean

help:
	@echo "make demo              run everything and verify the artifacts"
	@echo "make comparison        just the controller comparison"
	@echo "make test              run every check that does not need ROS"
	@echo "make calibration       reproduce the camera-to-LiDAR calibration"
	@echo "make sim               interactive shell in the ROS 2 Jazzy + Gazebo Harmonic image"
	@echo "make gazebo-comparison Gazebo cross-check of the comparison (three repeats)"
	@echo "make readme-check      check the README's numbers against the results"
	@echo "make clean             remove generated results"

demo bench:
	docker compose run --rm bench

comparison:
	docker compose run --rm comparison

test:
	docker compose run --rm test

calibration:
	docker compose run --rm calibration

sim:
	docker compose --profile sim run --rm sim

gazebo-comparison:
	docker compose --profile sim run --rm gazebo-comparison

readme-check:
	python3 scripts/check_readme.py

# Keeps results/controller_comparison.json, which is committed.
clean:
	find results -mindepth 1 -maxdepth 1 ! -name controller_comparison.json -exec rm -rf {} +
	rm -rf build install log
