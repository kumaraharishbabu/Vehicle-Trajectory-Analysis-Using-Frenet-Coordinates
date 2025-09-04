# Vehicle Trajectory Analysis Using Frenet Coordinates

This project analyzes vehicle trajectories on curvy roads by transforming Cartesian coordinates (X, Y) into Frenet coordinates (longitudinal s, lateral d). It evaluates lane usage, lateral offsets, and road utility, providing insights for traffic consultancy and road safety.

## Files

- `data/` – CSV files with vehicle trajectory data (X, Y coordinates)
- `notebooks/trajectory_analysis.ipynb` – Jupyter notebook with full analysis and visualizations
- `matlab/trajectory_transform.m` – MATLAB file for coordinate transformation
- `scripts/frenet_transform.py` – Optional Python script for XY to Frenet conversion

## Features

- Transform vehicle trajectories from Cartesian to Frenet coordinates
- Compute lateral vs longitudinal offsets for multiple vehicles
- Visualize trajectories in lateral vs longitudinal plots
- Analyze lane usage and road utility
- Generate consultancy recommendations for road design and traffic management

## Requirements

- Python 3.x
- NumPy
- Pandas
- Matplotlib
- SciPy (optional)
- MATLAB (for `.m` file execution)

## License

This project is licensed under the MIT License.

