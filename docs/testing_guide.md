# Testing Guide

This document explains how to run and extend the unit tests for the Industrial Robotics Simulation Platform.

## Prerequisites

Install the Python dependencies listed in `requirements.txt`:

```bash
pip install -r requirements.txt
```

The tests use lightweight stubs so a full ROS installation is not required.

## Running the Test Suite

Execute the entire suite from the repository root with:

```bash
pytest
```

This will discover all tests under the `tests/` directory. Before committing changes, run the style checks as well:

```bash
flake8 src tests
```

## Test Directory Layout

All test files reside in the `tests/` folder. Each file targets a specific module or ROS 2 node. The suite is designed to run quickly without external processes.

## Adding New Tests

Place new test modules in `tests/` following the existing naming convention. Tests should avoid requiring a running ROS 2 environment. Use the helper utilities in `tests/test_utils.py` to create any necessary stubs.
