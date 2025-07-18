# Testing Guide

This guide explains how to run the automated tests and style checks for the Industrial Robotics Simulation Platform.

## 1. Install Dependencies

Install the Python packages and development tools listed in `requirements.txt`:

```bash
pip install -r requirements.txt
```
Python 3.10 is the recommended interpreter when running the test suite.

The tests use stub modules so ROS&nbsp;2 is not required. Ensure all dependencies are available in your environment before running the suite.

Some tests rely on optional packages like **NumPy** and **PyYAML**. If these packages are missing, those tests will be skipped automatically.

## 2. Run Flake8

Check code formatting with `flake8`:

```bash
flake8 src tests
```

## 3. Execute the Tests

Run all unit tests with coverage enabled from the repository root:

```bash
pytest --cov=src --cov-report=xml
```

Individual tests can be executed by passing a file path. Coverage is still
enabled automatically:

```bash
pytest --cov=src --cov-report=xml tests/test_core_logic.py
```

## 4. Test Structure

All test cases live in the `tests/` directory. Each file targets a specific package or ROS&nbsp;2 node and uses lightweight stubs so the suite can run without a full ROS installation.

When adding new features, place the corresponding tests in this folder and keep them independent of ROS whenever possible.


For additional details see the "Running Tests" section in the project README.
