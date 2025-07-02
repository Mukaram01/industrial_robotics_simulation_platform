# AGENTS.md

## Coding style
- Follow PEP8 for Python code.
- Install dependencies before running checks:
  ```bash
  pip install -r requirements.txt
  ```
  Alternatively, run `scripts/setup_and_test.sh` to install dependencies and
  execute `flake8` and `pytest` automatically.
- Use the repository's `.flake8` configuration. Run:
  ```bash
  flake8 src tests
  ```
  before committing.

## Testing
- After installing dependencies, run the unit tests with `pytest` from the repository root:
  ```bash
  pytest
  ```
  Ensure all tests pass before committing.

## Commit messages
- Keep commit titles short and use imperative mood, e.g., "Add robot controller".
- Provide a brief description of the change if necessary.
