# web_interface_frontend

## Purpose
Static HTML, CSS and JavaScript assets for the simulation dashboard.

## Setup
Install or build the package so the assets can be served:

```bash
colcon build --packages-select web_interface_frontend
source install/setup.bash
```

## Usage
Files are served automatically by the Flask server in `web_interface_backend`.

## Extension
Modify the resources under `static/` and `templates/` to customise the web interface.
