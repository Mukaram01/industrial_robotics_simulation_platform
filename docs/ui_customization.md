# User Interface Customization

The web dashboard is built with standard Flask templates located in
`src/web_interface_frontend/templates`. Each HTML file corresponds to a page that
is served by the backend. You can edit these templates to change the layout or
style of individual views.

## Template Overview

- **`base.html`** – Shared layout with the navigation bar and placeholders for
  page content.
- **`index.html`** – Landing page that links to the rest of the dashboard.
- **`dashboard.html`** – Displays system status, charts and detected objects.
- **`control.html`** – Robot control panel for manual commands.
- **`editor.html`** – Three.js scene used to edit simulation scenarios.
- **`log.html`** – Table showing the action log from the backend.
- **`login.html`** – Simple login form when authentication is enabled.

## Customization Tips

1. **Styling** – Place additional CSS files under `static/` and reference them
   from the templates as needed.
2. **JavaScript** – Behaviour for each page is defined in files under
   `static/`. For example, `dashboard.js` handles live updates on the dashboard
   page.
3. **Extending Pages** – Jinja2 blocks defined in `base.html` allow child
   templates to inject scripts, extra CSS or content sections. Use these blocks
   to add widgets to any page.

After modifying the templates, rebuild the `web_interface_frontend` package
using `colcon build --packages-select web_interface_frontend` so the updated
files are installed.
