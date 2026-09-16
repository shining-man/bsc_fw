# Public WebUI assets

The shared appearance and navigation of the public firmware WebUI are maintained
in these source files:

- `public-ui.css`: layout, colors, navigation, dashboard, and responsive styles
- `public-ui.js`: navigation entries, labels, theme handling, and mobile behavior
- `web-settings-base.css`: unchanged legacy structure of generated settings forms
- `web-settings.css`: public-theme styles specific to settings forms
- `web-settings.js`: saving, validation, and clipboard behavior
- `web-settings-timer.js`: periodic data updates with build-time placeholders

Do not edit `include/web/publicWebUi.h` or `include/web/webSettingsWebUi.h`.
They are generated, compact firmware assets and are deliberately excluded from Git.

PlatformIO regenerates the header automatically before a firmware build. It can also be generated manually from the repository root:

```sh
python3 scripts/build_public_webui.py
```

The placeholders `__PUBLIC_UI_LANGUAGE__`, `__PUBLIC_UI_HAS_IO__`,
`__WEB_SETTINGS_TIMER_HANDLER__`, and `__WEB_SETTINGS_TIMER_INTERVAL__` are
replaced with firmware build values by the generator.
