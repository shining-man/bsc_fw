# Public WebUI assets

The shared appearance and navigation of the public firmware WebUI are maintained
in these source files:

- `public-ui.css`: layout, colors, navigation, dashboard, and responsive styles
- `public-ui.js`: navigation entries, labels, theme handling, and mobile behavior

Do not edit `include/web/publicWebUi.h`. It is a generated, compact firmware
asset and is deliberately excluded from Git.

PlatformIO regenerates the header automatically before a firmware build. It can
also be generated manually from the repository root:

```sh
python3 scripts/build_public_webui.py
```

The placeholders `__PUBLIC_UI_LANGUAGE__` and `__PUBLIC_UI_HAS_IO__` in the
JavaScript source are replaced with firmware build values by the generator.
