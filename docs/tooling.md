# Tooling

## Formatting and diagnostics

`.vscode/settings.json` selects formatters by language and enables format-on-save. C++ uses the ROS2 ament-based style: two spaces, 100 columns, definition braces on new lines, and blank lines between definitions (clang-format 14+). `.clang-tidy` checks naming and function size using a real compilation database; it does not automatically rename or split functions. Python uses Ruff, four spaces, single quotes, and separate statement/branch limits. YAML uses Prettier and yamllint; XML uses the XML extension. EditorConfig controls encoding and basic indentation.

Package-local `.vscode` settings apply when this package is opened as a workspace folder. Opening the parent ROS workspace does not automatically activate nested settings. Merge required entries into the actual editor root without overwriting existing tasks. The template's `--editor-root` option preserves existing files and reports conflicts. Existing parent language rules take precedence during template attachment.

## Commands

```bash
python3 tools/check_style.py --compile-db /absolute/workspace/build/distance_controller
doxygen docs/Doxyfile.check
doxygen docs/Doxyfile
```

The style script is read-only. Missing applicable tools or a compilation database report `NOT_READY`; selected naming/size findings fail the CLI check while editor diagnostics remain advisory. Record justified exceptions; do not compress code to hide warnings.

VS Code tasks provide build, test, result, and documentation entry points. Supply the real workspace path and load ROS and overlays in the target WSL environment first. C++ debugging requires a real built ELF path; Python debugging uses the active file. A launch configuration does not prove a breakpoint was hit.

## Runtime evidence

Use topics, rqt_graph, logs, rosbag2, PlotJuggler, and launch_testing when relevant. Record actual commands, QoS, readiness, time bounds, cleanup, and results. Templates do not install dependencies, configure meaningful tests, or supply behavioral acceptance evidence.

The current local normal-route regression uses external course fixtures and CSV capture. Package test planning remains in [test/README.md](../test/README.md). Detailed evidence is in the external course `code_lab/verification.md` under `/home/cooper/ros2_ws/training_notes/checkpoint18/robot_control_rosbot_xl/`.

The package Canvas uses package-root file paths. The separate Obsidian project Canvas uses vault-relative navigation notes linked to real WSL files; it does not duplicate the evidence ledger.
