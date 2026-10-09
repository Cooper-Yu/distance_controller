# Template Sources and Conventions

| Component | Foundation | Project adaptation |
| --- | --- | --- |
| Package layout | ROS2 ros2 pkg create | Engineering support around learner implementation |
| C++ formatting | ament_clang_format Humble | Definition spacing; clang-format 14+ |
| C++ naming/size | clang-tidy | snake_case, CamelCase, 60-line review threshold |
| Python | PEP 8 / Ruff | 100 columns, single quotes, statement/branch checks; not identical to every ament rule |
| YAML/XML | Prettier, yamllint, Red Hat XML | Formatting and validation entry points |
| Documentation | Doxygen Awesome v2.3.4 | Pinned CSS, package/interface navigation |
| Flow/navigation | Graphviz SVG, Mermaid in compatible viewers, Obsidian Canvas | Real file links and implementation-derived flow |
| Debug/tests | cppdbg/debugpy, colcon, GoogleTest/pytest | Requires real configuration and meaningful cases |

Original source URLs and SHA256 values are in `docs/template_sources.json`. The ament style license is in `docs/_theme/ament-style-LICENSE`; the Doxygen Awesome MIT license is in `docs/_theme/LICENSE`. Preserve these third-party notices. They do not select the package's own code license.

The original scaffold metadata records template 1.1.1; later project changes are reviewed separately. This combines established tools and is not an official complete ROS2 course template.
