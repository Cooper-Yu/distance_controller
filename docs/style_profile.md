# ROS2 Teaching Style Profile 1.1

The profile combines ROS2 ament C++ formatting, PEP 8/Ruff Python formatting, and Prettier/yamllint/XML configuration checks. Preserve established project conventions when attaching it to an existing package.

| Setting | Convention | Purpose |
| --- | --- | --- |
| Width | 100 columns and an editor ruler | Readable side-by-side review |
| C++ indentation | 2 spaces | ROS2 baseline |
| C++ braces | New line for definitions; same line for short control blocks | Distinguish definitions and control flow |
| Definitions | Blank lines; no collapsed short functions | Make responsibilities visible |
| Python | 4 spaces, single quotes, PEP 8 spacing | Consistent layout |
| YAML/XML | 2 spaces | Visible hierarchy |
| Naming | CamelCase classes, snake_case functions/variables, trailing underscore for private C++ members | Identify roles |
| Function size | C++ 60 lines and 4 nesting levels; Python statement/branch limits managed separately | Prompt responsibility review |

Line width, function length, and statement count are distinct. A ruler does not truncate strings. Size warnings prompt review rather than mechanical splitting.

## C++ example

```cpp
double limit_value(double value, double upper)
{
  if (value > upper) {
    return upper;
  }
  return value;
}
```

Formatting changes spacing and layout, not business logic or interface meaning. This package retains only Doxygen source comments. Member defaults are declared in the header with braces; runtime updates remain assignments.

## Python and YAML examples

```python
class ErrorCalculator:
    def compute_error(self, target: float, measured: float) -> float:
        return target - measured

    def reached_goal(self, error: float, tolerance: float) -> bool:
        return abs(error) <= tolerance
```

Annotations and calculations are written by the author, not invented by the formatter.

```yaml
example_node:
  ros__parameters:
    enabled: true
```

This is a formatting example, not a controller parameter declaration. Validate actual parameter names, types, units, and loading behavior separately.

## Editor reading aids

| Setting | Effect |
| --- | --- |
| `editor.rulers = [100]` | Column guide |
| `editor.guides.indentation` | Indentation guides |
| Bracket-pair guides/colorization | Matching bracket structure |
| `editor.renderWhitespace = selection` | Visible selected whitespace |
| `editor.stickyScroll.enabled` | Keep enclosing definitions visible |
| UTF-8 / LF | Default encoding and line endings |

Fonts, theme, font size, and line height remain personal preferences. Keep standard JSON configuration valid; explain settings here rather than inserting unsupported JSON comments.

## Integration

Rules live in `.clang-format`, `.clang-tidy`, `pyproject.toml`, `.yamllint`, `.prettierrc.json`, `.prettierignore`, `.editorconfig`, and `.vscode/settings.json`. clang-tidy IgnoreMacros requires a supporting version; LLVM 23.1.0 was verified locally. Existing files are preserved on template attachment and conflicts require review.

Verify save-time formatting and diagnostics in the named environment. Follow formatting with review, debugging, and behavioral testing. Template metadata records the original scaffold version; it does not certify current tool readiness or runtime success.

## References

[ROS2 style](https://github.com/ros2/ros2_documentation/blob/humble/source/The-ROS2-Project/Contributing/Code-Style-Language-Versions.rst), [clang-format](https://clang.llvm.org/docs/ClangFormatStyleOptions.html), [Ruff](https://docs.astral.sh/ruff/configuration/), and [VS Code settings](https://code.visualstudio.com/docs/configure/settings).
