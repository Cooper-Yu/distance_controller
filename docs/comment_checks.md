# Documentation Checks and Problems

Run `doxygen docs/Doxyfile.check` from the package root. The workspace documentation task is manually triggered; it is not an automatic save hook or a colcon build gate.

The check covers missing class/struct/member documentation (including private functions and variables), incorrect parameter names, and missing parameter/return descriptions within Doxygen's extraction capabilities. Exit code zero alone does not imply zero warnings. Warnings are sent to the terminal for a Problems matcher; XML is generated under `docs/generated/comment-check`.

Doxygen does not verify English usage, design rationale, semantic correctness, or completed TODOs. Ordinary comments are not structured documentation. This package keeps Doxygen source comments only. If the ROS workspace is the editor root, merge the task there with the package as its working directory; do not overwrite other tasks. GUI behavior needs separate verification.

## Function documentation pattern

Use `@brief`, `@par` with a short title and behavioral explanation, actual `@param` entries, an applicable `@return`, and `@note`. Keep `@note None at present.` when there is no additional note; TODO means unfinished work. Do not invent parameters or return values for void functions. Parameter names must match the signature, without decorative asterisks.

```cpp
/**
 * @brief Handle waiting and switching after a segment completes.
 *
 * @par Wait and switch
 * If the current segment is not completed, return false to continue tracking it.
 * Otherwise, keep the robot stopped until the waiting time has elapsed.
 * Then prepare the next segment, or shut down the node after the final segment.
 *
 * @param[in] current_time Node time captured by on_timer(); read with
 * dwell_start_time_ to measure dwell without modifying the caller's value.
 * @return False to continue tracking; true to end the current timer callback.
 * @note Publishes zero, updates segment state, and may shut down ROS.
 */
```

## Parameter direction and provenance

| Direction | Meaning | Required explanation |
| --- | --- | --- |
| `[in]` | Read caller-provided data | Calling function, source variable/field, and purpose |
| `[out]` | Write a result without relying on its incoming value | Destination variable/field, units, downstream consumer, and validity conditions |
| `[in,out]` | Read and update caller-visible data | Which fields are read and which are written back |

For example, `on_timer()` creates `data` and fills pose, errors, and PID interval. `compute_and_publish_command()` reads errors/yaw/interval and writes six velocity fields into that same object. `log_control_state()` then reads it. Update documentation when this call chain changes.

Reference syntax alone does not determine direction. Changing a member is a side effect, not an output through an input parameter. Zero-initialized output storage is not a computed result. Document `@return` and output references separately, including the false-return validity condition for `pid_dt`.

## Extraction and rendering

Both HTML and check configurations explicitly enable `EXTRACT_PRIVATE` and `EXTRACT_STATIC`, while retaining `EXTRACT_ALL=NO` for missing-documentation diagnostics. Check actual generated pages as well as warning counts: the previous HTML configuration hid private members even though the check included them.

The flow page uses a locally generated Graphviz SVG and `IMAGE_PATH`; a Mermaid fence alone was rendered as text by the current Doxygen setup. Verify the image reference and output file, then inspect the browser display. Editor Mermaid previews and Doxygen rendering are separate capabilities.

Doxygen 1.9.1 was verified on Ubuntu-22.04. Other environments require their own checks. This documentation workflow does not repair native clang-tidy diagnostics or prove robot behavior.
