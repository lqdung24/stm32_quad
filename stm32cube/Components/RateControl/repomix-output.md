This file is a merged representation of the entire codebase, combined into a single document by Repomix.
The content has been processed where content has been compressed (code blocks are separated by ⋮---- delimiter).

# File Summary

## Purpose
This file contains a packed representation of the entire repository's contents.
It is designed to be easily consumable by AI systems for analysis, code review,
or other automated processes.

## File Format
The content is organized as follows:
1. This summary section
2. Repository information
3. Directory structure
4. Repository files (if enabled)
5. Multiple file entries, each consisting of:
  a. A header with the file path (## File: path/to/file)
  b. The full contents of the file in a code block

## Usage Guidelines
- This file should be treated as read-only. Any changes should be made to the
  original repository files, not this packed version.
- When processing this file, use the file path to distinguish
  between different files in the repository.
- Be aware that this file may contain sensitive information. Handle it with
  the same level of security as you would the original repository.

## Notes
- Some files may have been excluded based on .gitignore rules and Repomix's configuration
- Binary files are not included in this packed representation. Please refer to the Repository Structure section for a complete list of file paths, including binary files
- Files matching patterns in .gitignore are excluded
- Files matching default ignore patterns are excluded
- Content has been compressed - code blocks are separated by ⋮---- delimiter
- Files are sorted by Git change count (files with more changes are at the bottom)

# Directory Structure
```
Inc/
  rate_control.h
Src/
  rate_control.c
```

# Files

## File: Inc/rate_control.h
```c
} RateControlAxis;
⋮----
} RatePidConfig;
⋮----
} RateControlConfig;
⋮----
} RateControlDebug;
⋮----
} RateControl;
⋮----
bool RateControl_Init(RateControl *control,
⋮----
void RateControl_Reset(RateControl *control);
void RateControl_SetCommand(RateControl *control,
⋮----
bool RateControl_Update(RateControl *control,
⋮----
bool RateControl_GetDebug(const RateControl *control,
⋮----
#endif /* RATE_CONTROL_H */
```

## File: Src/rate_control.c
```c
static bool config_valid(const RateControlConfig *config);
static float clampf(float value, float minimum, float maximum);
static void clear_dynamic_state(RateControl *control);
static float update_axis(RateControl *control,
⋮----
bool RateControl_Init(RateControl *control,
⋮----
void RateControl_Reset(RateControl *control)
⋮----
void RateControl_SetCommand(RateControl *control,
⋮----
bool RateControl_Update(RateControl *control,
⋮----
bool RateControl_GetDebug(const RateControl *control,
⋮----
static bool config_valid(const RateControlConfig *config)
⋮----
static float clampf(float value, float minimum, float maximum)
⋮----
static void clear_dynamic_state(RateControl *control)
⋮----
/* Derivative-on-measurement avoids a kick when the pilot moves the stick. */
⋮----
/*
   * Conditional integration: do not wind the integrator farther into an
   * output limit. Integration in the direction that leaves saturation is
   * still allowed.
   */
```
