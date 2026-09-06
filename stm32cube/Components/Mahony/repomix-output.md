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
  mahony.h
Src/
  mahony.c
README.md
```

# Files

## File: Inc/mahony.h
```c
} Mahony_Config_t;
⋮----
} Mahony_Handle_t;
⋮----
} Mahony_Euler_t;
⋮----
void Mahony_Init(Mahony_Handle_t *filter, const Mahony_Config_t *config);
bool Mahony_InitFromAccel(Mahony_Handle_t *filter, float ax, float ay, float az);
bool Mahony_Update(Mahony_Handle_t *filter,
⋮----
bool Mahony_GetEulerDegrees(const Mahony_Handle_t *filter, Mahony_Euler_t *euler);
⋮----
#endif /* MAHONY_H */
```

## File: Src/mahony.c
```c
static float Mahony_Clamp(float value, float minimum, float maximum);
⋮----
void Mahony_Init(Mahony_Handle_t *filter, const Mahony_Config_t *config)
⋮----
bool Mahony_InitFromAccel(Mahony_Handle_t *filter, float ax, float ay, float az)
⋮----
bool Mahony_Update(Mahony_Handle_t *filter,
⋮----
bool Mahony_GetEulerDegrees(const Mahony_Handle_t *filter, Mahony_Euler_t *euler)
⋮----
static float Mahony_Clamp(float value, float minimum, float maximum)
```

## File: README.md
```markdown
# Mahony Attitude Estimator

The project-wide frames, quaternion conventions, and installed sensor mapping
are defined in [`../Attitude/README.md`](../Attitude/README.md) and
`../Attitude/Inc/attitude.h`.
```
