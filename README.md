# Hardware Project Template

General template repository for our hardware projects, especially one-off and self-contained projects like custom test equipment. This template assumes that the hardware is being designed in KiCad.

## Organization

Pretty self-explanitory, put hardware related files into the `hardware` folder, firmware in the `firmware` folder. Other folders can be added as needed: for example a `media` folder is handy for storing pictures related to a project which can be used in documentation.

## Conventions

Partly for consistency, partly for functionality; please follow the following conventions:

- Try to keep folder names to single words and all lowercase
- Use underscores to separate words in names (e.g. `board_rev1`). **DO NOT USE SPACES! ESPECIALLY FOR FOLDERS CONTAINING CODE!** Spaces in file paths may cause issues in compilers and such so best to avoid them entirely.
