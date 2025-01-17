# Misc Scripts and Commands

> [!NOTE]
> All instructions are intended to be begin in the root directory of the
> `rover-Basestation-Release` repository.

## Submodule Updates

To update submodules to their latest versions, please use this command:

```bash
git submodule update --init --remote
```

This will update submodules, initializing them if they are not already. It will
also have them track the latest changes on GitHub (`--remote`).
