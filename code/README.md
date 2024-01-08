# Code

## VS Code Setup

Configure the default build task:

1. `cmd-shift-p`
2. search for "Tasks: Configure Default Build Task"
3. select "cargo build && cargo objcopy"

Add a keybinding for the compile and upload task:

1. `cmd-shift-p`
2. search for "Preferences: Keyboard Shortcuts (JSON)"
3. add the following JSON snippet to the list of keybinding overrides (note the brackets repeated here)

```json
[
    {
        "key": "cmd+shift+u",
        "command": "workbench.action.tasks.runTask",
        "args": "rust: upload firmware"
    }
]
```

Now, you can build and upload firmware with keyboard shortcuts:

- `cmd-shift-b` will compile the firmware
- `cmd-shift-u` will compile and upload the firmware
