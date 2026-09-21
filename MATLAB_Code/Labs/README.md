# Labs — 12061 / 12601 Control Systems

One folder per teaching week. Each week's folder holds that week's MATLAB
script *and* the data it produces, so a week is self-contained: hand a
student `week5/` and everything they need is in it.

```
Labs/
  week3/   Control_System_Lab1.m
  week4/   Control_System_Lab2.m        + Lab2_Data/  (created on first run)
  week5/   Control_System_Lab3.m        + data/
  week6/   Control_System_Lab4.m        + data/
  week7/ week8/ week10/ ... week13/     placeholders
```

## Weeks

Labs run from week 3, and the lab number trails the week number by two:
**Lab N is in `week(N+2)`.**

| Folder | Script | Topic |
|---|---|---|
| `week3`  | `Control_System_Lab1.m` | Link and command check |
| `week4`  | `Control_System_Lab2.m` | Signals, scaling, time response |
| `week5`  | `Control_System_Lab3.m` | Open-loop instability; measure the unstable pole |
| `week6`  | `Control_System_Lab4.m` | PID control (simulation + rig) |
| `week7`  | — | not written yet |
| `week8`  | — | not written yet |
| `week10` | — | not written yet |
| `week11` | — | not written yet |
| `week12` | — | not written yet |
| `week13` | — | not written yet |

**There is deliberately no `week9`** — that week is class-free. Do not
"fix" the gap by creating it.

## Where the data goes

Every script writes to a folder **next to itself**, located through the MATLAB
search path:

```matlab
labFolder = '';
onPath = which('Control_System_Lab4.m');      % the name of YOUR script
if ~isempty(onPath) && ~startsWith(onPath,tempdir,'IgnoreCase',true)
    labFolder = fileparts(onPath);
end
if isempty(labFolder), labFolder = pwd; end
cfg.DATA_FOLDER = fullfile(labFolder,"data");
```

**Do not use `mfilename("fullpath")` here.** It is reliable inside a *function*,
but these labs are run one section at a time with Ctrl+Enter, and MATLAB
executes Run Section (and Evaluate Selection) from a **temporary copy** of the
text. `mfilename` then returns `%TEMP%\Editor_xxxxx`, and every capture lands in
a folder Windows later deletes — silently, so you find out after the lab. That
is exactly what happened in week 6. `which` goes through the search path
instead, and `setupPath` puts every `Labs/week*` folder on it; the temp folder
is never on it.

Nor should you use a bare relative path. The scripts used to say
`cfg.DATA_FOLDER = "Lab3_Data"`, resolved against whatever folder MATLAB
happened to be pointing at, which is why those folders kept appearing in the
toolbox root instead of beside the lab.

Copy the pattern above into any new week's script.

Files are named `<GROUP_ID>_<label>_<timestamp>.mat` / `.csv`, so students
must set `cfg.GROUP_ID` or they overwrite each other. These data folders are
gitignored — captures are the student's, not repository content.

## Adding a new week

1. Make the folder, e.g. `week7/`.
2. Put the script in it, with the `which`-based `labFolder` pattern above.
3. Run `setupPath` again. It discovers `Labs/week*` at run time, so it needs
   no edit.

## One thing to watch

`setupPath` puts every week folder on the search path, so two scripts with
the same file name in different weeks would shadow each other — MATLAB runs
whichever folder comes first and says nothing. `setupPath` checks for this
and warns. Keep the week number in the file name if you ever reuse a script.
