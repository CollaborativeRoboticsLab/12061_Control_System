# Week 8

No lab script here yet.

When you add one, put it in this folder and locate its data folder through the
MATLAB search path:

```matlab
labFolder = '';
onPath = which('Control_System_LabN.m');      % the name of YOUR script
if ~isempty(onPath) && ~startsWith(onPath,tempdir,'IgnoreCase',true)
    labFolder = fileparts(onPath);
end
if isempty(labFolder), labFolder = pwd; end
cfg.DATA_FOLDER = fullfile(labFolder,"data");
```

**Not `mfilename("fullpath")`.** These labs are run section by section with
Ctrl+Enter, and MATLAB executes Run Section from a temporary copy of the text,
so `mfilename` returns `%TEMP%\Editor_xxxxx` and the captures are written to a
folder Windows deletes. `which` goes through the search path, which the temp
folder is never on.

Then run `setupPath` again — it discovers `Labs/week*` automatically.

See `../README.md` for the folder convention and the week table.
