# Week 4

No lab script here yet.

When you add one, put it in this folder and have it save its captures to
`week4/data/` by resolving the path from the script itself:

```matlab
labFolder = fileparts(mfilename("fullpath"));
if isempty(labFolder), labFolder = pwd; end
cfg.DATA_FOLDER = fullfile(labFolder,"data");
```

Then run `setupPath` again — it discovers `Labs/week*` automatically.

See `../README.md` for the folder convention and the week table.
