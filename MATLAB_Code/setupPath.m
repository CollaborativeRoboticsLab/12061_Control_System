function setupPath()
%SETUPPATH Put the pendulum toolbox folders on the MATLAB search path.
%
%   setupPath
%
%   Run this once per MATLAB session, from anywhere. It works out where it
%   lives and adds the folders relative to itself, so it does not care what
%   MATLAB's current directory is. To make it automatic, add this line to
%   your startup.m:
%
%       run('C:\...\Control_System_lab_matlab\setupPath.m')
%
%   WHAT GETS ADDED
%     pendulum/      the working toolbox for the rig
%     experiments/   hardware tests and the friction sweep
%     diagnostics/   only needed when something is wrong
%     bluepill/      the older toolkit, for the other board
%     simulink/      the model builders
%     Labs/          Lab1 and Lab2, which are not tied to one week
%     Labs/week*/    every weekly lab folder it can find, discovered at
%                    run time -- add week14 later and it is picked up with
%                    no edit here
%
%   _archive is deliberately left OFF the path: those files are kept for
%   reference and must never shadow a live function.
%
%   It also checks for the same .m file name appearing in two folders it
%   added. That matters here: the weekly labs were moved out of Labs/ into
%   Labs/week3/, and a leftover copy of the old file would silently shadow
%   the new one. MATLAB picks whichever folder comes first on the path and
%   says nothing, so you would edit one file and run another.

root = fileparts(mfilename('fullpath'));

fixed = {'pendulum','experiments','diagnostics','bluepill','simulink','Labs'};

added = strings(0,1);
for k = 1:numel(fixed)
    p = fullfile(root,fixed{k});
    if isfolder(p)
        addpath(p);
        added(end+1,1) = string(fixed{k});          %#ok<AGROW>
    end
end

% Weekly lab folders, whatever happens to exist. Sorted numerically so the
% printed list reads week3, week4, ... week13 rather than week10, week11,
% week12, week13, week3 (which is what a plain alphabetical sort gives).
weekDirs = dir(fullfile(root,'Labs','week*'));
weekDirs = weekDirs([weekDirs.isdir]);
if ~isempty(weekDirs)
    names = string({weekDirs.name});
    num = double(regexp(names,'\d+','match','once'));
    [~,order] = sort(num);
    names = names(order);
    for k = 1:numel(names)
        addpath(fullfile(root,'Labs',names(k)));
        added(end+1,1) = "Labs/" + names(k);        %#ok<AGROW>
    end
end

addpath(root);

if isempty(added)
    warning("Pendulum:NotOrganised", ...
        "No subfolders found under %s. Is setupPath.m in the toolbox root?",root);
    return;
end

fprintf("Path set for: %s\n",strjoin(added,", "));

%% Shadowing check -- the failure mode that costs an afternoon
seen = containers.Map('KeyType','char','ValueType','any');
folders = [string(root); fullfile(root,added(:))];
for k = 1:numel(folders)
    f = dir(fullfile(folders(k),'*.m'));
    for j = 1:numel(f)
        key = lower(f(j).name);
        if isKey(seen,key)
            seen(key) = [seen(key); folders(k)];    %#ok<AGROW>
        else
            seen(key) = folders(k);
        end
    end
end

clashKeys = {};
ks = keys(seen);
for k = 1:numel(ks)
    if numel(seen(ks{k})) > 1
        clashKeys{end+1} = ks{k};                   %#ok<AGROW>
    end
end

if ~isempty(clashKeys)
    fprintf(2,"\nWARNING: the same file name exists in more than one folder\n");
    fprintf(2,"on the path. MATLAB will run whichever comes first and will\n");
    fprintf(2,"not tell you which. Delete the copy you do not want.\n\n");
    for k = 1:numel(clashKeys)
        where = seen(clashKeys{k});
        fprintf(2,"  %s\n",clashKeys{k});
        for j = 1:numel(where)
            fprintf(2,"      %s\n",where(j));
        end
    end
    fprintf(2,"\n");
else
    fprintf("No duplicate file names on the path.\n");
end

fprintf("\nFirst two lines of any pendulum session:\n");
fprintf("  s = connectPendulum(""COM4"");\n");
fprintf("  homeCart(s);          %% cart to mid-rail by hand, then Enter\n");
fprintf("\nBluepill board instead: connectSTM32(""COM4"",9600)\n");
end
