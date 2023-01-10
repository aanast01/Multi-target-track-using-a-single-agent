function add_folders_to_path

if(isunix)   %just to use the right symbol for the path
    symb='/';
else
    symb='\';
end

wd = pwd;
cd ..
base = pwd;
cd(wd)

% Add that folder plus all subfolders to the path.
addpath(genpath(base));