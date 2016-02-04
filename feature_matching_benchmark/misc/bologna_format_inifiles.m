inipath = ['/home/andersgb1/workspace/datasets/bologna/'...
    'dataset2_scenes/3D_models/Stanford/Random/Dataset2_configFiles/'];

filepath = [inipath '/..'];
inifiles = dir([inipath '/*.ini']);

for i = 1:numel(inifiles)
    keys = inifile([inipath '/' inifiles(i).name], 'readall');

    number = str2num(keys{1,4});
    idx_scene = 0;
    for j = 1:size(keys, 1);
        if strcmp(keys{j,3}, 'path')
            idx_scene = j;
            break;
        end
    end
    assert(idx_scene > 0);
    scene = strrep(keys{idx_scene,4}, '"', '');
    scene = regexprep(scene, '\\', '/');
    scene = strrep(scene, '3D models', '3D_models');
    [p scene ext] = fileparts(scene);
    
    idx = 2;
    for j = 0:number-1
        model = strrep(keys{idx,4}, '"', '');
        model = regexprep(model, '\\', '/');
        model = strrep(model, '3D models', '3D_models');
        [p model ext] = fileparts(model);
        
        gtfile = strrep(keys{idx + 1,4}, '"', '');
        gtfile = regexprep(gtfile, '\\', '/');
        gtfile = strrep(gtfile, '3D models', '3D_models');
        [p gtfile ext] = fileparts(gtfile);
        gtfile_new_base = [scene '-' model];
        copyfile([filepath '/' gtfile ext], [filepath '/' gtfile_new_base ext]);
        
        idx = idx + 2;
    end
end

