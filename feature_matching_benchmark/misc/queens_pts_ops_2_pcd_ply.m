clear;clc;

% Set data directories
model_dir = '/home/thso/workspace/datasets/queens/Models';
scene_dir = '/home/thso/workspace/datasets/queens/LIDAR_Point_Clouds';

% All files
pts_ops_files = {};
num_pts_files = 0;

% Find model files
[stat ops_files_str] = unix(['find ' model_dir ' -name *.ops']);
while length(ops_files_str) > 0
    [token ops_files_str] = strtok(ops_files_str);
    pts_ops_files = [pts_ops_files token];
    %num_pts_files = num_pts_files + 1;
end

 % Find scene files
% [stat pts_files_str] = unix(['find ' scene_dir ' -name *.pts'])
% while length(pts_files_str) > 0
%     [token pts_files_str] = strtok(pts_files_str);
%     pts_ops_files = [pts_ops_files token];
%     num_pts_files = num_pts_files + 1;
% end

for i = 1:numel(pts_ops_files)
    f = pts_ops_files{i};
    f_out_base = f(1:end-4);
    
    fprintf('(%d/%d) %s --> %s...\n', i, numel(pts_ops_files), f, [f_out_base '.pcd|ply']);
    X = read_pts_ops(f);
    
    write_pcd([f_out_base '.pcd'], X);
    
    if i <= num_pts_files
        fprintf('\tPTS file found, reconstructing mesh using 2D Delaunay triangulation...\n');
        tri = pointcloud2mesh(X, [0 0 -1]);
        write_ply([f_out_base '.ply'], X, tri);
    else % OPS file, only write xyz+normal data
        fprintf('\tOTS file found. NOTE: Saving only points and normals!\n');
        write_ply([f_out_base '.ply'], X);
    end
end

disp 'All done!';
