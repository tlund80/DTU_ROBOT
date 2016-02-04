clc;clear;close all;

% Flags
output_pcd = 0; % Output PCD files to pcd_output_dir
output_pose = 0; % Output GT pose files to pose_output_dir
output_rgbd = 0; % Output RGB-D files to rgbd_output_dir

% Split sequence into segments of this size
segment_size = 5;

% Setup
rgbd_dir = '~/workspace/datasets/rgbd/rgbd-scenes'; % Image dir
pose_dir = '~/workspace/datasets/rgbd/rgbd-scenes_aligned'; % Pose dir
pcd_output_dir = '~/workspace/datasets/rgbd/scenes';
pose_output_dir = '~/workspace/datasets/rgbd/ground-truth';
rgbd_output_dir = '~/workspace/datasets/rgbd/scenes_rgbd';
% [desk kitchen_small meeting_small table table_small]
% [1-3     1                1         1      1-2     ]
rgbd_seq = 'table_small';
rgbd_seq_num = '2';

% Make sure output dirs exist
if ~exist(pcd_output_dir, 'dir'), mkdir(pcd_output_dir); end
if ~exist(pose_output_dir, 'dir'), mkdir(pose_output_dir); end
if ~exist(rgbd_output_dir, 'dir'), mkdir(rgbd_output_dir); end

% Open pose file
pose_file = [pose_dir '/' rgbd_seq '/' rgbd_seq '_' rgbd_seq_num '/' rgbd_seq '_' rgbd_seq_num '.pose'];
fidpose = fopen(pose_file);
assert(fidpose ~= -1);

% Get poses
Ts = [];
tline = fgetl(fidpose);
while ischar(tline)
    % Skip stamps
    [t tline] = strtok(tline);
    [t tline] = strtok(tline);
    % Rotation quaternion
    [w tline] = strtok(tline);
    [qx tline] = strtok(tline);
    [qy tline] = strtok(tline);
    [qz tline] = strtok(tline);
    % Translation
    [x tline] = strtok(tline);
    [y tline] = strtok(tline);
    [z tline] = strtok(tline);
    % To doubles and matrix
    t = str2num([x ' ' y ' ' z]);
    q = str2num([w ' ' qx ' ' qy ' ' qz]);
    T = [quat2dcm(q)' t' ; 0 0 0 1];
    % NOTE: We take the inverse to be able to place the first scene in a pair into the second
%     T = [T(1:3,1:3)' -T(1:3,1:3)'*t' ; 0 0 0 1];
    % Store
    Ts = [Ts ; T];
    
    % Iterate
    tline = fgetl(fidpose);
end

% Number of images
num_views = size(Ts, 1) / 4;

fprintf('Processing RGB-D Scene sequence %s-%s (%i scenes)...\n', rgbd_seq, rgbd_seq_num, num_views);

% Get file list
list = dir([rgbd_dir '/' rgbd_seq '/' rgbd_seq '_' rgbd_seq_num '/*depth.png']);
assert(numel(list) == num_views);
% [stat list] = unix(['ls -v ' rgbd_dir '/' rgbd_seq '/' rgbd_seq '_' rgbd_seq_num '/*depth.png']);
% assert(stat == 0);

% Put file list in a cell array and sort them ascending by frame index
files = {};
frame_indices = []; % For sorting
for i = 1:num_views
    % Get filename (no path)
    fname = list(i).name;
    files = [files  {fname}];
    % Get the frame index
    parts = strsplit(fname, '_');
    % For 'desk' and 'table':
    %   [sequence_name sequence_index frame_index depth.png]
    % For the others:
    %   [sequence_name1 sequence_name2 sequence_index frame_index depth.png]
    if numel(parts) == 4
        frame_index = str2double(parts(3));
    elseif numel(parts) == 5
        frame_index = str2double(parts(4));
    else
        error(['Invalid file: ' fname '!']);
    end
    frame_indices = [frame_indices frame_index];
end

% Do the sorting
[sorted idx] = sort(frame_indices);
files_sorted = cell(1, numel(files));
for i=1:numel(files), files_sorted{i} = files{idx(i)}; end
files = files_sorted;

% Add full paths again
for i = 1:numel(files), files{i} = [rgbd_dir '/' rgbd_seq '/' rgbd_seq '_' rgbd_seq_num  '/' files{i}]; end

% Generate data
for i = 1:2*segment_size:num_views
    if i + segment_size > num_views, break; end % End of sequence
    
    fprintf('\tFrame %i --> %i:\n', i, i + segment_size);
    fprintf('\t\t%s\n', files{i})
    fprintf('\t\t%s\n', files{i+segment_size})
    
    % Output RGB-D frames
    if output_rgbd
        % First depth file
        [p base ext] = fileparts(files{i});
        copyfile(files{i}, [rgbd_output_dir '/' base ext]);
        
        % Second depth file
        [p base ext] = fileparts(files{i+segment_size});
        copyfile(files{i+segment_size}, [rgbd_output_dir '/' base ext]);
        
        % First RGB file
        file_start_rgb = strrep(files{i}, '_depth.png', '.png');
        [p base ext] = fileparts(file_start_rgb);
        copyfile(file_start_rgb, [rgbd_output_dir '/' base ext]);
        
        % Second RGB file
        file_end_rgb = strrep(files{i+segment_size}, '_depth.png', '.png');
        [p base ext] = fileparts(file_end_rgb);
        copyfile(file_end_rgb, [rgbd_output_dir '/' base ext]);
    end
    
    rowstart = 4 * (i - 1) + 1;
    rowend = rowstart + 4 * segment_size;
    worldTstart = Ts(rowstart:rowstart+3,:);
    worldTend = Ts(rowend:rowend+3,:);
    
    % Relative pose between the two frames
    endTstart = inv(worldTend) * worldTstart;
    
    % Save in pose output dir
    [pstart basestart ext] = fileparts(files{i});
    [pend baseend ext] = fileparts(files{i + segment_size});
    pose_file = [pose_output_dir '/' basestart '-' baseend '.xf'];
    if output_pose, dlmwrite(pose_file, endTstart, ' '); end

    % Load depth images
    dstart = imread(files{i});
    dend = imread(files{i + segment_size});
    % Convert to point cloud
    pcstart = depth2pc(dstart);
    pcend = depth2pc(dend);
    % Save point clouds in image dir
    pcfilestart = [pcd_output_dir '/' basestart '.pcd'];
    pcfileend = [pcd_output_dir '/' baseend '.pcd'];
    if output_pcd
        write_pcd(pcfilestart, pcstart);
        write_pcd(pcfileend, pcend);
    end
end

disp 'All done!'
