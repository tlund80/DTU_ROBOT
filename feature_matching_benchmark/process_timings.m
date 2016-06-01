clear;close all;clc;

% Output PDF dir
%figure_output_dir = '~/workspace/CARO-Publications/Publications/2015/XXX_Which3DFeatureShouldIUse/gfx'; % Comment to avoid
figure_output_dir = '~/DTU_ROBOT/feature_matching_benchmark/output/bologna2_figures/'; % Comment to avoid
figure_output_file_estimation = 'timings_estimation.pdf';

% Input dir
%data_dir = 'output/bologna1_timing';
data_dir = 'output/bologna2';


% Feature radius
radius_mul = '15';

% Decimation levels of the scenes
support_sizes = [83 164 243 318 395 466 538 610 681 751 820 892 957 1029 1100 1174 1246 1325 1405 1487];
label_x_estimation = 'Support size {\it N}';
% decimations = 0.01 * (5:5:100);
% label_x = 'Resolution';

% All features
features = {
    'ecsad'
    'fpfh'
    'ndhist'
    'rops'
    'shot'
    'si'
    'usc'
    'pfh'
    '3dsc'
};

% Get colors/markers etc.
[label_map, color_map marker_map dimension_map] = setup(false);
font_size = 20;

% Load overall feature timings for the scenes [s]
estimation_timings = dlmread([data_dir '/meta_feature_timings.txt']);

% Load overall match timings for the scenes [s]
match_timings = dlmread([data_dir '/meta_match_timings.txt']);
assert(size(estimation_timings,1) == size(match_timings,1));

% Divide by the number of features to get per-vertex estimation time [ms]
feature_numbers = dlmread([data_dir '/meta_feature_numbers.txt']);
feature_numbers_scenes = feature_numbers(:,2);
for i = 1:size(estimation_timings, 1)
    estimation_timings(i,:) = 1000 * estimation_timings(i,:) / feature_numbers_scenes(i);
    match_timings(i,:) = 1000 * match_timings(i,:) / feature_numbers_scenes(i);
end


% Show estimation timings
figure('Name', 'Feature estimation timings');
hold on;
legends = cell(1,numel(features));
for i = 1:numel(features)
    plot(support_sizes', estimation_timings(:,i)', marker_map(features{i}), 'linewidth', 2, 'color', [color_map(features{i})]);
    legends{i} = label_map(features{i});
end
legend(legends, 'location', 'northwest')
xlabel(label_x_estimation, 'fontsize', font_size);
ylabel('Mean estimation time [ms]', 'fontsize', font_size);
set(gca, 'fontsize', font_size);
hold off

% Present match timings
fprintf('Feature(dim)\t& Matching time [ms]\t& Speedup\n');
match_timings_mean = mean(match_timings, 1);
for i = 1:numel(features)
    fprintf('%s(%s)\t& %.3f\t\t\t& %.3f \\\\\n',...
        label_map(features{i}),...
        dimension_map(features{i}),...
        match_timings_mean(i),...
        match_timings_mean(end)/match_timings_mean(i));
end


% Save PDFs, if enabled
if exist('figure_output_dir') && length(figure_output_dir) > 0
    fprintf('Saving figure to %s...\n', [figure_output_dir '/' figure_output_file_estimation]);
    print(gcf, [figure_output_dir '/' figure_output_file_estimation], '-dpdf');
end