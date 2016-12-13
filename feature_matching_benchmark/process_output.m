clc;clear;close all;

% Output PDFs
figure_output_dir = '~/DTU_ROBOT/feature_matching_benchmark/'; % Comment to avoid
%figure_output_dir = '~/DTU_ROBOT/feature_matching_benchmark/output/bologna/figures';

zoomed_plots = 1; % Use this for [uwa queens rgbd_scenes_0.125]
zoom_y_limits = [0 0.25]; % Only used if zoomed_plots == 1
zoom_y_ticks = 0:0.05:0.25; % Only used if zoomed_plots == 1

% Opts
use_pca = 0; % Use optionally for [bologna2]
use_fusion = 0; % Use fusion of features, not compatible with PCA
fusion_ternary = 0; % Use ternary instead of binary combinations for fusion
use_all_dists = 0; % Ignored for PCA and fusion
verbose = 1; % Enable terminal output and figures
legend_on = 1; % Guess what

% Fusion increased the performance :)
if use_fusion
    zoom_y_limits = [0 0.4]; % Only used if zoomed_plots == 1
    zoom_y_ticks = 0:0.1:0.4; % Only used if zoomed_plots == 1
end

% Bologna 1 outputs
% bologna_noise = '0.3'; % Noise [0.1 0.3] - use only with bologna1
% data_dir = ['output/bologna1_noise' bologna_noise];
% radius_mul = {'30','20','30','30','30','30','30'};

% Bologna 2 outputs (and PCA)
%data_dir = 'output/bologna2';
%radius_mul = {'30','20','30','30','30','30','30','20','30'};
%radius_mul = {'30','30','30','30','30'};

% Our outputs (and PCA)
data_dir = 'output/our/stl/final_data';
radius_mul = {'20','17.5','31','20','17.5','30','25','22.5'}; 
%radius_mul = {'30','12.5','31','20','17.5'}%,'15','15'}40

% Setting legend position for the pca case:
% set(findobj(gcf,'Type','axes','Tag','legend'), 'position', [0.6198 0.1510 0.4299 0.8765]);
% For the binary fusion case:
% set(findobj(gcf,'Type','axes','Tag','legend'), 'position', [0.5710 0.3574 0.4741 0.6920]);
% For the ternary fusion case:
% set(findobj(gcf,'Type','axes','Tag','legend'), 'position', [0.4687 0.3574 0.5895 0.6920]);

% UWA outputs
% data_dir = 'output/uwa';
% radius_mul = {'10','7.5','20','12.5','10','17.5','12.5'};

% Queens outputs
% data_dir = 'output/queens';
% radius_mul = {'10','7.5','22.5','12.5','12.5','20','12.5'};

% RGBD Scenes outputs
% data_dir = 'output/rgbd_scenes_0.125';
% radius_mul = {'25','17.5','25','20','22.5','25','25'};

% In PCA case, output dir has '_pca' suffix
if use_pca && use_fusion, error('Flags use_pca and use_fusion are exclusive!'); end
if use_pca, data_dir = [data_dir '_pca']; end
if use_fusion, data_dir = [data_dir '_fusion']; end

object_nr= 36;
objects = {
    '1-Small_house',
    '2-Pizza_house',
    '3-House_cocktailbar',
    '4-Yellow_house',
    '5-Angel',
    '6-Birds',
    '7-Cranium',
    '8-Rabbit',
    '9-Neutral',
    '10-Sanex',
    '11-Cup',
    '12-Sponge',
    '13-Tape',
    '14-Coke',
    '15-Pringles',
    '16-Filler',
    '17-Water_softner',
    '18-Hand_wash',
    '19-Lego',
    '20-Mounting_',
    '23-Emergency_button',
    '24-Water_trap',
    '25-Psu',
    '26-Pump_house',
    '28-Angle_bar',
    '31-White_box',
    '32-Power_plug_small_6703',
    '33-Power_plug_medium_6704',
    '34-Power_plug_large',
    '35-Power_plug_8656',
    '36-Power_plug_8657',
    '37-Thermostat',
    '38-Wheel',
    '39-Metal_triangle',
    '40-Orange_drainpipe',
    '41-Brake_disc',
    '42-White_plastic_part1',
    '43-White_plastic_part2',
    '44-Danfoss_cap',
    '45-Galvanized_box',
    '46-White_plastic_wheel',
    '47-Pipe1_block',
    '48-Pipe2',
    '50-Cobber_disc',
    '51-Galvanized_fork',
    'all'
};

% All features
features_all = {
    'ecsad',
    'fpfh',
    'ndhist',
    'rops',
    'shot',
     'si',
    'usc'
    %  'pfh'
 %   '3dsc'
};

% PCA features
features_pca = {
    'ecsad_pca99'
    'ecsad_pca95'
    'ecsad_pca90'
    'rops_pca99'
    'rops_pca95'
    'rops_pca90'
    'shot_pca99'
    'shot_pca95'
    'shot_pca90'
    'usc_pca99'
    'usc_pca95'
    'usc_pca90'
};

% Fusion features
features_fusion_binary = {
    % Binary
    'ecsad-ndhist'
	'ecsad-rops'
	'ecsad-shot'
	'ecsad-si'
	'ndhist-rops'
	'ndhist-shot'
	'ndhist-si'
	'rops-shot'
	'rops-si'
	'shot-si'
};

features_fusion_ternary = {
    % Ternary
    'ecsad-ndhist-rops'
    'ecsad-ndhist-shot'
    'ecsad-ndhist-si'
	'ecsad-rops-shot'
	'ecsad-rops-si'
	'ecsad-shot-si'
	'ndhist-rops-shot'
	'ndhist-rops-si'
	'ndhist-shot-si'
    'rops-shot-si'
};

features = features_all;
if use_pca
    features = features_pca;
    radius_mul = repmat({'30'}, [1 numel(features_pca)]); % TODO
elseif use_fusion
    if fusion_ternary, features = features_fusion_ternary; else features = features_fusion_binary; end
    radius_mul = repmat({'N/A'}, [1 numel(features)]);
end

% Distance metrics
% distances = {'L1' 'L1_RATIO' 'L2' 'L2_RATIO' 'LINF' 'LINF_RATIO', 'HELLINGER', 'CHISQ', 'EMD'};
if use_all_dists
    distances = {'L1', 'L2', 'LINF', 'HELLINGER', 'CHISQ', 'EMD'};
else
    distances = {'L2_RATIO'};
end
if use_pca || use_fusion || ~use_all_dists, distances = {'L2_RATIO'}; end

%% Plot niceness
font_size = 16;
font_size_legend = font_size;
if use_pca, font_size_legend = 14; else font_size_legend = font_size; end
curve_samples = 20;
[label_map, color_map, marker_map, dimension_map] = setup(use_pca);

%% Loop over all metrics
recall_max = 0;
zoomed_plot_ax = [];
match_timings_mean_all = [];
for distance = distances
    % Precision and recall generated from data, one column per feature
    precision = [];
    recall = [];
    auc = zeros(1, numel(features));

    % Loop over features and load precision and recall
    for i = 1:numel(features)
        if verbose, disp(['Processing "' features{i} '" at ' radius_mul{i} ' mr with ' distance{1} '...']); end

        % Get the file data
        if use_fusion
            fid = fopen([data_dir '/matching_output_' features{i} '_' distance{1} '.txt']);
        else
            fid = fopen([data_dir '/matching_output_' features{i} '_' radius_mul{i} '_' distance{1} '_' objects{object_nr} '_' '.txt']); %'_' objects{object_nr}
            [data_dir '/matching_output_' features{i} '_' radius_mul{i} '_' distance{1} '_' objects{object_nr} '_' '.txt']
        end
        if fid == -1, warning('No data found! Continuing...'), continue; end
        data_feature = fread(fid, inf, 'float');
        data_feature = [data_feature(1:2:end) data_feature(2:2:end)];
        fclose(fid);

        % Sort ascending according to feature distance (first column)
        data = sortrows(data_feature, 1);

        % Now generate precision and recall from the data
        rows = size(data,1);
        positives = rows;
        tp_cnt = 0; % Running counter of true positives
        pr_feature = zeros(rows, 2);
        for j = 1:rows
            tp_cnt = tp_cnt + data(j,2);
            pr_feature(j,1) = tp_cnt / j; % Precision
            pr_feature(j,2) = tp_cnt / positives; % Recall
        end

        % Store precision and recall
        precision = [precision pr_feature(:,1)];
        recall = [recall pr_feature(:,2)];
%         auc(i) = max(2 * pr_feature(:,1) .* pr_feature(:,2) ./ (pr_feature(:,1) + pr_feature(:,2)));
        auc(i) = trapz(pr_feature(:,2),pr_feature(:,1));
    end
    
    if isempty(precision), continue; end

    if curve_samples > 0
        if verbose, warning('Sampling precision-recall curves!'); end
        samples_idx = floor(linspace(0.01*size(precision,1), size(precision,1), curve_samples));
        recall = recall(samples_idx, :);
        precision = precision(samples_idx, :);
    end
    
    if max(recall(:)) > recall_max, recall_max = max(recall(:)); end

    %% Load meta
    feature_numbers_loaded = 0;
    if exist([data_dir '/meta_feature_numbers.txt'])
        feature_numbers = dlmread([data_dir '/meta_feature_numbers.txt']);
        feature_numbers_loaded = 1;
    end
    
    meta_loaded = 0;
    if exist([data_dir '/meta_feature_timings.txt']) &&...
            exist([data_dir '/meta_match_timings.txt'])
        feature_timings = dlmread([data_dir '/meta_feature_timings.txt']);
        match_timings = dlmread([data_dir '/meta_match_timings.txt']);
        meta_loaded = (numel(features) == size(feature_timings,2));
    end
    if ~meta_loaded, warning('Meta information not consistent with number of features - not shown!'); end

    % TODO: Remove left-out scenes
    if feature_numbers_loaded
        non_zero_rows = (feature_numbers(:,1) > 0);
        feature_numbers = feature_numbers(non_zero_rows,:);
        if meta_loaded
            match_timings = match_timings(non_zero_rows,:);
            feature_timings = feature_timings(non_zero_rows,:);
        end
    end

    %% Present meta, if verbose

    if verbose && feature_numbers_loaded && meta_loaded
        % Take mean timing per query point and scale to ms
        feature_timings = 1000 * feature_timings ./ repmat(feature_numbers(:,1), 1, numel(features));
        match_timings = 1000 * match_timings ./ repmat(feature_numbers(:,1), 1, numel(features));
        match_timings_mean_all = [match_timings_mean_all mean(match_timings)'];
        
        % Present
        disp 'AVERAGE PER-VERTEX TIMINGS AND STATS:';
        chars = fprintf('Feature\t\tEstimation [ms]\t\tMatching [ms]\t\tAUC\t\tRecall\n');
        fprintf('-------\t\t---------------\t\t-------------\t\t------\t\t------\n');
        for i = 1:numel(features)
            fprintf('%s\t\t%.3f\t\t\t%.3f\t\t\t%.3f\t\t%.3f\n',...
                features{i},...
                mean(feature_timings(:,i)),...
                mean(match_timings(:,i)),...
                auc(i),...
                recall(end,i));
        end
        fprintf('-------\t\t---------------\t\t-------------\t\t------\t\t------\n');
        fprintf('Average number of query/target features per scene: %d/%d\n\n',...
            round(mean(feature_numbers(:,1))),...
            round(mean(feature_numbers(:,2))));
        disp 'All estimation times:';
        fprintf('%f ', mean(feature_timings)');fprintf('\n\n');
%         disp(['All matching times (' distance{1} ')']);
%         fprintf('%f ', mean(match_timings)');fprintf('\n\n');
    end


    %% Present results, if enabled

    if verbose
        % Generate legends
        legends = cell(1,numel(features));
        for i = 1:numel(features), legends{i} = [label_map(features{i}) ' (' sprintf('AUC %.3f', auc(i)) ')']; end
        
        % Show full plot, if not zoomed enabled
        if ~zoomed_plots
            h = figure('name', ['Full plot (' distance{1} ')']); hold;
            for i = 1:numel(features)
                style = '';
                if curve_samples > 0, style = [style marker_map(features{i})]; end
               plot(1-precision(:,i), recall(:,i), style, 'linewidth', 2, 'color', color_map(features{i}));
            end
            xlim([0 1]), ylim([0 1]), set(gca, 'xtick', 0:0.1:1, 'ytick', 0:0.1:1)
            xlabel('1 - Precision', 'fontsize', font_size), ylabel('Recall', 'fontsize', font_size)
            % title(['Results for ' distance ' distance'])
            if legend_on, legend(legends, 'location', 'northeast', 'fontsize', font_size_legend); end
            set(gca, 'fontsize', font_size);
            
            if exist('figure_output_dir') && length(figure_output_dir) > 0
                %figure_output_file = [figure_output_dir '/' data_dir '_' distance{1}];
                figure_output_file = [figure_output_dir '/' data_dir '/' objects{object_nr} '_' distance{1}]
                figure_output_file = strrep(figure_output_file, '.', ''); % Dots are not handled well by pdflatex
                % Special case for fusion: two possibly graphs
                if use_fusion
                    if fusion_ternary
                        figure_output_file = strrep(figure_output_file, 'fusion', 'fusion_ternary');
                    else
                        figure_output_file = strrep(figure_output_file, 'fusion', 'fusion_binary');
                    end
                end
                fprintf('Saving full plot to %s...\n', [figure_output_file '.pdf']);
                print(h, figure_output_file, '-dpdf');
            end
        end

        if zoomed_plots
            figure('name', ['Zoomed plot (' distance{1} ')']), hold;
            for i = 1:numel(features)
                style = '-';
                if curve_samples > 0, style = marker_map(features{i}); end
                plot(1-precision(:,i), recall(:,i), style, 'linewidth', 2, 'color', color_map(features{i}));
            end
            xlim([0 1]), set(gca, 'xtick', 0:0.1:1)
            xlabel('1 - Precision', 'fontsize', font_size), ylabel('Recall', 'fontsize', font_size)
            title(strrep(objects{object_nr},'_', ' '))
            % title(['Results for ' distance ' distance'])
            if legend_on, legend(legends, 'location', 'northwest', 'fontsize', font_size_legend); end
            set(gca, 'fontsize', font_size);
            set(gca, 'UserData', distance{1}); % TODO: We store the distance title here because we loose the association by indexing when handling missing files
            zoomed_plot_ax = [zoomed_plot_ax gca];
        end
    end
end % End loop over distance metrics

% Present all match timings Latex-friendly
if verbose && meta_loaded && feature_numbers_loaded
    disp 'Match timings, Latex-friendly:'
    for i=1:numel(features)
        fprintf('%s & ', label_map(features{i}));
        for j=1:numel(distances)-1
            fprintf('%.3f & ', match_timings_mean_all(i,j));
        end
        fprintf('%.3f \\\\\n', match_timings_mean_all(i,end));
    end
end

%% Correct axes of zoomed plots
if zoomed_plots
    if recall_max > 0.5, tick = 0.1; else tick = 0.05; end
    recall_max = ceil(recall_max / tick) * tick;
%         set(zoomed_plot_ax, 'ylim', [0 recall_max], 'ytick', 0:tick:recall_max);
    set(zoomed_plot_ax, 'ylim', zoom_y_limits, 'ytick', zoom_y_ticks);

    % Save zoomed plot, if saving enabled
    if exist('figure_output_dir') && length(figure_output_dir) > 0
%         assert(numel(zoomed_plot_ax) == numel(distances));
        for i = 1:numel(zoomed_plot_ax)
            distance = get(zoomed_plot_ax(i), 'UserData');
            figure_output_file = [figure_output_dir '/' data_dir '/' objects{object_nr} '_' distance '_zoom' ]
            %figure_output_file = [figure_output_dir '/' strrep(data_dir, 'output/', 'output_') '_' distance '_zoom']
            %figure_output_file = [figure_output_dir '/' strrep(data_dir, 'output_') '_' distance '_zoom'];
            figure_output_file = strrep(figure_output_file, '.', ''); % Dots are not handled well by pdflatex
            % Special case for fusion: two possibly graphs
            if use_fusion
                if fusion_ternary
                    figure_output_file = strrep(figure_output_file, 'fusion', 'fusion_ternary');
                else
                    figure_output_file = strrep(figure_output_file, 'fusion', 'fusion_binary');
                end
            end
            fprintf('Saving zoomed plot to %s...\n', [figure_output_file '.pdf']);
            h = get(zoomed_plot_ax(i), 'parent');
            print(h, figure_output_file, '-dpdf');
        end
    end
end
