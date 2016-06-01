function [label_map color_map marker_map dimension_map] = setup(use_pca)

dimension_map = containers.Map(...
    {...
        'ecsad'
        'fpfh'
        'ndhist'
        'rops'
        'shot'
        'si'
        'usc'
        'pfh'
        '3dsc'
    },...
    {...
        '30'
        '33'
        '128'
        '135'
        '352'
        '153'
        '1960'
        '125'
        '1980'
    }...
);

if use_pca
    label_map = containers.Map(...
        {...
            'ecsad'
            'fpfh'
            'ndhist'
            'rops'
            'shot'
            'si'
            'usc'
            'pfh'
            '3dsc'
            % PCA
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
            % Fusion
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
        },...
        {...
            'ECSAD(30)'
            'FPFH(33)'
            'NDHist(128)'
            'RoPS(135)'
            'SHOT(352)'
            'SI(153)'
            'USC(1960)'
            'PFH(125)'
            '3DSC(1980)'
            % PCA
            'ECSAD+PCA(22)'
            'ECSAD+PCA(14)'
            'ECSAD+PCA(9)'
            'RoPS+PCA(68)'
            'RoPS+PCA(26)'
            'RoPS+PCA(21)'
            'SHOT+PCA(227)'
            'SHOT+PCA(130)'
            'SHOT+PCA(86)'
            'USC+PCA(1193)'
            'USC+PCA(618)'
            'USC+PCA(390)'
            % Fusion
            'ECSAD+NDHist'
            'ECSAD+RoPS'
            'ECSAD+SHOT'
            'ECSAD+SI'
            'NDHist+RoPS'
            'NDHist+SHOT'
            'NDHist+SI'
            'RoPS+SHOT'
            'RoPS+SI'
            'SHOT+SI'
            'ECSAD+NDHist+RoPS'
            'ECSAD+NDHist+SHOT'
            'ECSAD+NDHist+SI'
            'ECSAD+RoPS+SHOT'
            'ECSAD+RoPS+SI'
            'ECSAD+SHOT+SI'
            'NDHist+RoPS+SHOT'
            'NDHist+RoPS+SI'
            'NDHist+SHOT+SI'
            'RoPS+SHOT+SI'
        }...
    );
else
    label_map = containers.Map(...
        {...
            'ecsad'
            'fpfh'
            'ndhist'
            'rops'
            'shot'
            'si'
            'usc'
            'pfh'
            '3dsc'
            % PCA
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
            % Fusion
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
        },...
        {...
            'ECSAD'
            'FPFH'
            'NDHist'
            'RoPS'
            'SHOT'
            'SI'
            'USC'
            'PFH'
            '3DSC'
            % PCA versions
            'ECSAD+PCA(22)'
            'ECSAD+PCA(14)'
            'ECSAD+PCA(9)'
            'RoPS+PCA(68)'
            'RoPS+PCA(26)'
            'RoPS+PCA(21)'
            'SHOT+PCA(227)'
            'SHOT+PCA(130)'
            'SHOT+PCA(86)'
            'USC+PCA(1193)'
            'USC+PCA(618)'
            'USC+PCA(390)'
            % Fusion
            'ECSAD+NDHist'
            'ECSAD+RoPS'
            'ECSAD+SHOT'
            'ECSAD+SI'
            'NDHist+RoPS'
            'NDHist+SHOT'
            'NDHist+SI'
            'RoPS+SHOT'
            'RoPS+SI'
            'SHOT+SI'
            'ECSAD+NDHist+RoPS'
            'ECSAD+NDHist+SHOT'
            'ECSAD+NDHist+SI'
            'ECSAD+RoPS+SHOT'
            'ECSAD+RoPS+SI'
            'ECSAD+SHOT+SI'
            'NDHist+RoPS+SHOT'
            'NDHist+RoPS+SI'
            'NDHist+SHOT+SI'
            'RoPS+SHOT+SI'
        }...
    );
end

color_map = containers.Map(...
    {...
        'ecsad'
        'fpfh'
        'ndhist'
        'rops'
        'shot'
        'si'
        'usc'
        'pfh'
        '3dsc'
        % PCA
        'ecsad_pca99'
        'ecsad_pca95'
        'ecsad_pca90'
        'ndhist_pca99'
        'ndhist_pca95'
        'ndhist_pca90'
        'rops_pca99'
        'rops_pca95'
        'rops_pca90'
        'shot_pca99'
        'shot_pca95'
        'shot_pca90'
        'usc_pca99'
        'usc_pca95'
        'usc_pca90'
        % Fusion
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
    },...
    {...
        [0.9 0 0] % ecsad
        [0 0 0.9] % fpfh
        [0 0.9 0] % ndhist
        [0.25 0.25 0.25] % rops
        [120 20 225] ./ 255 % shot
        [150 50 0] ./ 255 % si
        [0.75 0.75 0] % usc
        [0 0.6 0.6] % pfh
        [0.6 0 0.6] % 3dsc
        % PCA
        [0.9 0 0]% ecsad
        [0.9 0 0]
        [0.9 0 0]
        [0 0.9 0] % ndhist
        [0 0.9 0]
        [0 0.9 0]
        [0.25 0.25 0.25] % rops
        [0.25 0.25 0.25]
        [0.25 0.25 0.25]
        [110 15 215] ./ 255 % shot
        [80 10 185] ./ 255
        [50 5 155] ./ 255
        [0.75 0.75 0] % usc
        [0.75 0.75 0]
        [0.75 0.75 0]
        % Fusion
        [0.9 0 0] % ecsad-ndhist
        [0.7 0 0] % ecsad-rops
        [0.5 0 0] % ecsad-shot
        [0.3 0 0] % ecsad-si
        [0 0.9 0] % ndhist-rops
        [0 0.7 0] % ndhist-shot
        [0 0.5 0] % ndhist-si
        [0.25 0.25 0.25] % rops-shot
        [0.15 0.15 0.15] % rops-si
        [110 15 215] ./ 255 % shot-si
        [0.9 0 0] % 'ecsad-ndhist-rops'
        [0.7 0 0] % 'ecsad-ndhist-shot'
        [0.5 0 0] % 'ecsad-ndhist-si'
        [0.3 0 0] % 'ecsad-rops-shot'
        [0.1 0 0] % 'ecsad-rops-si'
        [0 0 0] % 'ecsad-shot-si'
        [0 0.9 0] % 'ndhist-rops-shot'
        [0 0.7 0] % 'ndhist-rops-si'
        [0 0.5 0] % 'ndhist-shot-si'
        [0.25 0.25 0.25] % 'rops-shot-si'
    }...
);

marker_map = containers.Map(...
    {...
        'ecsad'
        'fpfh'
        'ndhist'
        'rops'
        'shot'
        'si'
        'usc'
        'pfh'
        '3dsc'
        % PCA
        'ecsad_pca99'
        'ecsad_pca95'
        'ecsad_pca90'
        'ndhist_pca99'
        'ndhist_pca95'
        'ndhist_pca90'
        'rops_pca99'
        'rops_pca95'
        'rops_pca90'
        'shot_pca99'
        'shot_pca95'
        'shot_pca90'
        'usc_pca99'
        'usc_pca95'
        'usc_pca90'
        % Fusion
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
    },...
    {...
        '-o' % ecsad
        '-+' % fpfh
        '-<' % ndhist
        '-s' % rops
        '-d' % shot
        '-v' % si
        '->' % usc
        '-^' %pfh
        '-^' %3dsc
        % PCA
        '--o' % ecsad
        '-.o'
        ':o'
        '--+' % ndhist
        '-.+'
        '_+'
        '--s' % rops
        '-.s'
        ':s'
        '--d' % shot
        '-.d'
        ':d'
        '-->' % usc
        '-.>'
        ':>'
        % Fusion
        '-o' % ecsad-ndhist
        '--o' % ecsad-rops
        '-.o' % ecsad-shot
        ':o' % ecsad-si
        '-<' % ndhist-rops
        '--<' % ndhist-shot
        ':<' % ndhist-si
        '-s' % rops-shot
        '--s' % rops-si
        '-d' % shot-si
        '-o' % 'ecsad-ndhist-rops'
        '--o' % 'ecsad-ndhist-shot'
        '-.o' % 'ecsad-ndhist-si'
        ':o' % 'ecsad-rops-shot'
        '-' % 'ecsad-rops-si'
        '--' % 'ecsad-shot-si'
        '-<'% 'ndhist-rops-shot'
        '--<'% 'ndhist-rops-si'
        ':<'% 'ndhist-shot-si'
        '-s' % 'rops-shot-si'
    }...
);