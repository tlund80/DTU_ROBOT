function pc = depth2pc(depth)

pc = depthToCloud(depth);
pc = squeeze(reshape(pc, [], 1, 3));
pc = pc(~isnan(pc(:, 1)), :);

end