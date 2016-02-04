function X = read_pts_ops(filename)

fid = fopen(filename, 'r');
assert(fid ~= -1, 'Failed to open file!');

% Get number of points
N = str2num(fgetl(fid));
assert(N > 0, 'No points in data file!');

% Get dimension from first line
tline = fgetl(fid);
dim = numel(str2num(tline));
assert(dim == 3 || dim == 6,...
    'Only XYZ or XYZ+normal point clouds supported!');

% Allocate output and write first line
X = zeros(N, dim);
cnt = 1;
while ischar(tline)
    X(cnt, :) = str2num(tline);
    tline = fgetl(fid);
    cnt = cnt + 1;
end

% Close file
assert(fclose(fid) == 0, 'Failed to close file!');

end