function write_pcd(filename, X)

dims = size(X);
assert(dims(1) > 0, 'No points in data set!');
assert(dims(2) == 3 || dims(2) == 6,...
    'Only XYZ or XYZ+normal point clouds supported!');

fid = fopen(filename, 'w');
assert(fid ~= -1, 'Failed to open file!');

% Write header
fprintf(fid, '# .PCD v0.7 - Point Cloud Data file format\n');
fprintf(fid, 'VERSION 0.7\n');
if(dims(2) == 3)
    fprintf(fid, 'FIELDS x y z\n');
    fprintf(fid, 'SIZE 4 4 4\n');
    fprintf(fid, 'TYPE F F F\n');
    fprintf(fid, 'COUNT 1 1 1\n');
    
else
    fprintf(fid, 'FIELDS x y z normal_x normal_y normal_z\n');
    fprintf(fid, 'SIZE 4 4 4 4 4 4\n');
    fprintf(fid, 'TYPE F F F F F F\n');
    fprintf(fid, 'COUNT 1 1 1 1 1 1\n');
end

fprintf(fid, 'WIDTH %i\n', dims(1));
fprintf(fid, 'HEIGHT 1\n');
fprintf(fid, 'VIEWPOINT 0 0 0 1 0 0 0\n');
fprintf(fid, 'POINTS %i\n', dims(1));
fprintf(fid, 'DATA binary\n');

% Write point data
fwrite(fid, X', 'float');

% Close file
assert(fclose(fid) == 0, 'Failed to close file!');
