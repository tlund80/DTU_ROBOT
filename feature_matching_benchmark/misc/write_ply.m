function write_ply(filename, X, poly)

dims = size(X);
assert(dims(1) > 0, 'No points in data set!');
assert(dims(2) == 3 || dims(2) == 6,...
    'Only XYZ or XYZ+normal point clouds supported!');

has_normal = (dims(2) == 6);

has_poly = (nargin > 2);
if has_poly, num_polys = size(poly); end

fid = fopen(filename, 'w');
assert(fid ~= -1, 'Failed to open file!');

fprintf(fid, 'ply\n');
fprintf(fid, 'format ascii 1.0\n');
fprintf(fid, 'element vertex %i\n', dims(1));
fprintf(fid, 'property float x\n');
fprintf(fid, 'property float y\n');
fprintf(fid, 'property float z\n');
if has_normal
    fprintf(fid, 'property float nx\n');
    fprintf(fid, 'property float ny\n');
    fprintf(fid, 'property float nz\n');
end
if has_poly
    fprintf(fid, 'element face %d\n', num_polys(1));
    fprintf(fid, 'property list uchar int vertex_indices\n');
end
fprintf(fid, 'end_header\n');

for i = 1:dims(1)
    fprintf(fid, '%f %f %f', X(i,1), X(i,2), X(i,3));
    if has_normal, fprintf(fid, ' %f %f %f', X(i,4), X(i,5), X(i,6)); end
    fprintf(fid, '\n');
end

if has_poly
    for i = 1:num_polys(1)
        fprintf(fid, '%d ', num_polys(2));
        for j = 1:num_polys(2)
            fprintf(fid, '%d ', poly(i,j)-1);
        end
        fprintf(fid, '\n');
    end
end

assert(fclose(fid) == 0, 'Failed to close file!');

end