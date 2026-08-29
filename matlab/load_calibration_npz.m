function calib = load_calibration_npz(relPath, targetImageSize)
% relPath: relative path from current model folder, e.g. '../config/camera_calibration.npz'
% targetImageSize: [rows, cols], e.g. [480, 640]

if nargin < 2
    targetImageSize = [480, 640];
end

% Resolve robust absolute path relative to this script / model
baseDir = fileparts(mfilename('fullpath'));
fullPath = fullfile(baseDir, relPath);

if ~isfile(fullPath)
    error('Calibration file not found at: %s', fullPath);
end

% Load .npz using MATLAB Python interface
np = py.importlib.import_module('numpy');
npzData = np.load(fullPath, pyargs('allow_pickle', true));

K = double(npzData.get('camera_matrix'));
dist = double(npzData.get('dist_coeffs'));
calibSize = double(npzData.get('image_size')); % [width, height], e.g. [1280, 720]

calibW = calibSize(1);
calibH = calibSize(2);
targetH = targetImageSize(1);
targetW = targetImageSize(2);

% Scale fx, fy, cx, cy to match runtime capture resolution
scaleX = targetW / calibW;
scaleY = targetH / calibH;

fx = K(1,1) * scaleX;
fy = K(2,2) * scaleY;
cx = K(1,3) * scaleX;
cy = K(2,3) * scaleY;

% OpenCV dist_coeffs format: [k1, k2, p1, p2, k3]
% MATLAB RadialDistortion: [k1, k2, k3]
% MATLAB TangentialDistortion: [p1, p2]
k1 = dist(1);
k2 = dist(2);
p1 = dist(3);
p2 = dist(4);
if numel(dist) >= 5
    k3 = dist(5);
else
    k3 = 0.0;
end

calib.focalLength          = [fx, fy];
calib.principalPoint       = [cx, cy];
calib.radialDistortion     = [k1, k2, k3];
calib.tangentialDistortion = [p1, p2];
calib.imageSize            = targetImageSize;

fprintf('Loaded calibration from: %s\n', fullPath);
fprintf('  Scaled Focal Length: [%.2f, %.2f]\n', fx, fy);
fprintf('  Scaled Principal Point: [%.2f, %.2f]\n', cx, cy);
end