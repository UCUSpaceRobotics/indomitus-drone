% calibrate_camera.m
% Scripted Camera Calibration using Checkerboard pattern
% Place captured calibration JPG images from the Raspberry Pi in './calibration_images/'

imageDir = './calibration_images/';
if ~exist(imageDir, 'dir')
    error('Directory %s does not exist. Create it and place calibration images inside.', imageDir);
end

images = imageDatastore(imageDir);
if numel(images.Files) == 0
    error('No images found in %s. Place 15-20 checkerboard photos there.', imageDir);
end

fprintf('Processing %d calibration images...\n', numel(images.Files));
[imagePoints, boardSize] = detectCheckerboardPoints(images.Files);

% Define world coordinates of the checkerboard corners (Square size in mm)
squareSize = 25;  % 25 mm square size (change if using a different size)
worldPoints = patternWorldPoints("checkerboard", boardSize, squareSize);

% Get image size from the first image
I = readimage(images, 1);
imageSize = [size(I, 1), size(I, 2)];

% Perform calibration
cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
    'ImageSize', imageSize, ...
    'EstimateSkew', false, ...
    'EstimateTangentialDistortion', true, ...
    'NumRadialDistortionCoefficients', 3);

% Display calibration metrics
fprintf('\n=== Calibration Results ===\n');
fprintf('Focal Length: [%.2f, %.2f] pixels\n', cameraParams.Intrinsics.FocalLength);
fprintf('Principal Point: [%.2f, %.2f] pixels\n', cameraParams.Intrinsics.PrincipalPoint);
fprintf('Image Size: [%d, %d]\n', cameraParams.Intrinsics.ImageSize);
fprintf('Radial Distortion: [%.6f, %.6f, %.6f]\n', cameraParams.Intrinsics.RadialDistortion);

% Save camera parameters for desktop testing and Simulink configuration
save('cameraParams.mat', 'cameraParams');
fprintf('\nSaved calibration parameters to cameraParams.mat\n');

% Show reprojection error plot
figure('Name', 'Reprojection Errors');
showReprojectionErrors(cameraParams);
