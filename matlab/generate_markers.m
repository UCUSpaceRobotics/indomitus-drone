% generate_markers.m
% Generates ArUco markers for Takeoff Pad (101) and Landing Target (102)
% Dictionary: DICT_ARUCO_ORIGINAL
% Target physical size: 15 cm x 15 cm (0.15 m)

fprintf('Generating ArUco markers (DICT_ARUCO_ORIGINAL)...\n');

% Marker 101 (Takeoff Pad)
marker101 = generateArucoMarker("DICT_ARUCO_ORIGINAL", 101, 600);
imwrite(marker101, 'aruco_101.png');
fprintf('Saved aruco_101.png (Marker 101 - Takeoff Pad)\n');

% Marker 102 (Landing Target)
marker102 = generateArucoMarker("DICT_ARUCO_ORIGINAL", 102, 600);
imwrite(marker102, 'aruco_102.png');
fprintf('Saved aruco_102.png (Marker 102 - Landing Target)\n');

% Display markers
figure('Name', 'ERC 2026 ArUco Markers');
subplot(1,2,1); imshow(marker101); title('Marker 101 (Takeoff Pad)');
subplot(1,2,2); imshow(marker102); title('Marker 102 (Landing Target)');

fprintf('\nIMPORTANT: Print these images so that the black marker square is EXACTLY 15 cm x 15 cm.\n');
