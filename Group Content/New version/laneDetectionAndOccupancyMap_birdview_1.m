function laneDetectionRealtime_dynamicThresholdWithOccupancyMap
% Lane line detection based on camera images, with dynamic threshold adjustment and occupancy map generation
% 处理流程:
% 1. 对单通道灰度图使用双边滤波，减小噪声的同时保留边缘；
% 2. 根据全图平均亮度动态调整阈值(线性插值):
%       如果 avgBrightness <= 13, 阈值 = 12;
%       如果 avgBrightness >= 150, 阈值 = 175;
%       否则: greythreshold = 1.0788 * avgBrightness + 4.0248;
% 3. 二值化结果进行形态学运算，以得到候选车道线区域；
% 4. 根据形态学结果构造“反转掩膜”，让车道部分显示为红色[255,0,0]，背景为黑色；
% 5. 使用相机的内参与外参(相机位置(0,0,0.8)，俯仰角 pitch=-30°)来构造单应矩阵(H)。由于imwarp做的是“图像到地面”的变换，需要对先前的“地面到图像”H做**矩阵求逆**；
% 6. 结果映射到设定的地面坐标范围(x ∈ [-1,3] 米, y ∈ [-1,1] 米, 分辨率0.05米/像素)以生成occupancy map(车道 = 1, 其他=0)；
% 7. 以2行×4列的subplot实时显示：原图、滤波结果、阈值化、中间形态学结果、叠加图、占据图，以及文字信息。

%% 1. 初始化ROS2节点和Subscriber
node = ros2node("test_node");
imageSub = ros2subscriber(node, "/multisense/left/image_rect", "sensor_msgs/Image", ...
    "Reliability", "besteffort", "Durability", "volatile", "History", "keeplast", "Depth", 10);

%% 2. 创建实时显示窗口(2行 × 4列)
hFig = figure('Name', 'Real-time Lane Detection, Dynamic Threshold & Occupancy Map', 'NumberTitle', 'off');

while isvalid(hFig)
    overallTimer = tic;
    
    %% 1. 图像接收
    try
        tPreReceive = tic;
        imgMsg = receive(imageSub, 5);  % 最多等待5秒
        rawImg = rosReadImage(imgMsg);  % 单通道灰度图，大小1024×544
        tReceive = toc(tPreReceive);
    catch ME
        warning('Failed to receive image: %s', ME.message);
        continue;
    end
    
    %% 2. 双边滤波：在保留边缘的同时减小噪声
    tPreBlur = tic;
    blurredImg = imbilatfilt(rawImg);
    tBlur = toc(tPreBlur);
    
    %% 3. 动态阈值 (线性插值)
    tPreThreshold = tic;
    avgBrightness = mean(rawImg(:));
    
    if avgBrightness <= 13
        greythreshold = 12;
    elseif avgBrightness >= 150
        greythreshold = 175;
    else
        % 简单的线性关系
        greythreshold = 1.0788 * avgBrightness + 4.0248;
    end
    
    extractGraph = rawImg < greythreshold; 
    tThreshold = toc(tPreThreshold);
    
    %% 4. 形态学操作：增强车道线区域连续性
    tPreMorph = tic;
    laneLineMask = processEdges(extractGraph);
    tMorph = toc(tPreMorph);
    
    %% 5. 构造“叠加图”，将车道线区域显示为红色
    tPreOverlay = tic;
    overlay = createOverlay(laneLineMask);
    tOverlay = toc(tPreOverlay);
    
    %% 6. 生成占据栅格图（透视变换到地面坐标）
    tPreOcc = tic;
    % 6.1 设定输出地面坐标范围与分辨率
    xWorldLimits = [-1, 5];   % x方向 [-1, 5] 米
    yWorldLimits = [-2, 2];   % y方向 [-2, 2] 米
    pixelExtent = 0.01;       % 分辨率 = 0.01米/像素
    outputCols = round(diff(xWorldLimits)/pixelExtent);  % 宽度方向像素数
    outputRows = round(diff(yWorldLimits)/pixelExtent);  % 高度方向像素数
    outputRef = imref2d([outputRows, outputCols], xWorldLimits, yWorldLimits);
    
    % 6.2 相机内参 (MultiSense S7 2MP缩放到1024×544时的近似值)
    K = [595,   0,   512;
          0,  590,   272;
          0,    0,     1];
    
    % 6.3 相机外参：相机位姿：C = (0,0,0.8), 俯仰(pitch) = -30°
    theta = -30 * pi/180;
    % 绕y轴的旋转矩阵(只考虑俯仰)
    R = [ cos(theta),  0, sin(theta);
          0,           1, 0;
         -sin(theta),  0, cos(theta) ];
    C = [0; 0; 0.8];
    
    % 平移向量 (因为旋转后，再将相机位置平移到原点，所以t_extr = -R*C)
    t_extr = -R * C;
    
    % 6.4 平面到图像的单应矩阵 H_plane2image
    % 在地面( Z=0 )下: [X; Y; 1] → s*[u; v; 1] = K * [R(:,1:2), t_extr] * [X; Y; 1]
    H_plane2image = K * [R(:,1:2), t_extr];
    
    % -------- 关键修改：需要从图像到地面的逆变换 --------
    % 原来的H_plane2image是地面→图像映射; imwarp需要的是图像→地面映射
    % 因此需要取 H_image2plane = inv(H_plane2image)
    H_image2plane = inv(H_plane2image);
    % 转换为projective2d时，要用转置：projective2d(H')。因此传入的矩阵也相应转置
    tform = projective2d(H_image2plane');
    
    % 6.5 提取叠加图中的红色像素(车道部分), 并投影到地面
    redChannel = overlay(:,:,1);
    occupancyInput = redChannel > 200;  % 车道区域
    
    % 通过imwarp映射到“地面坐标系”，且输出到自定义范围 outputRef
    [occupancyMap, ~] = imwarp(occupancyInput, tform, 'OutputView', outputRef);
    occupancyMap = occupancyMap > 0.5; 
    tOcc = toc(tPreOcc);
    
    overallTime = toc(overallTimer);
    
    %% 7. 分块显示结果(2行 × 4列)
    clf(hFig);
    
    subplot(2,4,1);
    imshow(rawImg, []);
    title(sprintf('原始图\n(接收: %.3f s)', tReceive));
    
    subplot(2,4,2);
    imshow(blurredImg, []);
    title(sprintf('双边滤波\n(%.3f s)', tBlur));
    
    subplot(2,4,3);
    imshow(extractGraph, []);
    title(sprintf('阈值( <%d )\n(%.3f s)', round(greythreshold), tThreshold));
    
    subplot(2,4,4);
    imshow(laneLineMask, []);
    title(sprintf('形态学处理\n(%.3f s)', tMorph));
    
    subplot(2,4,5);
    imshow(overlay);
    title(sprintf('叠加图\n(%.3f s)', tOverlay));
    
    subplot(2,4,6);
    imshow(occupancyMap, outputRef);
    title(sprintf('Occupancy Map\n(%.3f s)', tOcc));
    
    subplot(2,4,7);
    txt = sprintf(['总帧处理时间: %.3f s\n' ...
                   '接收: %.3f s\n双边滤波: %.3f s\n阈值: %.3f s\n形态学: %.3f s\n' ...
                   '平均亮度: %d\n动态阈值: %d'], ...
                   overallTime, tReceive, tBlur, tThreshold, tMorph, ...
                   round(avgBrightness), round(greythreshold));
    text(0.1, 0.5, txt, 'FontSize', 12);
    axis off;
    
    disp(['整帧耗时: ', num2str(overallTime), ' s  |  图像接收: ', num2str(tReceive), ' s']);
end
end

%% ----------------------- 辅助函数们 ------------------------------------
%% 形态学处理，增强车道线连续性
function enhancedEdges = processEdges(edgeImg)
    % 先腐蚀，再重建，再膨胀
    se = strel('rectangle', [1, 2]);
    marker = imerode(edgeImg, se);
    reconstructedEdges = imreconstruct(marker, edgeImg);
    enhancedEdges = imdilate(reconstructedEdges, strel('rectangle', [2, 6]));
end

%% 生成叠加图：将车道线区域显示为红色
function overlay = createOverlay(laneLineMask)
    % 注意：laneLineMask为true表示背景，false表示车道
    % 我们想让车道是红色 → 先取反 (~laneLineMask) 再给红通道赋值
    overlay = zeros([size(laneLineMask), 3], 'uint8');
    overlay(:,:,1) = 255 * uint8(~laneLineMask);  % 红通道
    % 其余通道为0
end
