% Demo for Structured Edge Detector (please see readme.txt first).
close all; 

% img_path = 'D:\qxw\database\segmentation\normal_illumintation';
img_path = 'D:\qxw\database\segmentation\real_scenario';

save_path = './results/SFED';
img_names = dir([img_path, '\*.jpg']);
max_length = 300;  % 最大边长

for ord_img = 1 : length(img_names)
    %% 读取图像并归一化尺度
    img_name = fullfile(img_names(ord_img).folder, img_names(ord_img).name);
    I = imread(img_name);
    aspect_hw = size(I, 1)/size(I, 2);
    if aspect_hw > 1
        I = imresize(I, [max_length, round(max_length/aspect_hw)]);
    else
        I = imresize(I, [round(max_length*aspect_hw), max_length]);
    end
    figure(1);
    subplot(2,3,1);imshow(I); title('original');
    
    %% 边缘检测
    threshold = [0.01 0.2];
    sigma = 3;
    ER = edge(I(:,:,1),'Canny',threshold, sigma); 
    EG = edge(I(:,:,2),'Canny',threshold, sigma); 
    EB = edge(I(:,:,3),'Canny',threshold, sigma); 
    RGB = ER | EB | EG;
    subplot(2,3,2);imshow(RGB); title('canny');
    
    %% 直线检测
    img_hough = RGB;
    subplot(2,3,3);imshow(img_hough); title('hough');
    hold on
    %得到霍夫空间
    [H1,T1,R1] = hough(img_hough,'Theta',-90:1:89);
    %求极值点
    Peaks = houghpeaks(H1,15,'Threshold',15);
    %得到线段信息
    lines = houghlines(img_hough,T1,R1,Peaks);
    length_line = zeros(length(Peaks),1);
    linesNew(1) = lines(1);
    length_line(1) = sqrt((lines(1).point1(1)-lines(1).point2(1))^2 + (lines(1).point1(2)-lines(1).point2(2))^2);
    %绘制线段
    kN = 1;
    for k = 1:length(lines)
        xy = [lines(k).point1;lines(k).point2];   
        plot(xy(:,1),xy(:,2),'LineWidth',4);
        if k > 1
            if linesNew(kN).theta == lines(k).theta && linesNew(kN).rho == lines(k).rho % 合并同一条直线
                pos = [linesNew(kN).point1;linesNew(kN).point2;lines(k).point1;lines(k).point2];
                if linesNew(kN).theta == 0
                    [xmin,Ind] = min(pos);
                    linesNew(kN).point1 = pos(Ind(2),:);
                    [xmax,Ind] = max(pos);
                    linesNew(kN).point2 = pos(Ind(2),:);
                else
                    [xmin,Ind] = min(pos);
                    linesNew(kN).point1 = pos(Ind(1),:);
                    [xmax,Ind] = max(pos);
                    linesNew(kN).point2 = pos(Ind(1),:);
                end
            else
                kN = kN + 1;
                linesNew(kN) = lines(k);
            end
            length_line(kN) = sqrt((lines(k).point1(1)-lines(k).point2(1))^2 + (lines(k).point1(2)-lines(k).point2(2))^2) + length_line(kN);
        end
    end
    hold off
    
    
    %% 提取四边形
    points4 = extract_quadrangle(img_hough, linesNew);
    figure(1);subplot(2,3,5);imshow(img_hough);title('quadrangle');
    hold on;
    x_all = [points4( :, 1)',points4(1, 1)];
    y_all = [points4(:, 2)',points4(1, 2)];
    plot(x_all,y_all,'LineWidth',4);
    hold off;
    
    %% 透视变换
     img_out = perspective_transformation(I, points4);
     subplot(2,3,6);imshow(img_out);title('result');
    
  end
    