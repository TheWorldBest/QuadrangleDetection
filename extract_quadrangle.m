function points4 = extract_quadrangle(img, lines)
% 根据检测的到的直线lines，找出img中的文档四边形
% output: 
%   points4: 4x2矩阵，分别为[ltx,lty; rtx,rty; lbx,lby; rbx,rby]

debug_my = 0; % 是否调试
l_lines = length(lines);
distance_max = 15; %判断交点是否在直线上的最大距离
angle_max = 40; % 四边形领边与垂线的最大夹角
angle_parallel = 15; % 平行边夹角最大值
distance_parallel = 40; % 平行边最小间距
infinity = 66666; % 直线平行时的交点默认坐标

[img_h,img_w,img_d] = size(img);
points4 = [1,1;img_w,1;img_w,img_h;1,img_h];

% 标志直线间的信息，如是否存在相交部分、相交点是否是其他直线的终止处、交点坐标(x,y)、与其他直线的角度等等
flags = struct('intersection',zeros(l_lines),... % 是否有交点在直线上，
               'endpoint',zeros(l_lines),... % 是否在端点上
               'points',infinity*ones(l_lines, l_lines, 2),...
               'angles',zeros(l_lines),...
               'nearpoint',zeros(l_lines)); % 注明交点靠近端点1还是2,0-接近端点1,1-接近端点2
% 统计各条直线间的关系     
for i = 1 : l_lines - 1
    if debug_my == 1
        figure(10);imshow(img);
        hold on;
        xy = [lines(i).point1;lines(i).point2];   
        plot(xy(:,1),xy(:,2),'LineWidth',4);
    end
    for j = i + 1 : l_lines
        if debug_my == 1
        	xy = [lines(j).point1;lines(j).point2];   
            plot(xy(:,1),xy(:,2),'LineWidth',4);
        end
        flags.angles(i,j) = lines(i).theta - lines(j).theta;
        flags.angles(j,i) = -flags.angles(i,j);
        if flags.angles(i,j) ~= 0
            [px, py] = point_calcualation(lines(i), lines(j)); % 求交点
            if debug_my == 1  
                plot(px,py,'r--*');
            end
            flags.points(i,j,:) = [px,py];
            flags.points(j,i,:) = [px,py];
            % 交点是否在直线i上
            [inter_flag, end_flag, near_point] = is_point_on_line([px,py], lines(i), distance_max);
            flags.intersection(i,j) = inter_flag;
            flags.endpoint(i,j) = end_flag;
            flags.nearpoint(i,j) = near_point;
            % 交点是否在直线j上
            [inter_flag, end_flag, near_point] = is_point_on_line([px,py], lines(j), distance_max);
            flags.intersection(j,i) = inter_flag;
            flags.endpoint(j,i) = end_flag;
            flags.nearpoint(j,i) = near_point;
        end
    end
    if debug_my == 1
        hold off;
    end
end

% 前n条最长直线构成的四边形
n_max = 1;
points_all = infinity*ones(n_max, 4, 2);
sides_all = zeros(n_max, 3); % 3维分别表示：第1端点处的侧边、第2端点处的侧边、平行边
for n = 1 : n_max
    % 寻找可能的平行边
    for i_side = 1 + n : l_lines
        if abs(flags.angles(n, i_side)) <= angle_parallel
            if abs(lines(n).rho - lines(i_side).rho) > distance_parallel
                sides_all(n, 3) = i_side;
                break;
            end
        end
    end
    
    % 寻找所有可能的侧边
    % 先寻找端点处的侧边
    for i_side = 1 + n : l_lines
        if abs(abs(flags.angles(n, i_side)) - 90) <  angle_max
            if flags.endpoint(n, i_side) == 1
                if sides_all(n, flags.nearpoint(n, i_side)+1) == 0
                    sides_all(n, flags.nearpoint(n, i_side)+1) = i_side;
                end
            end 
        end
    end
    % 若侧边没找全，则再寻找非相交的侧边
    if sides_all(n,1) == 0 || sides_all(n,2) == 0
        for i_side = 1 + n : l_lines
            if abs(abs(flags.angles(n, i_side)) - 90) <  angle_max
                if flags.intersection(n, i_side) == 0
                    if sides_all(n, flags.nearpoint(n, i_side)+1) == 0
                        sides_all(n, flags.nearpoint(n, i_side)+1) = i_side;
                    end
                end 
            end
        end
    end
    
    sides = [n,sides_all(n,1),sides_all(n,3),sides_all(n,2)]; % 
    points_all(n,:,:) = find_4point(img, lines, sides, flags);
%     side_num = 3 - sum(sides_all(n,:) == 0); % 找到的边数
%     
%     if side_num == 3
%         points_all(n, 1, :) = flags.points(n, sides_all(n, 1),:);
%         points_all(n, 2, :) = flags.points(n, sides_all(n, 2),:);
%         points_all(n, 3, :) = flags.points(sides_all(n, 2), sides_all(n, 3),:);
%         points_all(n, 4, :) = flags.points(sides_all(n, 1), sides_all(n, 3),:);
%         points4 = reshape(points_all(1,:,:),4,2);
%     elseif side_num == 2
%     elseif side_num == 1
%         
%     elseif side_num == 0
%         
%     end
    if debug_my == 1
        figure(11);imshow(img);
        hold on;
        x_all = [points_all(n, :, 1),points_all(n, 1, 1)];
        y_all = [points_all(n, :, 2),points_all(n, 1, 2)];
        plot(x_all,y_all,'LineWidth',4);
        hold off;
    end
end
points4 = reshape(points_all(1,:,:),4,2);
points4 = points_reshape(points4);
end

function all_points = find_4point(img, lines, sides, flags)
% 根据检测到的边缘情况找到目标四边形的4个顶点
% input
%   img：输入图像
%   lines：检测到的所有直线
%   sides：1x4矩阵，记录检测到的目标四边形的边的序号，确保sides(1)不为0；sides(3)为sides(1)的平行边；sides(i)=0-表示没有检测到
% output
% all_points：目标四边形的四个顶点

    [img_h,img_w,img_d] = size(img);
    all_points = [1,1;img_w,1;img_w,img_h;1,img_h]; %初始化
    
    side_num = 4 - sum(sides(:) == 0); % 找到的边数
    if side_num == 4
        all_points(1, :) = reshape(flags.points(sides(1), sides(2),:),1,2);
        all_points(2, :) = reshape(flags.points(sides(1), sides(4),:),1,2);
        all_points(3, :) = reshape(flags.points(sides(4), sides(3),:),1,2);
        all_points(4, :) = reshape(flags.points(sides(2), sides(3),:),1,2);
    elseif side_num == 3
        % 记录中间的边
        side_mid = 1;
        if sides(2) == 0
            side_mid = 4;
        elseif sides(4) == 0
            side_mid = 2;
        end
        line1_ord = sides(mod(side_mid, 4) + 1); % 存储较长边的序号
        line2_ord = sides(side_mid); % 记录中间边序号
        line3_ord = sides(mod(side_mid - 2, 4) + 1); % 记录较短边序号
        if lines(line1_ord).rho < lines(line3_ord).rho
            line_temp = line3_ord;
            line3_ord = line1_ord;
            line1_ord = line_temp;
        end
        
        % 计算两侧边与图像边界的两个交点
        points2_s1 = intersection_boundary(img, lines(line1_ord));
        points2_s3 = intersection_boundary(img, lines(line3_ord));
        % 计算两侧边与中间边的交点
        point1_2 = reshape(flags.points(line1_ord, line2_ord,:),1,2);
        point3_2 = reshape(flags.points(line3_ord, line2_ord,:),1,2);
        all_points(2, :) = point1_2;
        all_points(3, :) = point3_2;
        % 记录第1条边上远离交点p12的端点e1f
        if flags.nearpoint(line1_ord, line2_ord) == 0
            far_point_s1 = lines(line1_ord).point2;
        else
            far_point_s1 = lines(line1_ord).point1;
        end
        % 记录各向量
        vec1 = far_point_s1 - point1_2;
        vec1_1 = points2_s1(1,:) - point1_2;
        vec1_2 = points2_s1(2,:) - point1_2;
        % 确定边1和图像边界的交点，取内积大的点
        if vec1*vec1_1' > vec1*vec1_2'
            all_points(1, :) = points2_s1(1,:);
        else
            all_points(1, :) = points2_s1(2,:);
        end
        % 确定第4个交点，即边3和图像边界的交点；
        % 取与line1_ord夹角较小的点作为第4个顶点，即较大余弦值点
        vec1 = far_point_s1 - point1_2;
        vec1_1 = points2_s3(1,:) - point1_2;
        vec1_2 = points2_s3(2,:) - point1_2;
        angle_1 = dot(vec1, vec1_1)/(norm(vec1)*norm(vec1_1)); % 余弦
        angle_2 = dot(vec1, vec1_2)/(norm(vec1)*norm(vec1_2));
        if angle_1 >= angle_2
            all_points(4, :) = points2_s3(1,:);
        else
            all_points(4, :) = points2_s3(2,:);
        end
    elseif side_num == 2
    elseif side_num == 1 % 不处理
    end
end

function points2 = intersection_boundary(img, line1)
% 计算直线line1与图像边缘的两个交点
% output
%   points2：2x2矩阵，[x1,y1;x2,y2]

    [img_h,img_w,img_d] = size(img);
    % 直线1方程a1*x+b1*y+c1=0
    a1 = cos(deg2rad(line1.theta));
    b1 = sin(deg2rad(line1.theta));
    c1 = -line1.rho;
    
    if line1.theta == -90 % 水平，即与x轴平行
        points2 = [0, line1.point1(2); img_w, line1.point1(2)];
    elseif line1.theta == 0 % 垂直，即与y轴平行
        points2 = [line1.point1(1), 0; line1.point1(1), img_h];
    else
        num_p = 0; % 记录找到的有效点数
        temp = [0,(-c1)/b1; img_w,(-c1 - img_w*a1)/b1; (-c1)/a1, 0; (-c1 - img_h*b1)/a1, img_h];
        for i = 1 : 4
            if temp(i,1) >= 0 && temp(i,1) <= img_w && temp(i,2) >= 0 && temp(i,2) <= img_h
               num_p = num_p + 1;
               points2(num_p, : ) = temp(i,:);
               if num_p >= 2 
                   break;
               end
            end
        end
    end
end

function rect_out = points_reshape(rect_in)
% 将输入四边形的四个顶点重新整理为左上、右上、右下、左下顺序
% input：
%   rect_in：输入的四边形
% output：
%   rect_out：顶点顺序重整后的四边形

    rect_out = zeros(4,2);
    dists = zeros(4,1);
    for i = 1 : 4
        dists(i) = rect_in(i,1)^2+rect_in(i,2)^2;
    end
    [~,Ind] = min(dists); % 以短距离视为左上角点
    rect_out(1,:) = rect_in(Ind,:);
    point_rt = 0;
    % 右上角点
    if rect_in(mod(Ind,4)+1,1) >= rect_in(mod(Ind-2,4)+1,1)
        point_rt = 1;
        if rect_in(mod(Ind,4)+1,2) > rect_in(mod(Ind-2,4)+1,2)
            point_rt = -1;
        end
    end

    if point_rt > 0
        for j = 2 : 4
            rect_out(j,:) = rect_in(mod(Ind+j-2,4)+1,:);
        end
    else
        for j = 2 : 4
            rect_out(j,:) = rect_in(mod(Ind-j,4)+1,:);
        end
    end
end

function [px, py] = point_calcualation(line1, line2)
% 计算两直线的交点坐标，注意，确保输入的两条直线不平行
% input:
%   line1,line2：两条直线
% output：
%   px,py：交点坐标

    % 直线1方程a1*x+b1*y+c1=0
    a1 = cos(deg2rad(line1.theta));
    b1 = sin(deg2rad(line1.theta));
    c1 = -line1.rho;
    % 直线2方程a2*x+b2*y+c2=0
    a2 = cos(deg2rad(line2.theta));
    b2 = sin(deg2rad(line2.theta));
    c2 = -line2.rho;
    
    px = round((b1*c2-b2*c1)/(a1*b2-a2*b1));
    py = round((a2*c1-a1*c2)/(a1*b2-a2*b1));
end
function [inter_flag, end_flag, near_point] = is_point_on_line(point1, line1, distance_max)
% 判断点point1是否在直线line1上，是否为直线line1的端点
% input:
%   point1: 交点坐标
%   line1: 直线
%   distance_max： 判断点在直线上的允许距离差
% output：
%   inter_flag：记录交点是否在该直线上
%   end_flag：记录交点是否为在该直线的端点处
%   near_point：交点离哪个端点近，

    inter_flag = 0;
    end_flag = 0;
    px = point1(1);
    py = point1(2);
    x1 = line1.point1(1);
    y1 = line1.point1(2);
    x2 = line1.point2(1);
    y2 = line1.point2(2);
    if x1 == x2
        if py >= min(y1,y2) - distance_max && py <= max(y1,y2) + distance_max
            inter_flag = 1;
            if abs(py - y1) <= distance_max || abs(py - y2) <= distance_max
                end_flag = 1;
            end
        end
    else
        distance_project = ceil(abs(distance_max*sin(deg2rad(line1.theta)))); %distance_max投影到x轴上的长度
        if px >= min(x1,x2) - distance_project && px <= max(x1,x2) + distance_project
            inter_flag = 1;
            if abs(px - x1) <= distance_project || abs(px - x2) <= distance_project
                end_flag = 1;
            end
        end
    end
    if abs(px - x1) + abs(py - y1) > abs(px - x2) + abs(py - y2)
        near_point = 1;
    else
        near_point = 0;
    end
end