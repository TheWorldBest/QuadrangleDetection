function points4 = extract_quadrangle(img, lines)
% ���ݼ��ĵ���ֱ��lines���ҳ�img�е��ĵ��ı���
% output: 
%   points4: 4x2���󣬷ֱ�Ϊ[ltx,lty; rtx,rty; lbx,lby; rbx,rby]

debug_my = 0; % �Ƿ����
l_lines = length(lines);
distance_max = 15; %�жϽ����Ƿ���ֱ���ϵ�������
angle_max = 40; % �ı�������봹�ߵ����н�
angle_parallel = 15; % ƽ�б߼н����ֵ
distance_parallel = 40; % ƽ�б���С���
infinity = 66666; % ֱ��ƽ��ʱ�Ľ���Ĭ������

[img_h,img_w,img_d] = size(img);
points4 = [1,1;img_w,1;img_w,img_h;1,img_h];

% ��־ֱ�߼����Ϣ�����Ƿ�����ཻ���֡��ཻ���Ƿ�������ֱ�ߵ���ֹ������������(x,y)��������ֱ�ߵĽǶȵȵ�
flags = struct('intersection',zeros(l_lines),... % �Ƿ��н�����ֱ���ϣ�
               'endpoint',zeros(l_lines),... % �Ƿ��ڶ˵���
               'points',infinity*ones(l_lines, l_lines, 2),...
               'angles',zeros(l_lines),...
               'nearpoint',zeros(l_lines)); % ע�����㿿���˵�1����2,0-�ӽ��˵�1,1-�ӽ��˵�2
% ͳ�Ƹ���ֱ�߼�Ĺ�ϵ     
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
            [px, py] = point_calcualation(lines(i), lines(j)); % �󽻵�
            if debug_my == 1  
                plot(px,py,'r--*');
            end
            flags.points(i,j,:) = [px,py];
            flags.points(j,i,:) = [px,py];
            % �����Ƿ���ֱ��i��
            [inter_flag, end_flag, near_point] = is_point_on_line([px,py], lines(i), distance_max);
            flags.intersection(i,j) = inter_flag;
            flags.endpoint(i,j) = end_flag;
            flags.nearpoint(i,j) = near_point;
            % �����Ƿ���ֱ��j��
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

% ǰn���ֱ�߹��ɵ��ı���
n_max = 1;
points_all = infinity*ones(n_max, 4, 2);
sides_all = zeros(n_max, 3); % 3ά�ֱ��ʾ����1�˵㴦�Ĳ�ߡ���2�˵㴦�Ĳ�ߡ�ƽ�б�
for n = 1 : n_max
    % Ѱ�ҿ��ܵ�ƽ�б�
    for i_side = 1 + n : l_lines
        if abs(flags.angles(n, i_side)) <= angle_parallel
            if abs(lines(n).rho - lines(i_side).rho) > distance_parallel
                sides_all(n, 3) = i_side;
                break;
            end
        end
    end
    
    % Ѱ�����п��ܵĲ��
    % ��Ѱ�Ҷ˵㴦�Ĳ��
    for i_side = 1 + n : l_lines
        if abs(abs(flags.angles(n, i_side)) - 90) <  angle_max
            if flags.endpoint(n, i_side) == 1
                if sides_all(n, flags.nearpoint(n, i_side)+1) == 0
                    sides_all(n, flags.nearpoint(n, i_side)+1) = i_side;
                end
            end 
        end
    end
    % �����û��ȫ������Ѱ�ҷ��ཻ�Ĳ��
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
%     side_num = 3 - sum(sides_all(n,:) == 0); % �ҵ��ı���
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
% ���ݼ�⵽�ı�Ե����ҵ�Ŀ���ı��ε�4������
% input
%   img������ͼ��
%   lines����⵽������ֱ��
%   sides��1x4���󣬼�¼��⵽��Ŀ���ı��εıߵ���ţ�ȷ��sides(1)��Ϊ0��sides(3)Ϊsides(1)��ƽ�бߣ�sides(i)=0-��ʾû�м�⵽
% output
% all_points��Ŀ���ı��ε��ĸ�����

    [img_h,img_w,img_d] = size(img);
    all_points = [1,1;img_w,1;img_w,img_h;1,img_h]; %��ʼ��
    
    side_num = 4 - sum(sides(:) == 0); % �ҵ��ı���
    if side_num == 4
        all_points(1, :) = reshape(flags.points(sides(1), sides(2),:),1,2);
        all_points(2, :) = reshape(flags.points(sides(1), sides(4),:),1,2);
        all_points(3, :) = reshape(flags.points(sides(4), sides(3),:),1,2);
        all_points(4, :) = reshape(flags.points(sides(2), sides(3),:),1,2);
    elseif side_num == 3
        % ��¼�м�ı�
        side_mid = 1;
        if sides(2) == 0
            side_mid = 4;
        elseif sides(4) == 0
            side_mid = 2;
        end
        line1_ord = sides(mod(side_mid, 4) + 1); % �洢�ϳ��ߵ����
        line2_ord = sides(side_mid); % ��¼�м�����
        line3_ord = sides(mod(side_mid - 2, 4) + 1); % ��¼�϶̱����
        if lines(line1_ord).rho < lines(line3_ord).rho
            line_temp = line3_ord;
            line3_ord = line1_ord;
            line1_ord = line_temp;
        end
        
        % �����������ͼ��߽����������
        points2_s1 = intersection_boundary(img, lines(line1_ord));
        points2_s3 = intersection_boundary(img, lines(line3_ord));
        % ������������м�ߵĽ���
        point1_2 = reshape(flags.points(line1_ord, line2_ord,:),1,2);
        point3_2 = reshape(flags.points(line3_ord, line2_ord,:),1,2);
        all_points(2, :) = point1_2;
        all_points(3, :) = point3_2;
        % ��¼��1������Զ�뽻��p12�Ķ˵�e1f
        if flags.nearpoint(line1_ord, line2_ord) == 0
            far_point_s1 = lines(line1_ord).point2;
        else
            far_point_s1 = lines(line1_ord).point1;
        end
        % ��¼������
        vec1 = far_point_s1 - point1_2;
        vec1_1 = points2_s1(1,:) - point1_2;
        vec1_2 = points2_s1(2,:) - point1_2;
        % ȷ����1��ͼ��߽�Ľ��㣬ȡ�ڻ���ĵ�
        if vec1*vec1_1' > vec1*vec1_2'
            all_points(1, :) = points2_s1(1,:);
        else
            all_points(1, :) = points2_s1(2,:);
        end
        % ȷ����4�����㣬����3��ͼ��߽�Ľ��㣻
        % ȡ��line1_ord�нǽ�С�ĵ���Ϊ��4�����㣬���ϴ�����ֵ��
        vec1 = far_point_s1 - point1_2;
        vec1_1 = points2_s3(1,:) - point1_2;
        vec1_2 = points2_s3(2,:) - point1_2;
        angle_1 = dot(vec1, vec1_1)/(norm(vec1)*norm(vec1_1)); % ����
        angle_2 = dot(vec1, vec1_2)/(norm(vec1)*norm(vec1_2));
        if angle_1 >= angle_2
            all_points(4, :) = points2_s3(1,:);
        else
            all_points(4, :) = points2_s3(2,:);
        end
    elseif side_num == 2
    elseif side_num == 1 % ������
    end
end

function points2 = intersection_boundary(img, line1)
% ����ֱ��line1��ͼ���Ե����������
% output
%   points2��2x2����[x1,y1;x2,y2]

    [img_h,img_w,img_d] = size(img);
    % ֱ��1����a1*x+b1*y+c1=0
    a1 = cos(deg2rad(line1.theta));
    b1 = sin(deg2rad(line1.theta));
    c1 = -line1.rho;
    
    if line1.theta == -90 % ˮƽ������x��ƽ��
        points2 = [0, line1.point1(2); img_w, line1.point1(2)];
    elseif line1.theta == 0 % ��ֱ������y��ƽ��
        points2 = [line1.point1(1), 0; line1.point1(1), img_h];
    else
        num_p = 0; % ��¼�ҵ�����Ч����
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
% �������ı��ε��ĸ�������������Ϊ���ϡ����ϡ����¡�����˳��
% input��
%   rect_in��������ı���
% output��
%   rect_out������˳����������ı���

    rect_out = zeros(4,2);
    dists = zeros(4,1);
    for i = 1 : 4
        dists(i) = rect_in(i,1)^2+rect_in(i,2)^2;
    end
    [~,Ind] = min(dists); % �Զ̾�����Ϊ���Ͻǵ�
    rect_out(1,:) = rect_in(Ind,:);
    point_rt = 0;
    % ���Ͻǵ�
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
% ������ֱ�ߵĽ������꣬ע�⣬ȷ�����������ֱ�߲�ƽ��
% input:
%   line1,line2������ֱ��
% output��
%   px,py����������

    % ֱ��1����a1*x+b1*y+c1=0
    a1 = cos(deg2rad(line1.theta));
    b1 = sin(deg2rad(line1.theta));
    c1 = -line1.rho;
    % ֱ��2����a2*x+b2*y+c2=0
    a2 = cos(deg2rad(line2.theta));
    b2 = sin(deg2rad(line2.theta));
    c2 = -line2.rho;
    
    px = round((b1*c2-b2*c1)/(a1*b2-a2*b1));
    py = round((a2*c1-a1*c2)/(a1*b2-a2*b1));
end
function [inter_flag, end_flag, near_point] = is_point_on_line(point1, line1, distance_max)
% �жϵ�point1�Ƿ���ֱ��line1�ϣ��Ƿ�Ϊֱ��line1�Ķ˵�
% input:
%   point1: ��������
%   line1: ֱ��
%   distance_max�� �жϵ���ֱ���ϵ���������
% output��
%   inter_flag����¼�����Ƿ��ڸ�ֱ����
%   end_flag����¼�����Ƿ�Ϊ�ڸ�ֱ�ߵĶ˵㴦
%   near_point���������ĸ��˵����

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
        distance_project = ceil(abs(distance_max*sin(deg2rad(line1.theta)))); %distance_maxͶӰ��x���ϵĳ���
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