function img_out = perspective_transformation(img, rect_in)
% 根据rect_in中的四边形投影变换到整个img上

[h2,w2,img_d] = size(img);
p1 = rect_in;
p2=[1,1;w2,1;w2,h2;1,h2]; % 左上、右上、右下、左下

T = calc_homography(p1,p2);   %计算单应性矩阵
T = maketform('projective',T);   %投影矩阵

img_out = imtransform(img,T,'XData',[1 w2],'YData',[1 h2]);     %投影
end