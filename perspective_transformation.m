function img_out = perspective_transformation(img, rect_in)
% ����rect_in�е��ı���ͶӰ�任������img��

[h2,w2,img_d] = size(img);
p1 = rect_in;
p2=[1,1;w2,1;w2,h2;1,h2]; % ���ϡ����ϡ����¡�����

T = calc_homography(p1,p2);   %���㵥Ӧ�Ծ���
T = maketform('projective',T);   %ͶӰ����

img_out = imtransform(img,T,'XData',[1 w2],'YData',[1 h2]);     %ͶӰ
end