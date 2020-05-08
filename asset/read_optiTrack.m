%% data.mat: time, x, y, z, q1, q2, q3, q4, roll, pitch, yaw
% [att1,att2,att3] = quat2angle((data(:,5:8)')','ZXY');
att1=[]; att2=[]; att3=[];
for i=1:1:length(data(:,1))
    q = data(i,5:8);    q = q/norm(q);
    att1=[att1;atan2(-2*(q(2)*q(3)-q(1)*q(4)), q(1)^2-q(2)^2+q(3)^2-q(4)^2)];
    att2=[att2;asin(2*(q(3)*q(4)+q(1)*q(2)))];
    att3=[att3;atan2(-2*(q(2)*q(4)-q(1)*q(3)), q(1)^2-q(2)^2-q(3)^2+q(4)^2)];
end

att1 = att1*180/pi; att2 = att2*180/pi; att3 = att3*180/pi;
for i=1:1:length(att1)
    if att1(i)<-100
        att1(i)=att1(i)+180;
    elseif att1(i)>100
        att1(i)=att1(i)-180;
    end
end

range = 1:length(data(:,1));
figure(1)
subplot(3,1,1); plot(range,-att1,range,data(:,9)); title('pitch'); axis tight;
subplot(3,1,2); plot(range,att2, range,data(:,10)); title('roll'); axis tight;
subplot(3,1,3); plot(range,-att3,range,data(:,11)); title('yaw');  axis tight;
legend('attitude optitrack','attitude CF');
