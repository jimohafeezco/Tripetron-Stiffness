% Author: Popov Dmitry & Dmitrienko Margarita
% Innopolis University
% Advanced Robotic Manipulation
% Homework 4
%
% Calculate robot end-effector displacement
%
% Using:
% showDef_lin(F);
% Input: F - force vector (Fx,Fy,Fz,Mx,My,Mz) example: F = [100;00;0;0;0;0];
% Output: none 

% function showDef_lin(F)
F=[100 0 0 0 0 0];
x = -200:10:100;
y = -200:10:100;
z = -200:10:100;
z1 = zeros(length(x),length(y), length(z));


for i = 1:length(x)
%     disp(i/length(x)*100)
    for j = 1:length(y)
        % Calc robot stiffnes matrix
        for k= 1: length(z)
            K =VJM_lin_total(x(i)/1000, y(j)/1000,z(k)/1000);
            % Get deflections for all configurations
            dt=F*K;
            dr=sqrt(dt(1)^2+dt(2)^2+ dt(3)^2);
            z1(i,j,k) = dr;
        end

%         z4(i,j) = dr;
    end    
end

figure()
for kk=1: length(z1)
%     img = peaks(128);
%     contour(img,128)
    [M,c]=contour3(z1(:,:,kk));
    c.LineWidth=1;
%     daspect([1 1 1])
    colormap()
%     caxis([-10 10])
    colorbar
    hold on
end

