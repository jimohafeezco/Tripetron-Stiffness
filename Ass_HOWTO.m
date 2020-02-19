% Author: Popov Dmitry & Vladislav Berezhnoy
% Advanced Robotic Manipulation
% Homework 1
%
% Implements inverse kinematics
%

% Using:
% HOWTO(x,y,plot);
% Input: x,y - desired EE position , plot - if true than draw robot in plot
% Output: Q - 4x4 matrix with joints angles in rad

function [q] = HOWTO(px,py,pz)
a1 = 300e-3;
a2 = 300e-3;

c2 = (pz^2 + py^2 - a1^2 -a2^2)/(2*a1*a2);
s2 = sqrt(1-c2^2);
s1 = ((a1+a2*c2)*py-a2*s2*pz)/(py^2+py^2);
c1= ((a1+a2*c2)*py+a2*s2*pz)/(pz^2+py^2);

q1=px;
q2 = atan2(s2,c2);
q3 = atan2(s1,c1);
q4 = -q3-q2;
% q4 = -q3-q2;
% 

c2 = (px^2 + pz^2 - a1^2 -a2^2)/(2*a1*a2);
s2 = sqrt(1-c2^2);
s1 = ((a1+a2*c2)*px-a2*s2*pz)/(px^2+pz^2);
c1= ((a1+a2*c2)*px+a2*s2*pz)/(px^2+pz^2);

q5= py;
q6 = atan2(s2,c2);
q7 =atan2(s1,c1);
q8 = -q7-q6;

% 
c2 = (px^2 + py^2 - a1^2 -a2^2)/(2*a1*a2);
s2 = sqrt(1-c2^2);
s1 = ((a1+a2*c2)*px-a2*s2*py)/(px^2+py^2);
c1= ((a1+a2*c2)*px+a2*s2*py)/(px^2+py^2);

q9= pz;
q10 = atan2(s2,c2);
q11 =atan2(s1,c1);
q12 = -q11-q10;

[q] = [q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12];


end