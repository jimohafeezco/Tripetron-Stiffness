close all; clear all; clc;

% Q1 = 0; Q2 = 0; Q3 = 0; Q4 = 0; Q5 = 0; Q6 = 0;
d1 = 10*10e-3; d2 = 10*10e-3;
l1 = 300e-3; l2 = 300e-3; 
% q=[0.3, 0.4, 0.2];
q=HOWTO1(0.1,0.2,0.1);
px=0.2;
Tbase = eye(4); Ttool=eye(4);
kac = 10e5;
% d6=[1 0 0;
%     0 1 0;
%     0 0 0];
Dd=[1 0 0;
    0 1 0;
    0 0 0];
De= [eye(3,3) Dd; zeros(3,3) eye(3,3)];

% // rx=Rx(q(2));
% // R11=rx(1,1)   ; R12=rx(1,2) ; R13=rx(1,3);
% // R21=rx(2,1) ;R22=rx(2,2); R23=rx(2,3);
% // R31=rx(3,1); R32=rx(3,2); R33=rx(3,3);

rx3=Rz(q(10));
% // R11_2=rx2(1,1)   ; R12_2=rx2(1,2) ; R13_2=rx2(1,3);
% // R21_2=rx2(2,1) ;R22_2=rx2(2,2); R23_2=rx2(2,3);
% // R31_2=rx2(3,1); R32_2=rx2(3,2); R33_2=rx2(3,3);


rx4=Rz(q(11));
% // R11_3=rx3(1,1); R12_3=rx3(1,2) ; R13_3=rx3(1,3);
% // R21_3=rx3(2,1) ;R22_3=rx3(2,2); R23_3=rx3(2,3);
% // R31_3=rx3(3,1); R32_3=rx3(3,2); R33_3=rx3(3,3);

% // % rx3=Rx(q(2));
Q6= [rx3(1:3,1:3), zeros(3,3);
    zeros(3,3),rx3(1:3 ,1:3)
    ];

Q7= [rx4(1:3,1:3), zeros(3,3);
    zeros(3,3),rx4(1:3,1:3);
    ];

k23_11 = Q6*k11_calc(d1,l1)*Q6'; k23_12 = Q6*k12_calc(d2,l1)*Q6'; k23_21 = Q6*k23_12'*Q6'; k23_22 = Q6*k12_calc(d1,l1)*Q6';
k45_11 = Q7*k11_calc(d2,l2)*Q7'; k45_12 = Q7*k12_calc(d2,l2)*Q7'; k45_21 = Q7*k45_12'*Q7'; k45_22 = Q7*k22_calc(d2,l2)*Q7';

lambda_01_e = [0 0 0 1 0 0 ]; 
% lambda_01_x = [1 0 0; 0 1 0];
lambda_01_r = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0;0 0 0 0 1 0; 0 0 0 0 0 1];

lambda_12_r =  [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0;0 0 0 1 0  0; 0 0 0 0 1 0 ];
lambda_34_r = lambda_01_r;
% // % lambda_34_r = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 R11_2 R12_2 R13_2; 0 0 0 R21_2 R22_2 R23_2 ];
lambda_56_r = lambda_34_r;
% // % lambda_56_r = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 R11_3 R12_3 R13_3; 0 0 0 R21_3 R22_3 R23_3 ];
lambda_p = [0 0 0 0 0 1];

X = [zeros(6,6)    zeros(6,6)    zeros(6,6)     zeros(6,6)     zeros(6,6)     zeros(6,6)   zeros(6,6)    zeros(6,6)     eye(6,6)     zeros(6,6)        zeros(6,6)   zeros(6,6)    zeros(6,6)   zeros(6,6)   zeros(6,6)   zeros(6,6);
     zeros(6,6)     zeros(6,6)    -eye(6,6)     zeros(6,6)     zeros(6,6)     zeros(6,6)   zeros(6,6)    zeros(6,6)    zeros(6,6)    zeros(6,6)           k23_11        k23_12    zeros(6,6)   zeros(6,6)   zeros(6,6)   zeros(6,6);
     zeros(6,6)    zeros(6,6)     zeros(6,6)    -eye(6,6)     zeros(6,6)     zeros(6,6)   zeros(6,6)    zeros(6,6)    zeros(6,6)    zeros(6,6)           k23_21        k23_22    zeros(6,6)   zeros(6,6)   zeros(6,6)   zeros(6,6);
     zeros(6,6)    zeros(6,6)    zeros(6,6)     zeros(6,6)     -eye(6,6)     zeros(6,6)   zeros(6,6)    zeros(6,6)    zeros(6,6)    zeros(6,6)        zeros(6,6)    zeros(6,6)     k45_11      k45_12      zeros(6,6)   zeros(6,6);
     zeros(6,6)    zeros(6,6)    zeros(6,6)     zeros(6,6)      zeros(6,6)     -eye(6,6)   zeros(6,6)    zeros(6,6)    zeros(6,6)    zeros(6,6)        zeros(6,6)    zeros(6,6)     k45_21      k45_12      zeros(6,6)   zeros(6,6);
     zeros(6,6)    zeros(6,6)    zeros(6,6)     zeros(6,6)     zeros(6,6)     zeros(6,6)   zeros(6,6)    zeros(6,6)    zeros(6,6)    zeros(6,6)       zeros(6,6)    zeros(6,6)   zeros(6,6)  zeros(6,6)       eye(6,6)         -eye(6,6);%De
     zeros(6,6)    zeros(6,6)    zeros(6,6)     zeros(6,6)     zeros(6,6)     zeros(6,6)     eye(6,6)        De'       zeros(6,6)    zeros(6,6)        zeros(6,6)    zeros(6,6)   zeros(6,6)  zeros(6,6)    zeros(6,6)   zeros(6,6);                                                                                                                                   
     zeros(5,6)    zeros(5,6)    zeros(5,6)     zeros(5,6)     zeros(5,6)     zeros(5,6)   zeros(5,6)    zeros(5,6)    lambda_01_r   -lambda_01_r      zeros(5,6)    zeros(5,6)   zeros(5,6)  zeros(5,6)    zeros(5,6)   zeros(5,6);
       eye(6,6)      eye(6,6)    zeros(6,6)     zeros(6,6)     zeros(6,6)     zeros(6,6)   zeros(6,6)    zeros(6,6)    zeros(6,6)    zeros(6,6)        zeros(6,6)    zeros(6,6)   zeros(6,6)  zeros(6,6)    zeros(6,6)   zeros(6,6);
    lambda_01_e    zeros(1,6)    zeros(1,6)     zeros(1,6)     zeros(1,6)     zeros(1,6)   zeros(1,6)    zeros(1,6)  lambda_01_e*kac -lambda_01_e*kac  zeros(1,6)    zeros(1,6)   zeros(1,6)  zeros(1,6)    zeros(1,6)   zeros(1,6);
     zeros(5,6)    zeros(5,6)    zeros(5,6)     zeros(5,6)     zeros(5,6)     zeros(5,6)   zeros(5,6)    zeros(5,6)    zeros(5,6)     lambda_12_r     lambda_12_r    zeros(5,6)   zeros(5,6)  zeros(5,6)    zeros(5,6)   zeros(5,6);
     zeros(5,6)   lambda_12_r    lambda_12_r    zeros(5,6)     zeros(5,6)     zeros(5,6)   zeros(5,6)    zeros(5,6)    zeros(5,6)     zeros(5,6)       zeros(5,6)    zeros(5,6)   zeros(5,6)  zeros(5,6)    zeros(5,6)   zeros(5,6);
     zeros(1,6)     lambda_p     zeros(1,6)     zeros(1,6)     zeros(1,6)     zeros(1,6)   zeros(1,6)    zeros(1,6)    zeros(1,6)     zeros(1,6)       zeros(1,6)    zeros(1,6)   zeros(1,6)  zeros(1,6)    zeros(1,6)   zeros(1,6);
     zeros(1,6)    zeros(1,6)     lambda_p      zeros(1,6)     zeros(1,6)     zeros(1,6)   zeros(1,6)    zeros(1,6)    zeros(1,6)     zeros(1,6)       zeros(1,6)    zeros(1,6)   zeros(1,6)  zeros(1,6)    zeros(1,6)   zeros(1,6);
     zeros(5,6)    zeros(5,6)    zeros(5,6)     zeros(5,6)     zeros(5,6)     zeros(5,6)   zeros(5,6)    zeros(5,6)    zeros(5,6)     zeros(5,6)       zeros(5,6)   lambda_34_r  -lambda_34_r zeros(5,6)    zeros(5,6)   zeros(5,6);
     zeros(5,6)    zeros(5,6)    zeros(5,6)     lambda_34_r   lambda_34_r     zeros(5,6)   zeros(5,6)    zeros(5,6)    zeros(5,6)     zeros(5,6)       zeros(5,6)    zeros(5,6)   zeros(5,6)  zeros(5,6)    zeros(5,6)   zeros(5,6);
     zeros(1,6)    zeros(1,6)    zeros(1,6)     lambda_p       zeros(1,6)     zeros(1,6)   zeros(1,6)    zeros(1,6)    zeros(1,6)     zeros(1,6)       zeros(1,6)    zeros(1,6)   zeros(1,6)  zeros(1,6)    zeros(1,6)   zeros(1,6);
     zeros(1,6)    zeros(1,6)    zeros(1,6)     zeros(1,6)      lambda_p      zeros(1,6)   zeros(1,6)    zeros(1,6)    zeros(1,6)     zeros(1,6)       zeros(1,6)    zeros(1,6)   zeros(1,6)  zeros(1,6)    zeros(1,6)   zeros(1,6);
     zeros(5,6)    zeros(5,6)    zeros(5,6)     zeros(5,6)     zeros(5,6)     zeros(5,6)   zeros(5,6)    zeros(5,6)    zeros(5,6)     zeros(5,6)       zeros(5,6)    zeros(5,6)   zeros(5,6)  lambda_56_r  -lambda_56_r  zeros(5,6);
     zeros(5,6)    zeros(5,6)    zeros(5,6)     zeros(5,6)     zeros(5,6)     lambda_56_r  lambda_56_r   zeros(5,6)    zeros(5,6)     zeros(5,6)       zeros(5,6)    zeros(5,6)   zeros(5,6)  zeros(5,6)    zeros(5,6)   zeros(5,6);
     zeros(1,6)    zeros(1,6)    zeros(1,6)     zeros(1,6)     zeros(1,6)      lambda_p    zeros(1,6)    zeros(1,6)    zeros(1,6)     zeros(1,6)       zeros(1,6)    zeros(1,6)   zeros(1,6)  zeros(1,6)    zeros(1,6)   zeros(1,6);
     zeros(1,6)    zeros(1,6)    zeros(1,6)     zeros(1,6)     zeros(1,6)     zeros(1,6)    lambda_p     zeros(1,6)    zeros(1,6)     zeros(1,6)       zeros(1,6)    zeros(1,6)   zeros(1,6)  zeros(1,6)    zeros(1,6)   zeros(1,6);
     zeros(6,6)    zeros(6,6)    zeros(6,6)     zeros(6,6)     zeros(6,6)     zeros(6,6)     zeros(6,6)    eye(6,6)   zeros(6,6)     zeros(6,6)       zeros(6,6)    zeros(6,6)   zeros(6,6)  zeros(6,6)    zeros(6,6)   zeros(6,6)
];
  

A = X(1:90, 1:90);   

B = X(1:90, 91:96);        
C = X(91:96,1:90);        
D = X(91:96,91:96);
A_inv = pinv(A);
Kc = D - C*(A_inv)*B
% save('MSA_Model','Kc','A','B','C','D');




function[k] = k11_calc(d, L)
E = 70e9; % Young's modulus
G = 25.5e9; % Shear modulus

Iy = (pi*d*d*d*d)/64;
Iz = (pi*d*d*d*d)/64;
J = Iy + Iz;
S = (pi*d*d)/4;

J = Iy + Iz;
k = [E*S/L       0                  0                0           0          0;
    0           12*E*Iz/L^3         0                0           0          6*E*Iz/L^2;
    0           0                  12*E*Iy/L^3       0       -6*E*Iy/L^2    0;
    0           0                  0                 G*J/L       0          0;
    0           0                  -6*E*Iy/L^2       0       4*E*Iy/L       0;
    0           6*E*Iz/L^2         0                 0           0          4*E*Iz/L];
end


function [k] = k12_calc(d, L)
E = 70e9; % Young's modulus
G = 25.5e9; % Shear modulus

Iy = (pi*d*d*d*d)/64;
Iz = (pi*d*d*d*d)/64;
J = Iy + Iz;
S = (pi*d*d)/4;

J = Iy + Iz;
k = [-E*S/L       0                  0                0           0          0;
    0           -12*E*Iz/L^3         0                0           0          6*E*Iz/L^2;
    0           0                  -12*E*Iy/L^3       0       -6*E*Iy/L^2    0;
    0           0                  0                 -G*J/L       0          0;
    0           0                  6*E*Iy/L^2       0       2*E*Iy/L       0;
    0           -6*E*Iz/L^2         0                 0           0          2*E*Iz/L];
end

function [k] = k22_calc(d, L)
E = 70e9; % Young's modulus
G = 25.5e9; % Shear modulus

Iy = (pi*d*d*d*d)/64;
Iz = (pi*d*d*d*d)/64;
J = Iy + Iz;
S = (pi*d*d)/4;

k = [E*S/L       0                  0                0           0          0;
    0           12*E*Iz/L^3         0                0           0          -6*E*Iy/L^2;
    0           0                  12*E*Iy/L^3       0       6*E*Iy/L^2    0;
    0           0                  0                 G*J/L       0          0;
    0           0                  6*E*Iy/L^2       0       4*E*Iy/L       0;
    0           -6*E*Iy/L^2         0                 0           0          4*E*Iz/L];
end



function T = Tx(s)

T = [   1    0   0    s; ...
        0    1   0    0; ...
        0    0   1    0; ...
        0    0   0    1];
end
function T = Ty(s)

T = [   1    0   0    0; ...
        0    1   0    s; ...
        0    0   1    0; ...
        0    0   0    1];
end
function T = Tz(s)

T = [   1    0   0    0; ...
        0    1   0    0; ...
        0    0   1    s; ...
        0    0   0    1];
end
function T = Rx(q)

Sq = sin(q);    Cq = cos(q);

T = [   1   0    0     0; ...
        0   Cq  -Sq    0; ...
        0   Sq   Cq    0; ...
        0   0    0     1];
end
function T = Ry(q)

Sq = sin(q);    Cq = cos(q);

T = [  Cq   0    Sq    0; ...
        0    1    0     0; ...
       -Sq   0   Cq    0; ...
        0    0    0     1];
end
function T = Rz(q)

Sq = sin(q);    Cq = cos(q);

T = [  Cq   -Sq   0     0; ...
       Sq   Cq    0    0; ...
        0   Sq    1    0; ...
        0   0     0    1];
end
function rx = Rxd(phi)
rx = [ 0,         0,         0, 0
 0, -sin(phi),  cos(phi), 0
 0, -cos(phi), -sin(phi), 0
 0,         0,         0, 0];
 
end
function rx = Ryd(phi)
rx = [ -sin(phi), 0,  cos(phi), 0
        0, 0,         0, 0
 -cos(phi), 0, -sin(phi), 0
         0, 0,         0, 0];
 
end
function rx = Rzd(phi)
rx = [ -sin(phi),  cos(phi), 0, 0
 -cos(phi), -sin(phi), 0, 0
         0,         0, 0, 0
         0,         0, 0, 0];
 
end
function tx = Txd(x)
tx = [0 0 0 1
    0 0 0 0
    0 0 0 0
    0 0 0 0];
end
function tx = Tyd(x)
tx = [0 0 0 0
    0 0 0 1
    0 0 0 0
    0 0 0 0];
end
function tx = Tzd(x)
tx = [0 0 0 0
    0 0 0 0
    0 0 0 1
    0 0 0 0];
end