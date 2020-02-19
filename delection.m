close all; clear all; clc;
nx = 20;
ny = 20;
nz = 20;

x = linspace(200,300,nx);
y = linspace(200,300,ny);
z = linspace(200,300,nz);
[X,Y,Z] = meshgrid(x,y,z);
def = zeros(nx,ny,nz);


for i=1:nx
    for j=1:ny
        for k=1:nz
%             q = HOWTO1(nx/1000,ny/1000,nz/1000);
            kc = VJM_lin_total(nx/1000,ny/1000,nz/1000)
            tt = kc(1:3,1:3)
            [U,S,V] = svd(tt)
            def(i,j) = S(1,1);
        end
    end
end

contourf(def);

