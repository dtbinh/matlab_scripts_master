function [x,u,flag] = optimFunc(A,B,Q,R,x0,u0,x_l,x_u,u_l,u_u,du_l,du_u,N)
%MPCFUNC General MPC controller
%   Detailed explanation goes here

mx = size(A,1);
mu = size(B,2);

% Generates constraints on measurements and inputs
[vlb,vub] = genBegr2(N,N,x_l,x_u,u_l,u_u);
[A_delta, b_delta] = genAdelta(du_l,du_u,N,mx,mu,u0);

% Generate matrices for quadprog
G = blkdiag(kron(eye(N), Q), kron(eye(N), R));
Aeq = gena2(A,B,N,mx,mu);
beq = zeros(mx*N,1);
beq(1:mx) = A*x0;

% Solve the optimization problem
[z,fval,flag] = quadprog(G, [], A_delta, b_delta, Aeq, beq, vlb, vub);

x=zeros(mx,N);
u=zeros(mu,N);
if flag == 1
    % Sort the data from the solution in to the x and u valiables
    k=1;
    % Get the x vectors
    for i=1:mx:mx*N+1-mx
        x(:,k)=z(i:i+mx-1);
        k=k+1;
    end
    k=1;
    % Get the u vectors
    for i=mx*N+1:mu:N*(mx+mu)+1-mu
        u(:,k)=z(i:i+mu-1);
        k=k+1;
    end      
end


end

