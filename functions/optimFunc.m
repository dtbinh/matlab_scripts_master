function [x,u,flag] = optimFunc(A,B,Q,R,x0,u0,x_l,x_u,u_l,u_u,du_l,du_u,N,type)
%MPCFUNC General MPC controller
%   Detailed explanation goes here

mx = size(A,1);
mu = size(B,2);

% Generates constraints on measurements and inputs
[vlb,vub] = genBegr2(N,N,x_l,x_u,u_l,u_u);
[A_delta, b_delta] = genAdelta(du_l,du_u,N,mx,mu,u0);

% Generate matrices for quadprog
G=getG(Q,R,N,type);

Aeq = gena2(A,B,N,mx,mu);
beq = zeros(mx*N,1);
beq(1:mx) = A*x0;

% Options
opt = optimoptions('quadprog','Display', 'off');

% Solve the optimization problem
[z,fval,flag] = quadprog(G, [], A_delta, b_delta, Aeq, beq, vlb, vub, [], opt);
%[z,fval,flag] = quadprog(G, [], [], [], Aeq, beq, vlb, vub, [], opt);

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
else
    error('No optimal solution found')
end

    function y = linearFunc(ss,se,N,i)
        y=(((se-ss)/(N-1))*(i-1)+ss);
    end

    function y = quadFunc(ss,se,N,i)
        b=0;
        c=(se-ss*N^2)/(1-N^2);
        a=ss-c;

        y=a*i^2+b*i+c;
    end

    function y = expFunc(a,N,i)
        y=(exp(a*i)-exp(a))/(exp(a*N)-exp(a));
    end

    function G = getG(Q,R,N,type)
        ss=0;
        se=1;
        Qi=cell(1,N);
        switch type
            case 'linear'
                for i=1:N
                    Qi{i}=linearFunc(ss,se,N,i)*Q;
                end
                G = blkdiag(blkdiag(Qi{:}), kron(eye(N), R));
            case 'quadratic'
                for i=1:N
                    Qi{i}=quadFunc(ss,se,N,i)*Q;
                end
                G = blkdiag(blkdiag(Qi{:}), kron(eye(N), R));
            case 'exponential'
                for i=1:N
                    Qi{i}=expFunc(0.3,N,i)*Q;
                end
                G = blkdiag(blkdiag(Qi{:}), kron(eye(N), R));
            otherwise
                G = blkdiag(kron(eye(N), Q), kron(eye(N), R));
        end
    end

end

