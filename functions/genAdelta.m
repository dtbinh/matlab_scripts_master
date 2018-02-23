function [A,b] = genAdelta(du_l,du_u,N,mx,mu, u0)

% Function to buld the A_delta matrix to set constraints on the cange of
% input (delta u)
% 
% The state vector must include the states x in addition to the control 
% input u
%
% Vegard Line 
% NTNU 2018
%
%    -    -
%    |x_1 |
%    |x_2 |
%    |x_..|
%    |x_mx|
% x= |u_1 |                                                           
%    |u_2 |                                                           
%    |u_..|
%    |u_mu|
%    -    -
%
% The output matrix A
%      -                    -                                   
%      |         |1         |                                   
%  A = |         |-1        |                                   
%      |         |-1  1     |                                   
%      |    0    |1  -1     |                                   
%      |         |   -1  1  |                                   
%      |         |    1 -1  |                                   
%      -                    -                                   
%
% The output matrix b
%
%     -         -
%     |du_u+u_0 |
%     |du_l-u_0 |
%  b= |du_u     |
%     |du_l     |
%     |du_u     |
%     |du_l     |
%     |..       |
%     -        -
%
% [A,b] = genAdelta(du_l,du_u,N,mx,mu, u0)
%                                                               
% du_l - Lower limit delta u 
% du_u - Upper limit delta u
% N  - Estimation length
% mx - Number of states                                        
% mu - Number of inputs
% u0 - Initial control input

% Test
% mu=3;
% mx=2;
% du_l=-10*ones(mu,1);
% du_u=10*ones(mu,1);
% u0=7*ones(mu,1);
% N=4;

A=zeros(N*2*mu,N*(mx+mu));
b=zeros(N*2*mu,1);

A(1:2*mu,N*mx+1:N*mx+mu)=[eye(mu);-eye(mu)];
b(1:2*mu,1)=[du_u+u0;-du_l-u0];

for i=1:N-1
    A(i*2*mu+1:2*mu*(i+1),N*mx+1+(i-1)*mu:N*mx+(i+1)*mu)=[-eye(mu) eye(mu);eye(mu) -eye(mu)];
    b(i*2*mu+1:(i+1)*2*mu,1)=[du_u;-du_l];
end