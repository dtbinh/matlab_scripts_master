classdef kalmanFilter2 < handle
    %Kalman filter for position and velocity estimation including
    %estimation of the bias error from GNSS.
    %   Detailed explanation goes here
    
    properties (SetAccess = public)
        H
        z
        x_hat
        x_hat_pri
        P_pri
        P
        Phi
        Delta
        u
        Q
        R
    end
    
    methods
        function obj =  kalmanFilter2(Q,R,x0,P0)
            obj.Q=Q;
            obj.R=R;
            obj.x_hat=x0;
            obj.P=P0;
        end
        
        function projectAhead(obj,dt)
            %Update phi and delta:
            [obj.Phi,obj.Delta]=obj.updatePhiDelta(dt);
            
            % Project ahead:
            obj.x_hat_pri=obj.Phi*obj.x_hat+obj.Delta*obj.u;
            obj.P_pri=obj.Phi*obj.P*transpose(obj.Phi)+obj.Q;
        end
        
        function updateMeasurement(obj,type,measurement,varargin)
            % Update the H  z matrix
            obj.H=obj.getH(type);
            obj.z=obj.getZ(type,measurement);
            
            if nargin>3
                covariance=varargin{1};
                obj.R=obj.updateR(type,covariance);
            end
            
            obj.updateFilter();
        end
    %end
    
    %methods(Access=private)
        function updateFilter(obj)
            
            % Compute Kalman gain:
            K=obj.P_pri*transpose(obj.H)*inv(obj.H*obj.P_pri*transpose(obj.H)+obj.R);

            % Update estimate with measurement z:
            I=eye(9);
            obj.x_hat=obj.x_hat_pri+K*(obj.z-obj.H*obj.x_hat_pri);
            obj.P=(I-K*obj.H)*obj.P_pri*transpose(I-K*obj.H)+K*obj.R*transpose(K);
            
            % Set predictionas a priori if there is several measurement
            % updates
            obj.x_hat_pri=obj.x_hat;
            obj.P_pri=obj.P;
        end
        
        function [Phi,Delta] = updatePhiDelta(obj,dt)
            Phi=eye(9);
            Phi(1:3,4:6)=eye(3)*dt;

            Delta=[-eye(3)*dt;zeros(6,3)];
        end

        function z = getZ(obj,measure,data)
            z=zeros(9,1);
            if measure=="Aruco"
                z(4:6)=data;
            elseif measure=="LP_pos"
                z(1:3)=data;
            elseif measure=="LP_vel"
                z(7:9)=data;
            end
        end

        function H = getH(obj,measure)
            H=zeros(9,9);
            if measure=="LP_pos"
                H(1:3,:)=[eye(3,3),zeros(3,3),eye(3,3)];
            elseif measure=="Aruco"
                H(4:6,:)=[eye(3,3),zeros(3,3),zeros(3,3)];
            elseif measure=="LP_vel"
                H(7:9,:)=[zeros(3,3),eye(3,3),zeros(3,3)];
            end
        end
        
        function R = updateR(obj,measure,covariance)
            tempR=obj.R;
            if measure=="LP_pos"
                tempR(1:3,1:3)=covariance;
            elseif measure=="Aruco"
                tempR(4:6,4:6)=covariance;
            elseif measure=="LP_vel"
                tempR(7:9,7:9)=covariance;
            end
            R=tempR;
        end
    end
end

