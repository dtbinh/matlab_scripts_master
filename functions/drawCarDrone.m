%https://blogs.mathworks.com/graphics/2014/10/21/double_pendulum/
classdef drawCarDrone < handle
    % Public methods
    properties (SetAccess=private)
        Car = gobjects(1,1);
        Uav = gobjects(1,1);
        Angles = [0,0]
        Position = [0, 0;0, 0];
        AxisLengths = [200 200];
        AxisDim = [-50 250 -300 100];
    end
    methods
        function obj = drawCarDrone(axDim)
            obj.AxisDim=axDim;
            obj.drawObjects();
            obj.updateFigure();
        end
        function setPose(obj,poseCar,poseUav)
            obj.Angles(1)=poseCar(3)-pi/2;
            obj.Position(1,:)=poseCar(1:2);
            
            obj.Angles(2)=poseUav(3);
            obj.Position(2,:)=poseUav(1:2);
            
            obj.updateFigure();
        end
    end
    
    % Private methods
    methods(Access=private)
        function drawCar(obj,parent,length,color)
            body=rectangle('Parent',parent);
            body.Position = [-length/4, -length/6, length/2, length];
            body.FaceColor = 'none';
            body.EdgeColor = color;

            fro=rectangle('Parent',parent);
            fro.Position = [-length/4, length/2, length/2, length/3];
            fro.FaceColor = color;
            fro.EdgeColor = 'none';
        end
        function drawUav(obj,parent,length,color)
            body=rectangle('Parent',parent);
            body.Curvature = [1 1];
            body.Position = [-length/2, -length/2, length, length];
            body.FaceColor = color;
            body.EdgeColor = 'none';

            %fro=rectangle('Parent',parent);
            %fro.Position = [-length/6, length/6, length/3, length/3];
            %fro.FaceColor = color;
            %fro.EdgeColor = 'none';
        end
        function drawObjects(obj)
             fig=figure;
             colorCar='b';
             lengthCar=12;
             colorUav='r';
             lengthUav=5;
             ax = axes('Parent', fig);
             
             %Create the transforms
             obj.Car(1) = hgtransform('Parent', ax);
             obj.Uav(1) = hgtransform('Parent', ax);
             
             % Create the car and UAV
             drawCar(obj,obj.Car(1),lengthCar,colorCar);
             drawUav(obj,obj.Uav(1),lengthUav,colorUav);
             
             % Init axes
             ax.DataAspectRatio = [1,1,1];
             %ax.XLim = [-2 obj.AxisLengths(1)];
             %ax.YLim = [-2 obj.AxisLengths(1)];
             
             ax.XLim = obj.AxisDim(1:2);
             ax.YLim = obj.AxisDim(3:4);
        end
        function updateFigure(obj)
            carPos=obj.Position(1,:);
            uavPos=obj.Position(2,:);
            carAng=obj.Angles(1,1);
            uavAng=obj.Angles(1,2);
            
            obj.Car(1).Matrix = makehgtform('translate',[carPos,0],'zrotate',carAng);
            obj.Uav(1).Matrix = makehgtform('translate',[uavPos,0],'zrotate',uavAng);
        end
    end
end