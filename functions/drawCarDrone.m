%https://blogs.mathworks.com/graphics/2014/10/21/double_pendulum/
classdef drawCarDrone < handle
    % Public methods
    properties (SetAccess=private)
        Car = gobjects(1,1);
        Uav = gobjects(2,1);
        Traces = gobjects(3,1)
        Angles = zeros(3,1);
        Position = zeros(3,2);
        AxisLengths = [200 200];
        AxisDim = [-50 250 -300 100];
    end
    methods
        function obj = drawCarDrone(axDim)
            obj.AxisDim=axDim;
            obj.drawObjects();
            obj.updateFigure();
        end
        function setPose(obj,poseCar,poseUav1,poseUav2)
            obj.Angles(1)=poseCar(3)-pi/2;
            obj.Position(1,:)=poseCar(1:2);
            
            obj.Angles(2)=poseUav1(3);
            obj.Position(2,:)=poseUav1(1:2);
            
            obj.Angles(3)=poseUav2(3);
            obj.Position(3,:)=poseUav2(1:2);
            
            obj.addTracePoints();
            
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
        function addTracePoints(obj)
            for i = 1:3
                obj.Traces(i).addpoints(obj.Position(i,1),obj.Position(i,2));
            end
        end
        
        function drawObjects(obj)
             fig=figure;
             colorCar='b';
             lengthCar=12;
             colorUav(1)='r';
             colorUav(2)='g';
             lengthUav=5;
             ax = axes('Parent', fig);
             
             % Create the trace
             obj.Traces(1) = animatedline('Parent', ax, 'Color', colorCar);
             obj.Traces(2) = animatedline('Parent', ax, 'Color', colorUav(1));
             obj.Traces(3) = animatedline('Parent', ax, 'Color', colorUav(2));
             
             %Create the transforms
             obj.Car(1) = hgtransform('Parent', ax);
             obj.Uav(1) = hgtransform('Parent', ax);
             obj.Uav(2) = hgtransform('Parent', ax);
             
             % Create the car and UAV
             drawCar(obj,obj.Car(1),lengthCar,colorCar);
             drawUav(obj,obj.Uav(1),lengthUav,colorUav(1));
             drawUav(obj,obj.Uav(2),lengthUav,colorUav(2));
             
             % Init axes
             ax.DataAspectRatio = [1,1,1];
             %ax.XLim = [-2 obj.AxisLengths(1)];
             %ax.YLim = [-2 obj.AxisLengths(1)];
             
             ax.XLim = obj.AxisDim(1:2);
             ax.YLim = obj.AxisDim(3:4);
        end
        function updateFigure(obj)
            carPos=obj.Position(1,:);
            uavPos1=obj.Position(2,:);
            uavPos2=obj.Position(3,:);
            carAng=obj.Angles(1,1);
            uavAng1=obj.Angles(2,1);
            uavAng2=obj.Angles(3,1);
            
            obj.Car(1).Matrix = makehgtform('translate',[carPos,0],'zrotate',carAng);
            obj.Uav(1).Matrix = makehgtform('translate',[uavPos1,0],'zrotate',uavAng1);
            obj.Uav(2).Matrix = makehgtform('translate',[uavPos2,0],'zrotate',uavAng2);
        end
    end
end