%https://blogs.mathworks.com/graphics/2014/10/21/double_pendulum/
classdef drawCarDrone < handle
    % Public methods
    properties (SetAccess=private)
        Car = gobjects(1,1);
        Uav = gobjects(3,1);
        Traces = gobjects(4,1)
        Angles = zeros(4,1);
        Position = zeros(4,2);
        AxisLengths = [200 200];
        AxisDim = [-50 250 -300 100];
        numUav=3;
    end
    methods
        function obj = drawCarDrone(axDim,numDrones)
            obj.numUav=numDrones;
            obj.AxisDim=axDim;
            obj.drawObjects();
            obj.updateFigure();
        end
        function setPose(obj,poseCar,poseUav)
            %poseUav-> 3xnumUav
            obj.Angles(1)=poseCar(3)-pi/2;
            obj.Position(1,:)=poseCar(1:2);
            
            for u=1:obj.numUav
                obj.Angles(u+1)=poseUav(3,u);
                obj.Position(u+1,:)=poseUav(1:2,u);
            end
            
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
            for i = 1:obj.numUav+1
                obj.Traces(i).addpoints(obj.Position(i,1),obj.Position(i,2));
            end
        end
        
        function drawObjects(obj)
             fig=figure;
             colorCar='b';
             lengthCar=12;
             colorUav(1)='r';
             colorUav(2)='g';
             colorUav(3)='y';
             lengthUav=5;
             ax = axes('Parent', fig);
             
             %Create the trace
             %Create the transforms
             %Create the car and UAV
             obj.Traces(1) = animatedline('Parent', ax, 'Color', colorCar,'LineWidth',1.5);
             obj.Car(1) = hgtransform('Parent', ax);
             drawCar(obj,obj.Car(1),lengthCar,colorCar);
             
             for u=1:obj.numUav
                obj.Traces(u+1) = animatedline('Parent', ax, 'Color', colorUav(u),'LineWidth',1.5);
                obj.Uav(u) = hgtransform('Parent', ax);
                drawUav(obj,obj.Uav(u),lengthUav,colorUav(u));
             end

             
             % Init axes
             ax.DataAspectRatio = [1,1,1];
             %ax.XLim = [-2 obj.AxisLengths(1)];
             %ax.YLim = [-2 obj.AxisLengths(1)];
             
             ax.XLim = obj.AxisDim(1:2);
             ax.YLim = obj.AxisDim(3:4);
        end
        
        function updateFigure(obj)
            carPos=obj.Position(1,:);
            carAng=obj.Angles(1,1);
            obj.Car(1).Matrix = makehgtform('translate',[carPos,0],'zrotate',carAng);
            
            for u=1:obj.numUav
                obj.Uav(u).Matrix=makehgtform('translate',[obj.Position(u+1,:),0],'zrotate',obj.Angles(u+1,1));
            end
        end
    end
end