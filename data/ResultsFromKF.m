close all
%Import data from ROS node
%take nr
take=1;
%test name
%tname='aruco_and_navigation_static';
%tname='closed_loop_static_1';
tname='closed_loop_static_2';
path=strcat(tname,'/take_',num2str(take),'/');

%/home/rosuser/Projects/matlab_scripts_master/data/aruco_and_navigation_static/take_1

addpath('../functions/MrcJkb-expandaxes-27975d8')
%% Aruco tag
t = readtable(strcat(path,'KfArUco.csv'),'Delimiter',';');
t.Var3=[];
%head(t,5)

KF_Aruco_pos_NED=zeros(3,height(t));
KF_ArUco_Time=t.KF_ArUco_Time'*1e-9;
for i=1:height(t)
    KF_Aruco_pos_NED(:,i)=sscanf(t.KF_ArUco_pos_NED{i},'%f');
end

clear t i
%% Landing pad
t = readtable(strcat(path,'KfLandingPad.csv'),'Delimiter',';');
t.Var4=[];
%head(t,5)

KF_LP_Time=t.KF_LP_Time'*1e-9;
KF_LP_vel_NED=zeros(3,height(t));
KF_rel_pos_NED=zeros(3,height(t));
for i=1:height(t)
    KF_LP_vel_NED(:,i)=sscanf(t.KF_LP_vel_NED{i},'%f');
    KF_rel_pos_NED(:,i)=sscanf(t.KF_rel_pos_NED{i},'%f');
end

clear t i

%% Kalman filter states
t = readtable(strcat(path,'KfStates.csv'),'Delimiter',';');
t.Var4=[];
%head(t,5)

KF_Time=t.KF_Time'*1e-9;
KF_States=zeros(9,height(t));
KF_Covariance=zeros(9,height(t));
for i=1:height(t)
    KF_States(:,i)=sscanf(t.KF_States{i},'%f');
    KF_Covariance(:,i)=sscanf(t.KF_Covariance{i},'%f');
end

clear t i

%% Plot the data

%Position
plotKfData2sigma(1,KF_ArUco_Time,KF_Aruco_pos_NED,KF_LP_Time,KF_rel_pos_NED,KF_Time,KF_States(1:3,:),'Relative position UAV-LP [m]','ArUco','GNSS',KF_Covariance(1:3,:));

%Velocity
plotKfData2sigma(2,[],[],KF_LP_Time,KF_LP_vel_NED,KF_Time,KF_States(4:6,:),'Velocity LP [m/s]',[],'GNSS',KF_Covariance(4:6,:));

%Bias
plotKfData2sigma(3,[],[],[],[],KF_Time,KF_States(7:9,:),'Bias [m]',[],[],KF_Covariance(7:9,:));

if 1==0
    saveas(gcf,'closed_loop_static_2/take_1/position','epsc')
end

function plotKfData2sigma(figure_nr, measured_1_t ,measured_1, measured_2_t ,measured_2, states_t, states, title_,title_meas1, title_meas2, varargin)
    %fontname='Linux Libertine O';
    %set(0,'DefaultAxesFontName',fontname,'DefaultTextFontName',fontname);
    tstart=0;
    tend=70;
    f=figure(figure_nr);
    
    defColor=get(gca,'colororder');
    
    %Finds first timestep
    if isempty([measured_1_t,measured_2_t])
        t0=states_t(1);
    elseif isempty(measured_1_t)
        t0=min([measured_2_t(1),states_t(1)]);
    else
        t0=min([measured_1_t(1),measured_2_t(1),states_t(1)]);
    end
    
    if nargin>10
        covariance=varargin{1};
        variance=sqrt(covariance);
    end
    
    for ap=1:3
        sp(ap)=subplot(3,1,ap);
        if ~isempty(measured_1_t)
            plot(measured_1_t-t0,measured_1(ap,:),'*','Color',defColor(1,:)) %Measurement 1
            hold on
        end
        if ~isempty(measured_2_t)
            plot(measured_2_t-t0,measured_2(ap,:),'--','Color',defColor(2,:)) %Measurement 2
            hold on
        end
        plot(states_t-t0,states(ap,:),'Color',defColor(3,:)) %Kalman filter
        if nargin >10
            hold on
            plot(states_t-t0,states(ap,:)+2*variance(ap,:),'k-.')
            plot(states_t-t0,states(ap,:)-2*variance(ap,:),'k-.')
        end
        if ap==1
            %title(title_)
            set(gca,'XTick',[])
            ylabel('North [m]')
        elseif ap==2
            set(gca,'XTick',[])
            ylabel('East [m]')
        else
            xlabel('Time [s]')
            ylabel('Down [m]')
        end
        hold off
        xlim([tstart tend])
    end
    if isempty([measured_1_t,measured_2_t])
        lgd=legend(sp(1),{'Kalman filter','2\sigma'},'Orientation','horizontal','location','NorthWest');
    elseif isempty(measured_1_t)
        lgd=legend(sp(1),{title_meas2,'Kalman filter','2\sigma'},'Orientation','horizontal','location','SouthEast');
    else
        lgd=legend(sp(1),{title_meas1,title_meas2,'Kalman filter','2\sigma'},'Orientation','horizontal','location','SouthEast');
    end
    legend(sp(1),'boxoff')
    %lgd.Position=lgd.Position+[0,.025,0,0];
    set(gcf,'units','normalized','outerposition',[0 0 .5 .75])
    
    %Remove whitespace
    sr=.06;
    su=.03;
    AX = findobj(gcf,'type','axes');
    pos = AX(1).Position;
    for i=1:3
        AX(i).Position(1) = pos(1)-sr;
        AX(i).Position(2) = pos(2)-su+(i-1)*(1/3-.5*su);
        AX(i).Position(3) = pos(3)+2*sr;
        AX(i).Position(4) = pos(4)+2*su;
    end    
end

function plotzoom(figure_nr, measured_1_t ,measured_1, measured_2_t ,measured_2, states_t, states, title_,title_meas1, title_meas2,varargin)
    tstart=25;
    tend=35;
    sel_axis=1;
    f=figure(figure_nr);
    
    defColor=get(gca,'colororder');
    
    %Finds first timestep
    if isempty([measured_1_t,measured_2_t])
        t0=states_t(1);
    elseif isempty(measured_1_t)
        t0=min([measured_2_t(1),states_t(1)]);
    else
        t0=min([measured_1_t(1),measured_2_t(1),states_t(1)]);
    end
    
    if nargin>10
        covariance=varargin{1};
        variance=sqrt(covariance);
    end
    
    for ap=1:3
        sp(ap)=subplot(3,1,ap);
        if ~isempty(measured_1_t)
            plot(measured_1_t-t0,measured_1(ap,:),'*','Color',defColor(1,:)) %Measurement 1
            hold on
        end
        if ~isempty(measured_2_t)
            plot(measured_2_t-t0,measured_2(ap,:),'--','Color',defColor(2,:)) %Measurement 2
            hold on
        end
        plot(states_t-t0,states(ap,:),'Color',defColor(3,:)) %Kalman filter
        if nargin >10
            hold on
            plot(states_t-t0,states(ap,:)+2*variance(ap,:),'k-.')
            plot(states_t-t0,states(ap,:)-2*variance(ap,:),'k-.')
        end
        if ap==1
            %title(title_)
            set(gca,'XTick',[])
            ylabel('North [m]')
        elseif ap==2
            set(gca,'XTick',[])
            ylabel('East [m]')
        else
            xlabel('Time [s]')
            ylabel('Down [m]')
        end
        hold off
        xlim([0 tend])
    end
    if isempty([measured_1_t,measured_2_t])
        lgd=legend(sp(1),{'Kalman filter'},'Orientation','horizontal','location','NorthWest');
    elseif isempty(measured_1_t)
        lgd=legend(sp(1),{title_meas2,'Kalman filter'},'Orientation','horizontal','location','SouthEast');
    else
        lgd=legend(sp(1),{title_meas1,title_meas2,'Kalman filter'},'Orientation','horizontal','location','SouthEast');
    end
    legend(sp(1),'boxoff')
    %lgd.Position=lgd.Position+[0,.025,0,0];
    set(gcf,'units','normalized','outerposition',[0 0 .5 .75])
    
    %Remove whitespace
    sr=.06;
    su=.03;
    AX = findobj(gcf,'type','axes');
    pos = AX(1).Position;
    for i=1:3
        AX(i).Position(1) = pos(1)-sr;
        AX(i).Position(2) = pos(2)-su+(i-1)*(1/3-.5*su);
        AX(i).Position(3) = pos(3)+2*sr;
        AX(i).Position(4) = pos(4)+2*su;
    end    

end

AX = findobj(gcf,'type','axes');
AX(1).YAxis.Limits=[-.1 .1];
AX(2).YAxis.Limits=[-.1 .1];
AX(3).YAxis.Limits=[-.1 .1];
AX(1).YAxis.Label.String='Down [m/s]';
AX(2).YAxis.Label.String='East [m/s]';
AX(3).YAxis.Label.String='North [m/s]';
pos = AX(1).Position;
AX(1).Position(1) = pos(1)+.01;
AX(2).Position(1) = pos(1)+.01;
AX(3).Position(1) = pos(1)+.01;
