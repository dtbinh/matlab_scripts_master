%close all
%Import data from ROS node
%take nr
take=1;
%test name
%tname='aruco_and_navigation_static';
%tname='closed_loop_static_1';
%tname='closed_loop_static_2';
tname='closed_loop_static_3';
%tname='closed_loop_dynamic_1';
path=strcat(tname,'/take_',num2str(take),'/');

addpath('../functions/')

%/home/rosuser/Projects/matlab_scripts_master/data/aruco_and_navigation_static/take_1
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

%% State machine data
t = readtable(strcat(path,'StateMachine.csv'),'Delimiter',';');
t.Var3=[];
%head(t,5)

SM_Time=t.SM_Time'*1e-9;
SM_Status=t.Status';

s_name={'Intercept','Hover','Lower','Adjust','Final','Land','Landed'};
SM_change=[];
SM_change_name={};
j=1;
prev=SM_Status(1);
for i=2:length(SM_Time)
    if prev~=SM_Status(i)
        SM_change(j)=SM_Time(i);
        SM_change_name(j)=s_name(SM_Status(i));
        j=j+1;
    end
    prev=SM_Status(i);
end

tmp=[];
%Remove Hover and Landed
rm={'Hover','Landed'};
for i=1:length(SM_change_name)
    for j=1:length(rm)
        if strcmp(SM_change_name(i),rm(j))
            tmp(end+1)=i;
        end
    end
end
SM_change_name(tmp)=[];
SM_change(tmp)=[];


clear t i j prev s_name

%% Plot the data
tstart=0;
if take==1
    tend=67;
elseif take==2
    tend=65;
end
f=figure(1);

defColor=get(gca,'colororder');

%Finds first timestep
t0=KF_Time(1);


for ap=1:3
    sp(ap)=subplot(3,1,ap);

    plot(KF_Time-t0,KF_States(ap,:),'Color',defColor(1,:)) %Kalman filter states
    %grid on
    if ap==1
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
    
    

    %Plot vertical lines
    if take==1
        if ap==1
            ylim([-15 3])
        elseif ap==2
            ylim([-7 1])
        else
            ylim([-2 17])
        end
    elseif take==2
        if ap==1
            ylim([-10 3])
        elseif ap==2
            ylim([-8 2])
        else
            ylim([-2 17])
        end
    end

    if ap==3
        vline(SM_change-t0,'r--',SM_change_name);
    else
        vline(SM_change-t0,'r--');
    end
end


lgd=legend(sp(1),{'Relative position'},'Orientation','horizontal','location','NorthWest');

legend(sp(1),'boxoff')

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


    
if 1==0
    saveas(gcf,'closed_loop_static_3/take_1/controller','epsc')
end
