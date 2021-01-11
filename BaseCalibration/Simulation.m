% Section IV Simulation

% Please Refer to:

% Liao Wu, Hongliang Ren. Finding the kinematic base frame of a robot by 
% hand-eye calibration using 3D position data. IEEE Transactions on 
% Automation Science and Engineering. 2017, 14(1): 314-324.

close all;
clear;
clc;

%% configure simulation
k=0; %

c=0.001;  %unit coefficient: mm c=1; m c=0.001
        
%default parameters
M=50; %measurement number
KR_BH = 1; %the viariation range of the robot input q_rob
Kt_BH = 1; %the robot size
Kt_BW = 1; %the distance from the base frame to the world frame
KP_H=1; %the position of the marker in the hand frame

%select one line
% for M=[6:10,20:10:100] %Fig 2
for KR_BH = 0.1:0.1:1 %Fig 3(a)
% for Kt_BH = 0.1:0.1:1 %Fig 3(b)
% for Kt_BW = [0.1:0.1:1,2:1:10] %Fig 3(c)
% for KP_H = [0.1:0.1:1,2:1:10] %Fig 4

    N=100; %Trial Number
    dr_BW = zeros(1,N); dP_H = zeros(1,N); dt_BW = zeros(1,N); dt_WB = zeros(1,N);
    dr_BW_init = zeros(1,N); dP_H_init = zeros(1,N); dt_BW_init = zeros(1,N); dt_WB_init = zeros(1,N);

    R_BW_id=rotx(pi/3)*roty(pi/3)*rotz(pi/3); %rotation from base of robot to world
    t_BW_id=[2000;100;-1000]*c*Kt_BW; %translation from base of robot to world
    P_H_id=[100;100;100]*c*KP_H; %translation from hand of robot to marker

    t_WB_id=-R_BW_id'*t_BW_id; %translation from world to base of robot

    % no noise
    % RotationNoiseScale_R_BH = 0/180*pi;  
    % TranslationNoiseScale_t_BH = 0*c;  
    % TranslationNoiseScale_P_W = 0*c; 

    % low level noise
    % RotationNoiseScale_R_BH = 0.02/180*pi;  
    % TranslationNoiseScale_t_BH = 0.06*c;  
    % TranslationNoiseScale_P_W = 0.06*c; 

    % medium level noise
     RotationNoiseScale_R_BH = 0.1/180*pi;  
     TranslationNoiseScale_t_BH = 0.3*c;  
     TranslationNoiseScale_P_W = 0.3*c;  

    % high level noise
    % RotationNoiseScale_R_BH = 0.5/180*pi;  
    % TranslationNoiseScale_t_BH = 1.5*c;  
    % TranslationNoiseScale_P_W = 1.5*c;  

%% start simulation
    tic
    for j=1:N
        %% prepare data
        g_BH_id=zeros(4,4,M);R_BH_id=zeros(3,3,M);R_BH=zeros(3,3,M);t_BH_id=zeros(3,M);t_BH=zeros(3,M);P_W=zeros(3,M);
        for i=1:M
            %ideal measurements
            g_BH_id(:,:,i)=LBRfkine((rand(7,1)*2-1)*pi*KR_BH);
            R_BH_id(:,:,i)=g_BH_id(1:3,1:3,i);
            %add noise
            RotationAxis_R_BH_noise = 1-2*rand(3,1); RotationAxis_R_BH_noise = RotationAxis_R_BH_noise / norm(RotationAxis_R_BH_noise);
            RotationAngle_R_BH_noise = RotationNoiseScale_R_BH*rand(1);
            R_BH(:,:,i)=R_BH_id(:,:,i)*rotationMatrix(RotationAxis_R_BH_noise,RotationAngle_R_BH_noise);
            t_BH_id(:,i)=g_BH_id(1:3,4,i)*1000*c*Kt_BH;
            TransDir_t_BH_noise = 1-2*rand(3,1); TransDir_t_BH_noise = TransDir_t_BH_noise / norm(TransDir_t_BH_noise);
            TransAmp_t_BH_noise = TranslationNoiseScale_t_BH*rand(1);
            t_BH(:,i)=t_BH_id(:,i)+TransAmp_t_BH_noise*TransDir_t_BH_noise;
            TransDir_P_W_noise = 1-2*rand(3,1); TransDir_P_W_noise = TransDir_P_W_noise / norm(TransDir_P_W_noise);
            TransAmp_P_W_noise = TranslationNoiseScale_P_W*rand(1);
            P_W(:,i)=R_BW_id'*(R_BH_id(:,:,i)*P_H_id+t_BH_id(:,i)-t_BW_id)+TransAmp_P_W_noise*TransDir_P_W_noise;
        end
        %% closed-form/initial solution
        [ R_BW_init, t_BW_init, P_H_init ] = closedForm( R_BH, t_BH, P_W );

        %evaluate closed-form solution 
        dR_BW_init=R_BW_id'*R_BW_init;
        dr_BW_init(j)=norm([dR_BW_init(2,3);dR_BW_init(1,3);dR_BW_init(1,2)])*1000; %mrad
        dP_H_init(j)=norm(P_H_init-P_H_id)/c;
        dt_BW_init(j)=norm(t_BW_init-t_BW_id)/c;

        t_WB_init=-R_BW_init'*t_BW_init;
        dt_WB_init(j)=norm(t_WB_init-t_WB_id)/c;

        %% iterative solution
        [ R_BW, t_BW, P_H ] = iterative( R_BH, t_BH, P_W, R_BW_init, t_BW_init, P_H_init );

        %evaluate iterative solution
        dR_BW=R_BW_id'*R_BW;
        dr_BW(j)=norm([dR_BW(2,3);dR_BW(1,3);dR_BW(1,2)])*1000; %mrad
        dP_H(j)=norm(P_H-P_H_id)/c;
        dt_BW(j)=norm(t_BW-t_BW_id)/c;

        t_WB=-R_BW'*t_BW;
        dt_WB(j)=norm(t_WB-t_WB_id)/c;

    end

    toc
    k = k+1;
    
    %statistics
    meandr_BW_init(k) = mean(dr_BW_init);
    meandt_BW_init(k) = mean(dt_BW_init);
    meandt_WB_init(k) = mean(dt_WB_init);
    meandP_H_init(k) = mean(dP_H_init);

%     maxdr_BW_init(k) = max(dr_BW_init);
%     maxdt_BW_init(k) = max(dt_BW_init);
%     maxdt_WB_init(k) = max(dt_WB_init);
%     maxdP_H_init(k) = max(dP_H_init);

%     mindr_BW_init(k) = min(dr_BW_init);
%     mindt_BW_init(k) = min(dt_BW_init);
%     mindt_WB_init(k) = min(dt_WB_init);
%     mindP_H_init(k) = min(dP_H_init);

    meandr_BW(k) = mean(dr_BW);
    meandt_BW(k) = mean(dt_BW);
    meandt_WB(k) = mean(dt_WB);
    meandP_H(k) = mean(dP_H);

%     maxdr_BW(k) = max(dr_BW);
%     maxdt_BW(k) = max(dt_BW);
%     maxdt_WB(k) = max(dt_WB);
%     maxdP_H(k) = max(dP_H);
% 
%     mindr_BW(k) = min(dr_BW);
%     mindt_BW(k) = min(dt_BW);
%     mindt_WB(k) = min(dt_WB);
%     mindP_H(k) = min(dP_H);

 end

%% plot
figure()
hold on
plot(1:k,meandr_BW);
plot(1:k,meandt_BW);
%plot(1:k,meandt_WB);
plot(1:k,meandP_H);