%% Kinova Mico Simscape Initialize Script
%Create Variant Objects
JointSpace=Simulink.Variant('Space==1');
TaskSpace=Simulink.Variant('Space==2');

addpath("./StepFiles");
micoRigidBodyTreeCreation;
micoCandlePosition=deg2rad([0,180,180,0,0,0]);

%% Execute This Block of Code To Configure Model For Joint Space
Space=1;

%% Execute This Block of Code To Configure Model For Task Space
Space=2;

%% Execute This Block of Code to Generate Random Joint Space Trajectory
for i =1:11
    ViaPointsJointSpace(:,i)=generateRandomViaPointJointSpaceConfig;
end
t=0:2:2*size(ViaPointsJointSpace,2)-1;

%% Execute This Block of Code to Generate Task Space Trajectory
ViaPointsTaskSpace=[[0.35,0,0.35]',[0,0.35 0.35]',[-0.35,0,0.35]',[0,-0.35 0.35]',[0,0.0761,0.9376]',[0,0.0761,0.6376]',[0.9376,0.0761,0.1568]'];
% ViaPointsTaskSpace=[ViaPointsTaskSpace;[[0;pi;0],[0;pi;0],[0;pi;0],[0;pi;0],[0;0;0],[0;0;0],[0;pi;0]]];
ViaPointsTaskSpace=[ViaPointsTaskSpace;[[pi;0;pi],[pi;0;pi],[pi;0;pi],[pi;0;pi],[0;0;0],[0;0;0],[pi;0;pi]]];

t=0:2:2*size(ViaPointsTaskSpace,2)-1;

%Create IK solver object to provide kinova IC
ik=inverseKinematics('RigidBodyTree',mico,'SolverAlgorithm','LevenbergMarquardt');
tformIC=[[roty(pi),[0.35,0,0.45]'];[0 0 0  1]];
[IC,~]=ik('EE',tformIC,[0.9 0.9 0.9 1 1 1],mico.homeConfiguration);

%% Function for generating Random Trajectories
function q =generateRandomViaPointJointSpaceConfig

q1=(rand-0.5)/0.5*2*pi;
q2=(rand-0.5)/0.5*pi/2+pi;
q3=(rand-0.5)/0.5*pi/2+pi;
q4=(rand-0.5)/0.5*pi;
q5=(rand-0.5)/0.5*pi;
q6=(rand-0.5)/0.5*pi;
q=[q1;q2;q3;q4;q5;q6];

end


