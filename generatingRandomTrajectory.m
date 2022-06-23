%% The script generates a random trajectory for the Kinova Mico to follow.
% There is no significance to the generated trajectory, it serves only to demonstrate the trajectory tracking performance of the included control laws

%Below are some IC varaibles for the simscape model
%Create candle position configuration
micoCandlePosition=deg2rad([0,180,180,0,0,0]);
IC=micoCandlePosition;
interactiveHomePosition=deg2rad([0 90 177.5 90 0 0]');


for i =1:11
    %Generate joint space via points
    ViaPointsJointSpace(:,i)=generateRandomViaPointJointSpaceConfig;

end



% Create a trajectory to follow in task space

ViaPointsTaskSpace=[[0.45,0,0.5]',[0,0.45 0.5]',[-0.45,0,0.5]',[0,-0.45 0.5]',[0,0.0761,0.9376]',[0,0.0761,0.6376]',[0.9376,0.0761,0.1568]'];
ViaPointsTaskSpace=[ViaPointsTaskSpace;zeros(3,7)];
t=0:2:2*size(ViaPointsTaskSpace,2)-1;


function q =generateRandomViaPointJointSpaceConfig

q1=(rand-0.5)/0.5*2*pi;
q2=(rand-0.5)/0.5*pi/2+pi;
q3=(rand-0.5)/0.5*pi/2+pi;
q4=(rand-0.5)/0.5*pi;
q5=(rand-0.5)/0.5*pi;
q6=(rand-0.5)/0.5*pi;
q=[q1;q2;q3;q4;q5;q6];


end



