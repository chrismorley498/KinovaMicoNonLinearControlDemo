mico2=loadrobot('kinovaMicoM1N6S200');
addpath("./StepFiles");
%Below the unused parts of the robot are removed from the rigid body tree
temp=mico2;
for i=13:-1:9
    removeBody(temp,temp.Bodies{i}.Name);
end

%extact the properties of the robot within the forloop below for each link
numBodies=size(temp.Bodies,2);
for i=3:numBodies
    %Link properties
    mico2Mass{i-2}=temp.Bodies{i}.Mass;
    mico2COM{i-2}=temp.Bodies{i}.CenterOfMass;
    mico2Inertia{i-2}=temp.Bodies{i}.Inertia;
    %Joint properties
    mico2JointRotation{i-2}=temp.Bodies{i}.Joint.JointToParentTransform(1:3,1:3);
    mico2JointTranslation{i-2}=temp.Bodies{i}.Joint.JointToParentTransform(1:3,4);
end

%Remove the first two bodies so that body 'i' in the RBT is representative of link 1
for i=3:8
tempBodies{i-2}=temp.Bodies{i};
end

% Create the custom EE and insert it here
customEE=rigidBody('customEE');
customEEJoint=tempBodies{6}.Joint;
customEE.Joint=customEEJoint;
addVisual(customEE,"Mesh",'forceSensorEE.stl');%Change stl. file here for regular mico
customEE.Mass=.357;
customEE.Inertia=[0.001,0.001,0.001,-3.328e-6,-3.56e-6,-2.394e-5];
customEE.CenterOfMass=[0 0 -0.057];
tempBodies{6}=customEE;

mico2Mass{6}=customEE.Mass;
mico2COM{6}=[0 0 -0.057];
mico2Inertia{6}=customEE.Inertia;


%Create a new temporary RBT to assemble all the desirable links together
%Note that this tree has no base visual as of this point
temp2=rigidBodyTree;
addBody(temp2,tempBodies{1},'base')
for i=2:5
    addBody(temp2,tempBodies{i},tempBodies{i}.Parent.Name);      
end

%Add the custom EE to the RBT
addBody(temp2,customEE,tempBodies{5}.Name);


%Add EE frame
EE=rigidBody('EE');
EE.Mass=0;
EE.Inertia=zeros(1,6);
EEJoint=rigidBodyJoint('EEJoint','Fixed');
setFixedTransform(EEJoint,[1 0 0 0;0 -1 0 0;0 0 -1 -0.1207;0 0 0 1]);%Remove second argument here for regular mico without needle
EE.Joint=EEJoint;
addBody(temp2,EE,temp2.Bodies{6}.Name);
%Assign the RBT named 'mico2' the desired RBT
mico=temp2;
mico.Gravity=[0 0 -9.81];
% clear temp temp2 tempBodies numBodies i

for i=1:6
    mico.Bodies{i}.Joint.PositionLimits=[-2*pi,2*pi];
end

mico.DataFormat='Column';

Mico.Mass=mico2Mass;
Mico.COM=mico2COM;
Mico.Inertia=mico2Inertia;
Mico.JointTransforms.Rotation=mico2JointRotation;
Mico.JointTransforms.Translation=mico2JointTranslation;

clear temp temp2 tempBodies numBodies mico2Mass mico2JointTranslation mico2JointRotation mico2Inertia mico2COM mico2 ikblock_info_bus i EEJoint EE customEEJoint Custom EE ans customEE


