close
clear
clc

%% Environment Initialization:


scenario = robotScenario(UpdateRate=1000);

% Floor
addMesh(scenario,"Box",Position=[0.35 -0.15 -0.0251],Size=[1.3 1.3 0.05],Color=[0.7 0.5 0.5],Collision="mesh")
% Create a static surface structure representing a table by using scenario meshes fitted with collision objects. The surface structure acts as an obstacle in the scenario.
addMesh(scenario,"Box",Position=[0.75 0.05 0.1],Size=[0.5 0.9 0.05],Color=[0.7 0.7 0.5],Collision="mesh")
addMesh(scenario,"Box",Position=[0.35 -0.6 0.1],Size=[1.3 0.4 0.2],Color=[0.7 0.7 0.5],Collision="mesh")
addMesh(scenario,"Box",Position=[0.05 -0.6 0.25825],Size=[.2 0.2 0.1165],Color=[0.7 1 0.5],Collision="mesh")
% Static Hood
addMesh(scenario,"Box",Position=[.8 -0.25 0.3],Size=[0.4 0.03 0.35],Color=[0.7 0.7 0.7],Collision="mesh")
addMesh(scenario,"Box",Position=[.8 0.4 0.3],Size=[0.4 0.03 0.35],Color=[0.7 0.7 0.7],Collision="mesh")
addMesh(scenario,"Box",Position=[.85 0.075 0.49],Size=[0.3 0.71 0.03],Color=[0.7 1 0.7],Collision="mesh")
% Place two boxes on the table with collision-object-fitted scenario meshes. The two boxes are static meshes that act as obstacles and cannot attach to a robotPlatform object.
addMesh(scenario,"Box",Position=[0.9 -0.05 0.175],Size=[0.06 0.06 0.1],Color=[1 0.5 0.25],Collision="mesh")
addMesh(scenario,"Box",Position=[0.725 -0.1 0.175],Size=[0.06 0.06 0.1],Color=[1 0.5 0.25],Collision="mesh")
% Human
addMesh(scenario,"Box",Position=[0.075 -0.3 0.0875],Size=[0.12 0.06 0.175],Color=[.5 1 1],Collision="mesh")
addMesh(scenario,"Box",Position=[0.075 -0.3 0.2625],Size=[0.2 0.06 0.175],Color=[.5 1 1],Collision="mesh")
addMesh(scenario,"Box",Position=[0.075 -0.3 0.4],Size=[0.06 0.06 0.06],Color=[.5 1 1],Collision="mesh")

visuals = 'on';
collisions = 'on';
show3D(scenario,Visuals=visuals,Collisions=collisions);
title("Scene with collision object-based static meshes")
view(60,15)
light

%% FrankaEmika Panda Loaded in:

% Robot loaded in
%robot = importrobot('panda.urdf','urdf','DataFormat','column');
robot = loadrobot("frankaEmikaPanda",DataFormat="column");

    clearCollision(robot.Bodies{9})

config = [0  0  0 -1.5708  0   0.7500  0.6500  0.0400  0.0400]';
robo = robotPlatform('panda_hand',scenario,RigidBodyTree=robot,Collision='mesh');
move(robo,"joint",config')
rng default
weights = [0.3 0.3 0.3 1 1 1]; 

%% Add Box for Manipulation ( Want it to connect to the End Effector)

Payload = robotPlatform("Payload",scenario,Collision="mesh",InitialBasePosition=[0.775 0.1 0.175]);
updateMesh(Payload,"Cuboid",Collision="mesh",Size=[0.06 0.06 0.1],Color=[0.875 0.0 0.0]);

Box = rigidBody('BoxToMove');
addCollision(Box,"box",[0.06 0.06 0.1],[eye(3),[0 0 0.1]'; 0 0 0 1]);
addVisual(Box,"Box",[0.06 0.06 0.1],[eye(3),[0 0 0.1]'; 0 0 0 1]);
Payload_jnt = rigidBodyJoint('Payload_jnt','fixed');
setFixedTransform(Payload_jnt,[0 0 .1 0 ],'dh');
Box.Joint = Payload_jnt;



ax = show3D(scenario,Visuals=visuals,Collisions=collisions);
view(60, 15)
title("Scene with static meshes and robot platform")
light
%% Robot Parameters

planner = manipulatorRRT(robot,scenario.CollisionMeshes);
planner.MaxConnectionDistance = 0.35;
planner.ValidationDistance = 0.05;
planner.SkippedSelfCollisions = "parent";

% Number of interpolations between each adjacent configuration.
numState = 10;

%% Home to Pickup

Guess1 = [0.2371 -0.0200 0.0542 -2.2272 0.0013 2.2072 -0.9670 0.0400 0.0400]';
% Specify a pick-up joint configuration for the manipulator based on the target box location.
pickUpConfig = [0.775 0.1 0.275]';
pickupRot = [  1    0   0
               0    -1   0
               0    0   -1 ];
pickupT = [pickupRot pickUpConfig; 0 0 0 1];
ik = inverseKinematics('RigidBodyTree',robot);
[Goal1,SolInfo1] = ik('panda_hand',pickupT,weights,Guess1);
path1 = plan(planner,config',Goal1');
q1 = interpolate(planner,path1,numState);

% Visualize scene
show3D(scenario,FastUpdate=true,Parent=ax,Visuals=visuals,Collisions=collisions);
title("Approaching towards pick-up location")
xlabel('x')
ylabel('y')
zlabel('z')
setup(scenario)

tStart = tic;

for idx = 1:size(q1,1)
    % Get current joint configuration.
    jointConfig = q1(idx,:);

    % Move manipulator with current joint configuration.
    move(robo,"joint",jointConfig)

    % Use fast update to move platform visualization frames.
    show3D(scenario,FastUpdate=true,Parent=ax,Visuals=visuals,Collisions=collisions);

    % Get robot end effector pose. Draw trajectory.
    hold on
    poseNow = scenario.TransformTree.getTransform(robo.ReferenceFrame,robo.Name + "_" + 'panda_hand');
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),"b.",MarkerSize=12);
    drawnow
    hold off

    % Advance scenario simulation time.
    advance(scenario);
end

addBody(robot,Box,'panda_hand')
attach(robo,"Payload","panda_hand","ChildToParentTransform",trvec2tform([0.0 0.0 0.1]))
updateMesh(Payload,"Cuboid",Collision="mesh",Size=[0.06 0.06 0.1],Color=[0.875 0.0 0.0]);

show3D(scenario,FastUpdate=true,Parent=ax,Visuals=visuals,Collisions=collisions);
title("Picked-up target box")

%% Pickup to Drop off

Guess2 = [-0.6564 0.2885 -0.3187 -1.5941 0.1103 1.8678 -0.2344 0.04 0.04]';
% Specify a pick-up joint configuration for the manipulator based on the target box location.
dropOffConfig = [0.05 -0.6 0.3665]';
dropOffRot = [  1    0   0
               0    -1   0
               0    0   -1 ];
dropOffT = [dropOffRot dropOffConfig; 0 0 0 1];
ik = inverseKinematics('RigidBodyTree',robot);
[Goal2,SolInfo2] = ik('BoxToMove',dropOffT,weights,Guess2);
path2 = plan(planner,Goal1',Goal2');
q2 = interpolate(planner,path2,numState);

% Visualize scene
show3D(scenario,FastUpdate=true,Parent=ax,Visuals=visuals,Collisions=collisions);
title("Approaching towards drop location")

for idx = 1:size(q2,1)
    % Get current joint configuration.
    jointConfig2 = q2(idx,:);

    % Move manipulator with current joint configuration.
    move(robo,"joint",jointConfig2)

    % Use fast update to move platform visualization frames.
    show3D(scenario,FastUpdate=true,Parent=ax,Visuals=visuals,Collisions=collisions);

    % Get robot end effector pose. Draw trajectory.
    hold on
    poseNow = scenario.TransformTree.getTransform(robo.ReferenceFrame,robo.Name + "_" + 'panda_hand');
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),"r.",MarkerSize=12);
    drawnow
    hold off
    
    % Advance scenario simulation time.
    advance(scenario);
end

detach(robo)
removeBody(robot,'BoxToMove')

path3 = plan(planner, Goal2',config');

% Number of interpolations between each adjacent configuration.
numState = 10;
q3 = interpolate(planner,path3,numState);

% Visualize scene
show3D(scenario,FastUpdate=true,Parent=ax,Visuals=visuals,Collisions=collisions);
title("Approaching Home Position")

for idx = 1:size(q3,1)
    % Get current joint configuration.
    jointConfig3 = q3(idx,:);

    % Move manipulator with current joint configuration.
    move(robo,"joint",jointConfig3)

    % Use fast update to move platform visualization frames.
    show3D(scenario,FastUpdate=true,Parent=ax,Visuals=visuals,Collisions=collisions);

    % Get robot end effector pose. Draw trajectory.
    hold on
    poseNow = scenario.TransformTree.getTransform(robo.ReferenceFrame,robo.Name + "_" + 'panda_hand');
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),"r.",MarkerSize=12);
    drawnow
    hold off
    
    % Advance scenario simulation time.
    advance(scenario);
end

tEnd = toc(tStart);
q = [q1' q2' q3']';
T = 0:tEnd/(length(q)):tEnd;
Tmod = T(1:length(q));

figure (2)
plot(Tmod,q(:,1)',Tmod,q(:,2)',Tmod,q(:,3)',Tmod,q(:,4)',Tmod,q(:,5)',Tmod,q(:,6)',Tmod,q(:,7)',Tmod,q(:,8)',Tmod,q(:,9)')
title('Joint Angles')
xlabel('Time (seconds)')
ylabel('Angle (radians)')
legend('joint 1','joint 2','joint 3','joint 4','joint 5','joint 6','joint 7','joint 8','joint 9')
