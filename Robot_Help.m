clc
clear
close

%% Environemnt Initialization:
% Create Scene with Collision-Object-Based Static Meshes
scenario = robotScenario(UpdateRate=10);
% Floor
addMesh(scenario,"Box",Position=[0.3 -0.15 -0.0251],Size=[1.3 1.3 0.05],Color=[0.7 0.5 0.5],Collision="mesh")
% Create a static surface structure representing a table by using scenario meshes fitted with collision objects. The surface structure acts as an obstacle in the scenario.
addMesh(scenario,"Box",Position=[0.7 0.05 0.1],Size=[0.5 0.9 0.05],Color=[0.7 0.7 0.5],Collision="mesh")
addMesh(scenario,"Box",Position=[0.3 -0.6 0.1],Size=[1.3 0.4 0.2],Color=[0.7 0.7 0.5],Collision="mesh")
addMesh(scenario,"Box",Position=[0.4 -0.5 0.25825],Size=[.2 0.2 0.1165],Color=[0.7 0.7 0.5],Collision="mesh")
% Static Hood
addMesh(scenario,"Box",Position=[0.7 -0.25 0.3],Size=[0.5 0.03 0.35],Color=[0.7 0.7 0.7],Collision="mesh")
addMesh(scenario,"Box",Position=[0.75 0.25 0.3],Size=[0.4 0.03 0.35],Color=[0.7 0.7 0.7],Collision="mesh")
addMesh(scenario,"Box",Position=[0.75 0 0.49],Size=[0.4 0.53 0.03],Color=[0.7 0.7 0.7],Collision="mesh")
% Place two boxes on the table with collision-object-fitted scenario meshes. The two boxes are static meshes that act as obstacles and cannot attach to a robotPlatform object.
addMesh(scenario,"Box",Position=[0.85 -0.05 0.175],Size=[0.06 0.06 0.1],Color=[1 0.5 0.25],Collision="mesh")
addMesh(scenario,"Box",Position=[0.7 -0.1 0.175],Size=[0.06 0.06 0.1],Color=[1 0.5 0.25],Collision="mesh")
% Human
addMesh(scenario,"Box",Position=[0.3 -0.3 0.0875],Size=[0.12 0.06 0.175],Color=[.5 1 1],Collision="mesh")
addMesh(scenario,"Box",Position=[0.3 -0.3 0.2625],Size=[0.2 0.06 0.175],Color=[.5 1 1],Collision="mesh")
addMesh(scenario,"Box",Position=[0.3 -0.3 0.4],Size=[0.06 0.06 0.06],Color=[.5 1 1],Collision="mesh")
% Visualize the visual meshes and collision meshes in the scenario by specifying the Visuals and Collisions name-value arguments.

%% Robot Placement
tpts = 0:4;
sampleRate = 20;
tvec = tpts(1):1/sampleRate:tpts(end);
numSamples = length(tvec);

robot = loadrobot("frankaEmikaPanda",DataFormat="column");
config = homeConfiguration(robot);
rng default

frankaSpaceWaypoints = [0.5 0.25 0.25; 0.75 0 0.35; 0.5 -0.25 0.75; 0.5 0.25 0.25]';
frankaTimepoints = linspace(tvec(1),tvec(end),4);
[pos,vel] = minjerkpolytraj(frankaSpaceWaypoints,frankaTimepoints,numSamples);

rng(0) % Seed the RNG so the inverse kinematics solution is consistent
ik = inverseKinematics(RigidBodyTree=robot);
ik.SolverParameters.AllowRandomRestart = false;
q = zeros(9,numSamples);
weights = [0.2 0.2 0.2 1 1 1]; % Prioritize position over orientation
initialGuess = [0, 0, 0, -pi/2, 0, 0, 0, 0.01, 0.01]'; % Choose an initial guess within the robot joint limits
for i = 1:size(pos,2)
    targetPose = trvec2tform(pos(:,i)')*eul2tform([0, 0, pi]);
    q(:,i) = ik("panda_hand",targetPose,weights,initialGuess);
    initialGuess = q(:,i); % Use the last result as the next initial guess
end

figure
set(gcf,"Visible","on")
show(robot);

rc = rateControl(sampleRate);
figure("Name","Pick and Place Using RRT",...
    "Units","normalized",...
    "OuterPosition",[0, 0, 1, 1],...
    "Visible","on");
show(robot,config,"Visuals","off","Collisions","on");
hold on
visuals = "on";
collisions = "on";
show3D(scenario,Visuals=visuals,Collisions=collisions);
title("Scene with collision object-based static meshes")
view(81,19)
light
hold on
for i = 1:numSamples
    show(robot, q(:,i),FastUpdate=true,PreservePlot=false);
    waitfor(rc);
end
view(60,15)
xlabel('x')
ylabel('y')
zlabel('z')
figure(3)
plot (tvec,vel)
figure(4)
plot(tvec,pos)