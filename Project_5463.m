clc
clear
close

%% Initialize the Environment

WORLD = collisionBox(4,2,0.1); % XYZ Lengths
WORLD.Pose = trvec2tform([1 0 -.05]); % XYZ Position
Wall1 = collisionBox(0.5,0.25,0.65); % XYZ Lengths
Wall1.Pose = trvec2tform([0.5 0.5 0.325]); % XYZ Position
Wall2 = collisionBox(0.25,0.25,0.85); % XYZ Lengths
Wall2.Pose = trvec2tform([-0.5 0.5 0.425]); % XYZ Position
Wall3 = collisionBox(0.5,0.5,0.1); % XYZ Lengths
Wall3.Pose = trvec2tform([-0.075 0.5 0.225]); % XYZ Position
Wall4 = collisionBox(0.4,0.5,0.5); % XYZ Lengths
Wall4.Pose = trvec2tform([0.075 -0.5 0.25]); % XYZ Position
Post1 = collisionCylinder(0.05,0.225); % Radius, Length
Post1.Pose = trvec2tform([-0.075 0.5 0.1125]); % XYZ Position
env = {WORLD Wall1 Wall2 Wall3 Wall4 Post1};
sv.Environment = env;
hold on
%% Initialize the Robot

manipulator = loadrobot("universalUR5e", DataFormat="row", Gravity=[0 0 -9.81]);
showdetails(manipulator)
q = homeConfiguration(manipulator);


%% Initialize Human Entity


%% Display the Environment

figure("Name","Pick and Place Using RRT",...
    "Units","normalized",...
    "OuterPosition",[0, 0, 1, 1],...
    "Visible","on");
show(manipulator,q,"Visuals","off","Collisions","on");
hold on
for i = 1:length(env)
    show(env{i})
end
view(60,15)
xlabel('x')
ylabel('y')
zlabel('z')