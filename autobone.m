leftLeg = rigidBodyTree;

leftThighbone = rigidBody('Left thigh bone');
leftShinbone = rigidBody('Left shin bone');
leftFoot = rigidBody('Left foot');

leftThighbonejnt = rigidBodyJoint('lthighjnt', 'revolute');
leftShinbonejnt = rigidBodyJoint('lshinjnt', 'revolute');
leftFootjnt = rigidBodyJoint('lfootjnt', 'revolute');

leftThighbonejnt.JointAxis = [1, 0, 0];
leftShinbonejnt.JointAxis = [1, 0, 0];
leftFootjnt.JointAxis = [1, 0, 0];

thighBonelength = 4;
shinBonelength = 4;
footLength = 2.5;

%{
collThigh = collisionCylinder(1.5, thighBonelength);
collShin = collisionCylinder(1.0, shinBonelength);
collFoot = collisionBox(2, footLength, 1);

collThigh.Pose = trvec2tform([0, 0, thighBonelength/2]);
collShin.Pose = trvec2tform([0, 0, shinBonelength/2]);
collFoot.Pose = trvec2tform([0, 0, 1 / 2]);

addCollision(leftThighbone, collThigh);
addCollision(leftShinbone, collShin);
addCollision(leftFoot,collFoot);
%}

setFixedTransform(leftThighbonejnt, trvec2tform([0, 0, 0]));
setFixedTransform(leftShinbonejnt, trvec2tform([0, 0, -thighBonelength]));
setFixedTransform(leftFootjnt, trvec2tform([0, 0, -shinBonelength]));

%{
A = thighBonelength; %% Length of the common normal line between the two z-axes
alpha = 0; %% Angle of rotation for the common normal
d = 0; %% Offset along the z-axis in the normal direction, from parent to child
theta = 0; %% Angle of rotation for the x-axis along the previous z-axis

dhparams = [0, alpha, d, theta;
            A, alpha, d, theta;
            A, alpha, d, theta];

setFixedTransform(leftThighbonejnt, dhparams(1, :), 'dh');
setFixedTransform(leftShinbonejnt, dhparams(2, :), 'dh');
setFixedTransform(leftFootjnt, dhparams(3, :), 'dh');
%}


leftThighbone.Joint = leftThighbonejnt;
leftShinbone.Joint = leftShinbonejnt;
leftFoot.Joint = leftFootjnt;

addBody(leftLeg, leftThighbone,'base');
addBody(leftLeg, leftShinbone, 'Left thigh bone');
addBody(leftLeg, leftFoot, 'Left shin bone');


showdetails(leftLeg);

show(leftLeg);
%show(leftLeg, "Collisions","on", "Frames","off");
%drawnow;
%axis([-10, 10, -10, 10, -10, 10]);

figure("Name", "Interactive GUI");
gui = interactiveRigidBodyTree(leftLeg, MarkerScaleFactor=3);
%%eePose = [3, 3, 3];

%%eeBody = rigidBody('End Effector');
%%setFixedTransform(eeBody.Joint, trvec2tform(eePose));
%%addBody(leftLeg, eeBody, leftLeg.BaseName);

%{
gik = generalizedInverseKinematics;
gik.RigidBodyTree = leftLeg;
gik.ConstraintInputs = {'position'};

posTgt = constraintPositionTarget('Left foot');
posTgt.TargetPosition = [0, 1, 2];

%q0 = homeConfiguration(leftLeg);
q0 = randomConfiguration(leftLeg);
[q, solutionInfo] = gik(q0, posTgt);

%%show(leftLeg, q);
%%title(['Solver status: ', solutionInfo.Status])
%%axis([-6, 6, -6, 6, -6, 6]);
%}





