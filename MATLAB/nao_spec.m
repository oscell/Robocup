% Specification of nao robots
height = 0.58; % (m), height of nao6 robot
width = 0.31; % (m), width of nao6 robot
mass = 5.48; % (kg), weight of nao6 robot

DFOV = 68.2; % (deg) Display Field of View
HFOV = 57.2; % (deg) Horizontal Field of View
VFOV = 44.3; % (deg) Vertical Field of View

maxStepX = 0.080; % (m), maximum forward translation along X
defaultStepX = 0.040; % (m), default forward translation along X
minStepX = 0.010; % (m), minimum forward translation along X

maxStepTheta = 0.524; % (radian), absolute maximum rotation around Z (30 degrees)
defaultStepTheta = 0.349; % (radian), absolute default rotation around Z (20 degrees)
minStepTheta = 0.001; % (radian), absolute minimum rotation around Z  (0.0573 degrees)
maxStepTheta_deg = maxStepTheta*360/(2*pi); % (Deg), Convert radian to degree
defaultStepTheta_deg = defaultStepTheta*360/(2*pi); % (Deg), Convert radian to degree
minStepTheta_deg = minStepTheta*360/(2*pi); % (Deg), Convert radian to degree

maxStepPeriod = 0.6; % (s), maximum step duration
defaultStepPeriod = 0.5; % (s), default step duration
minStepPeriod = 0.42; % (s), minimum step duration

v_max = maxStepX / maxStepPeriod; % maximum velocity
v_default = defaultStepX / defaultStepPeriod; % normal velocity 
v_min = minStepX / minStepPeriod; % minimum velocity

w_max = maxStepTheta_deg / maxStepPeriod; % maximum angular velocity
w_default = (defaultStepTheta_deg / defaultStepPeriod)*(2*pi/(360)); % default angular velocity
w_min = minStepTheta_deg / minStepPeriod; % minimum angular velocity

t_fall = 0.55; % (s), time taken to fall down
t_getUp = 3.90; % (s), time taken to get up after the fall
t_fallTotal = t_fall + t_getUp; % (s), total time taken from falling down to getting back up


% References
% http://doc.aldebaran.com/2-5/naoqi/motion/control-walk.html
% https://www.youtube.com/watch?v=H8xc6LpiNVs
% https://www.researchgate.net/publication/272790526_On_the_maximum_walking_speed_of_NAO_humanoid_robots
% https://www.robopreneur.com/nao-v6
% https://www.generationrobots.com/en/403100-programmable-humanoid-robot-nao-v6.html