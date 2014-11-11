%% Yosef Razin, Group 3
%  CS7649

%% Problem 2a
% Domain1

% Don't make video
CR_pf_a  = Potential_Nav([10 60],[210 75],[50 60 20; 150 75 35; ],1,0)

% make video
%CR_pf_a  = Potential_Nav([10 60],[210 75],[50 60 20; 150 75 35; ],1,1,'prob2a.avi');

%% Problem 2b
obs= [50 60 20; 150 75 50];
obs = [obs; 100 120 100; 140 20 80];

% Don't make video
CR_pf_b  = Potential_Nav([10 60],[210 75],obs,2,0);

% make video
%CR_pf_b  = Potential_Nav([10 60],[210 75],obs,2,1,'prob2b.avi');
 
%% Extra
% Random Field of 30 obstacles
x = 100 + 75.*randn(30,1);
y = 50 + 40.*randn(30,1);
r = 30 + 25.*randn(30,1);

r(r<0)=20;

obs= [x y r];

CR_pf_3  = Potential_Nav1([10 60],[210 75],obs,3,0);