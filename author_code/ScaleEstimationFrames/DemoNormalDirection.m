%% This code is used to compute the absoulte scale based on 
close all;
clear all;
clc;
addpath 'CameraGeometry'
global param;

%% matching parameters
param.nms_n                  = 5;   % non-max-suppression: min. distance between maxima (in pixels)
param.nms_tau                = 50;  % non-max-suppression: interest point peakiness threshold
param.match_binsize          = 100;  % matching bin width/height (affects efficiency only)
param.match_radius           = 200; % matching radius (du/dv in pixels)
param.match_disp_tolerance   = 1;   % du tolerance for stereo matches (in pixels)
param.outlier_disp_tolerance = 3;   % outlier removal: disparity tolerance (in pixels)
param.outlier_flow_tolerance = 10;   % outlier removal: flow tolerance (in pixels)
param.multi_stage            = 1;   % 0=disabled,1=multistage matching (denser and faster)
param.half_resolution        = 0;   % 0=disabled,1=match at half resolution, refine at full resolution
param.refinement             = 2;   % refinement (0=none,1=pixel,2=subpixel)

%% camera intrinsic parameters  
param.f  = 707.09;
param.u0 = 601.89;
param.v0 = 183.11;
param.h  = 1.65;
% Intrinsic Matrix
param.K = [param.f 0 param.u0;
    0 param.f  param.v0;
    0       0      1  ]; 

nprior = [0 1 0];                   % camera height

img_folder = './04/image_0/';
pose = load('./poses/04.txt');
numFrames = 10;

start_frame = 1;

for frame = start_frame:numFrames
    
if(mod(frame,100)==0)
frame
end  
% read images
I1 = imread([img_folder  num2str(frame-1,'%06d') '.png']);
I2 = imread([img_folder  num2str(frame,'%06d') '.png']);
[R1,t1,Tr]   = Absolute_to_RelativePose(pose,frame,frame+1);
 
imgheight = size(I1,1);

 
%% features points detection and matching 
% init matcher
matcherMex('init',param);
% push back images
matcherMex('push',I1);
tic
matcherMex('push',I2);
% disp(['Feature detection: ' num2str(toc) ' seconds']);
% match images
tic; matcherMex('match',0); matcherMex('bucketing',50,50,5);
p_matched = matcherMex('get_matches',0);
% disp(['Feature matching:  ' num2str(toc) ' seconds']);
% close matcher
matcherMex('close');
% show matching results
% disp(['Number of matched points: ' num2str(length(p_matched))]);

%% display the matching points
I = [I1;I2];
figure;
imshow(I);
hold on;
[img_height,img_width] = size(I1);
p_matched1 = p_matched + repmat([0;0;0;img_height],1,size(p_matched,2));
plot(p_matched1(1,:) ,p_matched1(2,:),'+', 'LineWidth',2,'MarkerSize',2,'Color', [1,0,0]);
plot(p_matched1(3,:) ,p_matched1(4,:),'+', 'LineWidth',2,'MarkerSize',2,'Color', [1,0,0]);

p_matched_full    = p_matched;
srcPoints_full    = p_matched_full(1:2,:)';
disPoints_full    = p_matched_full(3:4,:)';

%% relative pose estimaiton
[R,t,inlierindex]      =       Rt_solver(srcPoints_full,disPoints_full); 
p_matched =  p_matched_full(:,inlierindex);
%% homography matrix estimation in a predifend Region of Interest
[H, inliersindex] = H_solver(p_matched(1:2,:), p_matched(3:4,:));
I = [I1;I2];
figure;
imshow(I);
hold on;
[img_height,img_width] = size(I1);
p_matched1 = p_matched(:,inliersindex) + repmat([0;0;0;img_height],1,size(inliersindex,2));
plot(p_matched1(1,:) ,p_matched1(2,:),'+', 'LineWidth',2,'MarkerSize',2,'Color', [1,0,0]);
plot(p_matched1(3,:) ,p_matched1(4,:),'+', 'LineWidth',2,'MarkerSize',2,'Color', [1,0,0]);

%% solver the initial value for d(distance from  the camera center to ground plane) and n(normal direction)  
[d0, n] = initial_solver(R,t,H);  
end
