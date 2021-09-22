# -*- coding: utf-8 -*-
"""
Created on Wed Sep 22 13:29:27 2021

@author: a1846236
"""
#import os
import opensim as osim


def makemodel(OSModel,outputmodel)

%% 

# % Construct a model and add it to the tool. models
# % accept a base model and allow you to easily modify the model by appending
# % ModelOperators. Operations are performed in the order that they are
# % appended to the model.
# % Create the base Model by passing in the model file.

modelP = osim.ModelProcessor(OSModel)

# % Part 1b: Add frames to the model that will represent our IMU locations. 
# % The function addIMUFrame() adds a PhysicalOffsetFrame to a body at a 
# % specified location and orientation. Each frame is added at the path:
# %
# % /bodyset/<body_name>/<body_name>_imu_offset
model = modelP.process();


addIMUFrame(model, 'torso',   Vec3(0.08, 0.3, 0), Vec3(0, 0.5*pi, 0.5*pi));
addIMUFrame(model, 'femur_r', Vec3(0, -0.2, 0.05), Vec3(0, 0, 0.5*pi));
addIMUFrame(model, 'tibia_r', Vec3(0, -0.2, 0.05), Vec3(0, 0, 0.5*pi));

% Part 1c: Add IMU components to the model using the PhysicalOffsetFrames
% we just added to the model. We'll use the helper function addModelIMUs()
% included with OpenSenseUtilities.
imuFramePaths = StdVectorString();
imuFramePaths.add('/bodyset/torso/torso_imu_offset');
imuFramePaths.add('/bodyset/femur_r/femur_r_imu_offset');
imuFramePaths.add('/bodyset/tibia_r/tibia_r_imu_offset');
OpenSenseUtilities().addModelIMUs(model, imuFramePaths);
model.initSystem();

model.print(outputmodel);
fprintf('model created')
return 

def addIMUFrame(model, bodyName, translation, orientation)

body = model.updBodySet().get(bodyName);
name = [char(body.getName()) '_imu_offset'];
bodyOffset = PhysicalOffsetFrame(name, body, Transform());
bodyOffset.set_translation(translation);
bodyOffset.set_orientation(orientation);
body.addComponent(bodyOffset);
model.finalizeConnections();
