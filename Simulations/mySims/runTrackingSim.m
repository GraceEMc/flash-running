runTorqueTracking = 1;
runMuscleTracking = 0;
runMusclePrediction = 0;
spd=num2str(4);
inputkin=['subject01_Run_' spd '0002_cycle3_Kinematics_q.sto'];
LoadSetup=['RunMadLoads_HamnerGRF_' spd 'ms.xml'];
model='gaitModel2DwIMU.osim';

%seem to have to use more complex model to convert
%WriteSTO(['subject01_Run_' spd '0002_cycle3_Kinematics_q.mot'],'Run_MaDwcontact.osim',inputkin)

torqueDrivenMarkerTracking(model,inputkin, LoadSetup, spd)

%%need to rerun with fixed grf
%%try modifying pelvis
function torqueDrivenMarkerTracking(model,inputkin, LoadSetup, speed)
% adapts from Aaorn code
%Weights
trackingWeight = 10;
effortWeight = 0.1;
grfWeight = 1;
speedWeight = 1;
symmetryWeight = 1;
%Set optimal force and max torque
optimalForce = 100;
maxTorque = inf;
complexModel=0;
import org.opensim.modeling.*;
% Construct a ModelProcessor and add it to the tool. ModelProcessors
% accept a base model and allow you to easily modify the model by appending
% ModelOperators. Operations are performed in the order that they are
% appended to the model.
% Create the base Model by passing in the model file.
modelProcessor = ModelProcessor(model);

% Remove all the muscles in the model's ForceSet.
modelProcessor.append(ModOpRemoveMuscles());

%Process model for additional parameters
torqueModel = modelProcessor.process();
torqueModel.initSystem();

% Add CoordinateActuators to the model degrees-of-freedom. This
% ignores the pelvis coordinates which already have residual
% CoordinateActuators.
modelProcessor.append(ModOpAddReserves(250));

if complexModel==0
    
    reserveList = {'pelvis_tilt',...
        'hip_flexion_l', 'hip_flexion_r',...
        'knee_angle_l','knee_angle_r',...
        'ankle_angle_l', 'ankle_angle_r'};
else
    error ('Complex model not supported here. Need to add model coordinates...')
end


for tt=1:length(reserveList)
    addReserve(torqueModel, reserveList{tt}, optimalForce, maxTorque)
end

%Finalise connections
torqueModel.finalizeConnections()
%Get the track model as a processor object
torqueModelProcessor = ModelProcessor(torqueModel);

%%Set-up tracking problem

%Process current model
trackModel = torqueModelProcessor.process();
trackModel.initSystem();

%Construct the tracking object and set basic parameters
track = MocoTrack();
track.setName('RunTracking_torqueDriven');
track.setModel(torqueModelProcessor);

%Set kinematic data and parameters
tableProcessor = TableProcessor(inputkin);
% tableProcessor.append(osim.TabOpLowPassFilter(12)) %%% data already filtered via RRA
tableProcessor.append(TabOpUseAbsoluteStateNames())
track.setStatesReference(tableProcessor)
track.set_states_global_tracking_weight(trackingWeight)
%Set some coordinates not to be tracked to avoid poor motion
%Similar process done in Dembia et al. Moco paper
stateWeights = MocoWeightSet();
weightList = [0 0 0.1];

%need to remove tx on treadmill?
weightName={'/jointset/ground_pelvis/pelvis_tx','/jointset/ground_pelvis/pelvis_ty','/jointset/ground_pelvis/pelvis_tilt'};
for w=1:length(weightList)
    stateWeights.cloneAndAppend(MocoWeight([weightName{w}  '/value'], weightList(w)))
    stateWeights.cloneAndAppend(MocoWeight([weightName{w}  '/speed'], weightList(w)))
end

track.set_states_weight_set(stateWeights)
%Set tracked states to be used in guess
track.set_apply_tracked_states_to_guess(true)
%Set unused state references to be allowed in case of file errors
track.set_allow_unused_references(true)
%Set position derivatives to be used as speeds
track.set_track_reference_position_derivatives(true)

%Set control weights
track.set_control_effort_weight(effortWeight)

% kinematics = TableProcessor(inputkin);
kintable=TimeSeriesTable(inputkin);
%time is an independent coloumn
initialTime=kintable.getIndependentColumn().get(0);
finalTime=kintable.getIndependentColumn().get(kintable.getNumRows() - 1);
%Set timing parameters
track.set_initial_time(initialTime)
track.set_final_time(finalTime)
duration = finalTime - initialTime;
meshNo = 100 ;%higher mesh here for muscle driven
meshInterval = duration / meshNo;

track.set_mesh_interval(meshInterval)



%Customise the base tracking problem with relevant goals
study = track.initialize();
problem = study.updProblem();

%Adjust time bounds to allow for subtle fluctations in finish time
problem.setTimeBounds(initialTime, [finalTime - 0.1, finalTime + 0.1])

%Update the control effort goal to a cost of transport type cost
effort = MocoControlGoal().safeDownCast(problem.updGoal('control_effort'));
effort.setDivideByDisplacement(true)

%Set to not highly penalise the lumbar actuator
if complexModel==1
    effort.setWeightForControl('/forceset/tau_lumbar_ext', 0.001)
else
    effort.setWeightForControl('/forceset/lumbarAct', 0.001)
end


%Set an average speed goal based on sprinting data

%%this wont work as treadmill running?
kinematics = TableProcessor(inputkin);
kintable=TimeSeriesTable(inputkin);
pelvist= kintable.getDependentColumn('/jointset/ground_pelvis/pelvis_tx/value').getAsMat();
startPos = pelvist(1);
endPos =  pelvist(end);
sprintSpeed = (endPos - startPos) / (finalTime - initialTime);
speedGoal = MocoAverageSpeedGoal('speed');
speedGoal.set_desired_average_speed(str2double(speed));
speedGoal.setWeight(speedWeight);
problem.addGoal(speedGoal);

%% symetry
% Define the periodicity goal
periodicityGoal = MocoPeriodicityGoal('symmetryGoal');



% All states are periodic except pelvis anterior-posterior translation
for i = 1:trackModel.getNumStateVariables()
    currentStateName = string(trackModel.getStateVariableNames().getitem(i-1));
    if (~contains(currentStateName,'pelvis_tx/value'))
        periodicityGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName));
    end
end

% All controls are periodic
for i = 1:trackModel.getNumControls()
    currentControlName = string(problem.createRep().createControlInfoNames().get(i-1));
    periodicityGoal.addControlPair(MocoPeriodicityGoalPair(currentControlName));
end
periodicityGoal.setWeight(symmetryWeight)

problem.addGoal(periodicityGoal);
%% Add contact tracking goal
if complexModel==1
    forceNamesRightFoot = {'forceset/contactHeel_r',...
        'forceset/contactMH1_r',...
        'forceset/contactMH3_r',...
        'forceset/contactMH5_r',...
        'forceset/contactHallux_r',...
        'forceset/contactOtherToes_r'};
    forceNamesLeftFoot = {'forceset/contactHeel_l',...
        'forceset/contactMH1_l',...
        'forceset/contactMH3_l',...
        'forceset/contactMH5_l',...
        'forceset/contactHallux_l',...
        'forceset/contactOtherToes_l'};
else
    forceNamesRightFoot = {'forceset/contactHeel_r',       'forceset/contactMidfoot_r',        'forceset/contactToe_r'};
    forceNamesLeftFoot = {'forceset/contactHeel_l',       'forceset/contactMidfoot_l',       'forceset/contactToe_l'};
end

%Create contact tracking goal
contactGoal = MocoContactTrackingGoal('contactGoal', grfWeight);
%Set external loads
contactGoal.setExternalLoadsFile(LoadSetup)
%Set force name groups
forceNames_r = StdVectorString();
forceNames_l = StdVectorString();
for ff =1:length(forceNamesRightFoot)
    forceNames_r.add(forceNamesRightFoot{ff})
    forceNames_l.add(forceNamesLeftFoot{ff})
end
%Create and add tracking groups
trackRightGRF = MocoContactTrackingGoalGroup(forceNames_r, 'RightGRF');
trackRightGRF.append_alternative_frame_paths('/bodyset/toes_r');
contactGoal.addContactGroup(trackRightGRF)
trackLeftGRF = MocoContactTrackingGoalGroup(forceNames_l, 'LeftGRF');
trackLeftGRF.append_alternative_frame_paths('/bodyset/toes_l');
contactGoal.addContactGroup(trackLeftGRF)
%Set parameters
contactGoal.setProjection('plane')
contactGoal.setProjectionVector(Vec3(0, 0, 1))
%Add contact tracking goal
problem.addGoal(contactGoal)
%%
%Add state bounds
%This also sets initial bounds on lumbar and pelvis coordinates that reflect
%'normal' running motions - based off the experimental data
problem.setStateInfo('/jointset/back/lumbar_extension/value', [deg2rad(-30),0],                        [deg2rad(-10),0])
problem.setStateInfo('/jointset/ground_pelvis/pelvis_tilt/value', [-10*pi/180, 0*pi/180], [deg2rad(-5),0])
problem.setStateInfo('/jointset/ground_pelvis/pelvis_tx/value', [0, 3])
problem.setStateInfo('/jointset/ground_pelvis/pelvis_ty/value', [0.8, 1.25])
problem.setStateInfo('/jointset/hip_l/hip_flexion_l/value', [-25*pi/180, 75*pi/180])
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', [-25*pi/180, 75*pi/180])
if complexModel==1
    problem.setStateInfo('/jointset/walker_knee_l/knee_angle_l/value', [0, 140*pi/180])
    problem.setStateInfo('/jointset/walker_knee_r/knee_angle_r/value', [0, 140*pi/180])
else
    problem.setStateInfo('/jointset/knee_l/knee_angle_l/value', [-140*pi/180, 0])
    problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', [-140*pi/180, 0])
end
problem.setStateInfo('/jointset/ankle_l/ankle_angle_l/value', [-20*pi/180, 30*pi/180])
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', [-20*pi/180, 30*pi/180])

%Configure the solver
solver = MocoCasADiSolver.safeDownCast(study.updSolver());
solver.resetProblem(problem)
solver.set_optim_constraint_tolerance(1e-2)
solver.set_optim_convergence_tolerance(1e-2)
study.print(['tracktest.omoco']);
%Solve
solution = study.solve();
%%
solname=['Run_torqueDriven_' speed 'ms'];
solnfile=[solname '_solution.sto'];
solution.write(solnfile)

report = osimMocoTrajectoryReport(trackModel, ...
        solnfile, 'bilateral', true);
% The report is saved to the working directory.
reportFilepath = report.generate();
open(reportFilepath);

predictSolution = MocoTrajectory(solnfile); %havent' got controls in the result)
prevStatesTable= predictSolution.exportToStatesTable(); % Extract previous solution's states
prevControlsTable = predictSolution.exportToControlsTable(); % Extract previous solution's controls
%accereraton output
outputPaths = StdVectorString();
outputPaths.add('.*accelerometer_signal');

accelerometerSignals = opensimSimulation.analyzeVec3(trackModel, ...
    prevStatesTable, ...
    prevControlsTable, ...
    outputPaths);
imuFramePaths = StdVectorString();
imuFramePaths.add('/bodyset/torso/torso_imu_offset');
imuFramePaths.add('/bodyset/femur_r/femur_r_imu_offset');
imuFramePaths.add('/bodyset/tibia_r/tibia_r_imu_offset');
accelerometerSignals.setColumnLabels(imuFramePaths);
sto = STOFileAdapterVec3();
sto.write(accelerometerSignals, [solname '_accel.sto']);

trackModel.print('Mymodel.osim');

%write grf
externalLoads = opensimMoco.createExternalLoadsTableForGait(trackModel, solution,  forceNames_r,     forceNames_l);
STOFileAdapter.write(externalLoads,[solname '_trackedGRF_torqueDriven_2D.mot'])


end

function addReserve(model, coordName, optForce, max_control)

import org.opensim.modeling.*;

coordSet = model.updCoordinateSet();

actu = CoordinateActuator();

actu.setName(['tau_' coordName]);
actu.setCoordinate(coordSet.get(coordName));
actu.setOptimalForce(optForce);
actu.setMinControl(-max_control);
actu.setMaxControl(max_control);

% Add to ForceSet
model.addForce(actu);

end