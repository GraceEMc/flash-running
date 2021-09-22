runTorqueTracking = 1;
runMuscleTracking = 0;
runMusclePrediction = 0;
inputkin='subject01_Run_20002_cycle3_Kinematics_qInverse.sto';
LoadSetup='RunMadLoads_HamnerGRF.xml';

function torqueDrivenMarkerTracking(model,inputkin, LoadSetup)
% adapts from Aaorn code
    %Weights
    trackingWeight = 10;
    effortWeight = 0.1;
    grfWeight = 1;
    speedWeight = 1;
    symmetryWeight = 1;
    complexmodel=0;
    %Set optimal force and max torque
    optimalForce = 100;
    maxTorque = inf;

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

if complexModel==1
  
        reserveList = ['pelvis_tilt',
                        'hip_flexion_l', 'hip_flexion_r',
                        'knee_angle_l','knee_angle_r',
                        'ankle_angle_l', 'ankle_angle_r'];
else
    error ('Complex model not supported here. Need to add model coordinates...')
end

track.setModel(modelProcessor);

for tt=1:length(reserveList)
        addReserve(torqueModel, reserveList(tt), optimalForce, maxTorque)
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
    weightList = [0 0.1];
    weightName={'/jointset/ground_pelvis/pelvis_ty','/jointset/ground_pelvis/pelvis_ty'};
for w=1:length(weightList)
    stateWeights.cloneAndAppend(MocoWeight([weightName{w}  '/value'], weightList(w)))
        stateWeights.cloneAndAppend(MocoWeight([weightName{w}  '/speed'], weightList(w)))  
end

track.set_states_weight_set(stateWeights)
    %Set tracked states to be used in guess
    track.set_apply_tracked_states_to_guess(True)
    %Set unused state references to be allowed in case of file errors
    track.set_allow_unused_references(True)
    %Set position derivatives to be used as speeds
    track.set_track_reference_position_derivatives(True)
    
 %Set control weights
    track.set_control_effort_weight(effortWeight)
    
    %Set timing parameters
    track.set_initial_time(initialTime)
    track.set_final_time(finalTime)
    track.set_mesh_interval(meshInterval)
    
    %Customise the base tracking problem with relevant goals
    study = track.initialize();
    problem = study.updProblem();
    
    %Adjust time bounds to allow for subtle fluctations in finish time
    problem.setTimeBounds(initialTime, [finalTime - 0.1, finalTime + 0.1])
    
    %Update the control effort goal to a cost of transport type cost
    effort = MocoControlGoal().safeDownCast(problem.updGoal('control_effort'));
    effort.setDivideByDisplacement(True)
    
     %Set to not highly penalise the lumbar actuator
    if complexModel==1
        effort.setWeightForControl('/forceset/tau_lumbar_ext', 0.001)
    else
        effort.setWeightForControl('/forceset/lumbarAct', 0.001)
    end

    
    %Set an average speed goal based on sprinting data
    df_kinematics = TableProcessor(inputkin);
    startPos = df_kinematics['/jointset/ground_pelvis/pelvis_tx/value'].to_numpy()[0]
    endPos = df_kinematics['/jointset/ground_pelvis/pelvis_tx/value'].to_numpy()[-1]
    sprintSpeed = (endPos - startPos) / (finalTime - initialTime)
    speedGoal = osim.MocoAverageSpeedGoal('speed')
    speedGoal.set_desired_average_speed(sprintSpeed)
    speedGoal.setWeight(speedWeight)
    problem.addGoal(speedGoal)
    
% Generate a PDF report containing plots of the variables in the solution.
% For details, see osimMocoTrajectoryReport.m in Moco's
% Resources/Code/Matlab/Utilities folder.
model = modelProcessor.process();
report = osimMocoTrajectoryReport(model, ...
            'exampleMocoTrack_markertracking_solution.sto');
reportFilepath = report.generate();
open(reportFilepath);

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