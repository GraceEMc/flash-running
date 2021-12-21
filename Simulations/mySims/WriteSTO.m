function WriteSTO(kinematicsFileName,osimModelFileName,outputFileName)
file=importdata(kinematicsFileName);
Data=file.data;
    time = Data(:,1);
    IKOpenSim = Data(:,2:end);
    %fname = 'test.sto';
    % Initialize 'motion file data matrix' for writing data of interest.
    nRows = size(IKOpenSim,1);
    nCols=size(IKOpenSim,2)+1; %add 1 for time
    motData = zeros(nRows, nCols);
    % Write time array to data matrix.
    motData(:, 1) = time;
    motData(:, 2:end) = IKOpenSim;          
    label = file.colheaders(2:end);
    
      import org.opensim.modeling.*;

    colcount=1; % Only want kinematic coloumns
    labelsfull={};
    kinematicModel = Model(osimModelFileName);
    for ii=1:nCols-1
        currAngle = label{ii};
      try
            %Get full path to coordinate
            fullPath = [char(kinematicModel.updCoordinateSet().get(currAngle).getAbsolutePathString()) '/value'];
            %Set angle name appropriately using full path
            labelsfull= [labelsfull fullPath];
            colcount=colcount+1;
      catch e
      end

    end
    
     nCols=colcount; 
    
    
    % Open file for writing.
    fid = fopen(outputFileName, 'wt');
    fprintf('\n------------------------------------------');
    fprintf('\n      Printing sto file     ');
    fprintf('\n------------------------------------------');
    if fid == -1
        error(['unable to open ', outputFileName])
    end
    % Write header.
    fprintf(fid, 'name %s\n', outputFileName(1:end-4));
     fprintf(fid, 'version=1 \n');
         fprintf(fid, 'nRows=%d\n', nRows);
    fprintf(fid, 'nColumns=%d\n', nCols);

     fprintf(fid, 'inDegrees=yes\n');
  %  fprintf(fid, 'range %.2f %.2f\n', time(1), time(nRows));
    fprintf(fid, 'endheader\n');
 
      % Write column labels.
    fprintf(fid, 'time');
    fprintf(fid,'\t');
    for i = 1:nCols-1
        fprintf(fid, '%s\t', labelsfull{i});
    end
  
        
    % Write data.
    for i = 1:nRows
        fprintf(fid, '\n'); 
        for j = 1:nCols
            if j == 1
                fprintf(fid,'%g\t', motData(i));
            else
                fprintf(fid, '%20.8f\t', motData(i, j));
            end
        end
    end
    fclose(fid);
end