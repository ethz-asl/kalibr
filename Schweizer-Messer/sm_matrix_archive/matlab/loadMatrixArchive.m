function [ ama ] = loadMatrixArchive( filename )
%LOADMATRIXARCHIVE Loads an  matrix archive
%
% Input:
% Filename - the filename of the matrix archive.
%
% Output:
% ma - a struct with fields named according to the names loaded from the
%      matrix archive
%

startMagicMatrix = 'A';
startMagicString = 'S';
endMagic   = 'B';
nameFixedSize = 32;

[fid, message] = fopen(filename,'r','ieee-le');
if fid < 0
    error('unable to open file %s for reading: %s',filename, message);
end

try

    ama = struct();


    % Read the start magic character
    start = fread(fid,1,'uint8=>char');
    while ~feof(fid)
        if start ~= startMagicString && start ~= startMagicMatrix
            error('The start of a matrix block did not have the expected character. Wanted %s or %s, got %s', startMagicMatrix, startMagicString, start);
        end
      
        % Read the name
        name = strtrim(fread(fid,nameFixedSize,'uint8=>char')');
        if isempty(name)
            error('Failed to read the matrix name');
        end
        
        if(start == startMagicString)
            % Read the data size
            mxSize = fread(fid,1,'uint32');

            % Read the data
            M = fread(fid, [1, mxSize(1)],'uint8=>char');
        else
            % Read the data size
            mxSize = fread(fid,2,'uint32');

            % Read the data
            M = fread(fid,mxSize(1)*mxSize(2),'double');
            M = reshape(M,mxSize(1),mxSize(2));
        end
    
        % Read the end magic character
        endchar = fread(fid,1,'uint8=>char');
        if endchar ~= endMagic
            error('The end of a matrix block for matrix named %s did not have the expected character. Wanted %s, got %s', name, endMagic, endchar);
        end
    
        % Set the field on the return struct
        ama.(name) = M;
        
        % Reprime the loop (this will cause feof to evaluate to true if the
        % file has been finished)
        % Read the start magic character
        start = fread(fid,1,'uint8=>char');
        
    end

    fclose(fid);
catch ME
    fclose(fid);
    rethrow(ME);
end


end

