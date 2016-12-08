% classdef TreeJointCLASS < handle
%
% Defines a joint in a kinematic tree that is implemented as a set of
% linked objects. 
% 
% Methods:
%  joint = TreeJointCLASS(name, predBody, sucBody) % Creates a joint object
%                      with name 'name' that links the body specified in
%                      'predBody' with the body specified in 'sucBody'.
%                      Both are objects of the type 'TreeBodyCLASS', and
%                      function as predecessor and successor in a kinematic
%                      tree.
%  inwardPass()      % Prints out the information about the joint and
%                      recursively calls the preceeding body
%  outwardPass(accumulatedData) % Recursively calls the succeeding body
%                      after appending information to the accumulated data
% Public properties:
%     - none -
%
%   C. David Remy cdremy@umich.edu
%   Matlab R2012b
%   9/12/2013
%   v11
%
classdef TreeJointCLASS < handle
     
    % Public properties
    properties (SetAccess = public, GetAccess = public)
        % none
    end
    
    % Protected properties
    properties (SetAccess = protected, GetAccess = protected)
        predBody;  % This is the predecessor of the joint in the kinematic
                   % tree
        sucBody;   % This is the successor of the joint in the kinematic
                   % tree
        name;      % The name of the joint.
    end
    
    % Public methods
    methods (Access = public)
        % Constructor
        function obj = TreeJointCLASS(name, predBody, sucBody)
            % Store the information about the predecessor and successor of
            % this joint:
            obj.predBody = predBody;
            obj.sucBody = sucBody;
            % Save the name
            obj.name = name;
            % Now link this body outward and inward, by registering it
            % with the parent and child bodies:
            sucBody.setParentJoint(obj);
            predBody.addChildJoint(obj);
        end
        
        % Make a recursive INWARD pass through the entire tree.  I.e.,
        % print the name of this joint, and then call the inwardPass
        % function of it's predecessor body 
        function s = inwardPass(obj)
            disp(['+',obj.name,' called.  String = empty']);
            % Recursively call the inwardPass method of the predecessor
            % body in the kinematic tree.
            s1 = inwardPass(obj.predBody);
            s = [s1,' - ',obj.name, '(',obj.getType(),')'];
            disp(['-',obj.name,' done.  String = ',s]);
        end
        
        % Make a recursive OUTWARD pass through the entire tree.  I.e.,
        % update the accumulated data by appending the name of this joint.
        % Then call the outwardPass function of the successor body.
        function outwardPass(obj, accumulatedData)
            disp(['+',obj.name,' called.  String = ',accumulatedData]);
            % Append the name of this joint to the accumulated data:
            newAccumulatedData = [accumulatedData, obj.name, '(',obj.getType(),')', ' - '];
            % Recursively call the outwardPass method of the successor body
            % in the kinematic tree.
            obj.sucBody.outwardPass(newAccumulatedData);
            disp(['-',obj.name,' done.  String = ',newAccumulatedData]);
        end
    end
    
    % Protected methods
    % These methods cannot be accessed from the outside of the function,
    % but from within any class that inherited from JointCLASS    
    methods (Access = protected)
        % This function displays which type of joint we're dealing with. It
        % should change appropriately in an inhereting subclass
        function s = getType(obj)
            s = 'Generic Joint';
        end
    end
      
    % Private methods
    methods (Access = private)
        % none 
    end  
end

