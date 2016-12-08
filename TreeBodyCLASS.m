% classdef TreeBodyCLASS < handle
%
% Defines a body in a kinematic tree that is implemented as a set of linked
% objects.
% 
% Methods:
%  body = TreeBodyCLASS(name) % Creates a body-object with the given name
%  setParentJoint(parentJoint) Set the parents joint to the TreeJointCLASS
%                      object 'parentJoint'.
%  addChildJoint(childJoint) % Add a child joint which is given by the
%                      TreeJointCLASS object 'childJoint'.
%  inwardPass()      % Prints out the information about the body and
%                      recursively calls the parent joint
%  outwardPass(accumulatedData) % Recursively calls the child joints
%                      after appending information to the accumulated data.
%                      if this body is a leave in the tree, the entire
%                      information is displayed.
% 
% Public properties:
%     - none -
%
%   C. David Remy cdremy@umich.edu
%   Matlab R2012b
%   9/12/2013
%   v11
%
classdef TreeBodyCLASS < handle
     
    % Public properties
    properties (SetAccess = public, GetAccess = public)
            % none
    end
    
    % Protected properties
    properties (SetAccess = protected, GetAccess = protected)
        parentJoint;    % This joint connects the body with its predecessor
                        % in the kinematic tree (a JointCLASS object)
        childJoints;    % These joints connect the body with its successors 
                        % in the kinematic tree (a cell array of JointCLASS
                        % objects) 
        name;           % the name of the body.
        nChildren = 0;  % Number of child joints that connect with further 
                        % bodies in the tree
        isRoot = true;  % Is true if this body has no parent joint.
        isLeaf = true;  % Is true if this body has no child joints.
    end

    % Public methods
    methods (Access = public)
        % Constructor
        function obj = TreeBodyCLASS(name)
            % Save the name of the body for further usage:
            obj.name = name;
        end
        
        % Set the parents joint.  This function is called to link this body
        % with its predecessor in the kinematic tree via a parents joint.
        function setParentJoint(obj, parentJoint)
            obj.parentJoint = parentJoint;
            % If the body has a parent joint, it cannot be the root of the
            % tree:
            obj.isRoot = false;
        end
        
        % Add a child joint
        function addChildJoint(obj, childJoint)
            % Increase the number of child joints by one...
            obj.nChildren = obj.nChildren + 1;
            % ... and fill in the new place in the cell array
            obj.childJoints{obj.nChildren} = childJoint;
            % If the body has a child joint, it cannot be a leave of the
            % tree:
            obj.isLeaf = false;
        end
        
        % Make a recursive INWARD pass through the entire tree.  I.e.,
        % print the name of this body, and then call the inwardPass
        % function of it's parent joint (if the body is not the root of the
        % tree) 
        function s = inwardPass(obj)
            disp(['+',obj.name,' called.  String = empty']);
            % If this is the root of the tree...
            if obj.isRoot == true
                % ... we're done and can just return the name of this body
                s = [obj.name,'(root)'];
            else
                % ... otherwise we recursively call the inwardPass method 
                % of the parent joint in the kinematic tree to get the
                % first part of the string and then append the name of this
                % body.  
                s1 = inwardPass(obj.parentJoint);
                s = [s1,' - ',obj.name];
            end
            disp(['-',obj.name,' done.  String = ',s]);
        end
        
        % Make a recursive OUTWARD pass through the entire tree.  I.e., if
        % this body is a leave of the tree, write out the full list of
        % joints and bodies that has been accumulated in the
        % accumulatedData.
        % If not, call the outwardPass function of EACH child joint, with a
        % new accumulated data.
        function outwardPass(obj, accumulatedData)
            disp(['+',obj.name,' called.  String = ',accumulatedData]);
            if obj.isLeaf == true
                % If this is a leave of the tree, write out the accumulated
                % data:
                disp([accumulatedData, obj.name,'(leaf)'])
            else
                % If not, append the name of this body to the accumulated
                % data...
                newAccumulatedData = [accumulatedData, obj.name, ' - '];
                % ... and recursively call the outwardPass method of all
                % child joints.
                for i = 1:obj.nChildren
                    obj.childJoints{i}.outwardPass(newAccumulatedData);
                end
            end
            disp(['-',obj.name,' done.  String = ',accumulatedData]);
        end
   end
    
    % Protected methods
    methods (Access = protected)
        % none 
    end
    
    % Private methods
    methods (Access = private)
        % none 
    end
end

