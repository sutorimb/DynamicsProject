% classdef PlanarJointCLASS < TreeJointCLASS
%
% Defines a joint in a kinematic tree that is implemented as a set of
% linked objects. 
% 
% Methods:
%  joint = PlanarJointCLASS(name, predBody, sucBody) 
%            Creates a joint object with name 'name' that links the body
%            specified in 'predBody' with the body specified in 'sucBody'.
%            Both are objects of the type 'PlanarBodyCLASS', and function
%            as predecessor and successor in a kinematic tree. 
%  recursiveForwardKinematics(B_r_IB, A_IB) 
%            Recursivley computes the position and position and orientation
%            variables of the successor body from the provided values of
%            the predecessor body.  Recursively calls the successor body to
%            all child joints to store this information and pass it on
%  recursiveGraphics() 
%            Draws a line from the predecessor to the successor, indicating
%            where the joint coordinates systems DP and DS are located. A
%            recursive call to the successor body keeps drawing the rest of
%            the tree
%
% Properties:
%
%   P_r_PDp % Position of the joint in the predecessor body
%   S_r_SDs % Position of the joint in the successor body
%   A_PDp   % Orientation of the joint w.r.t the predecessor body
%   A_SDs   % Orientation of the joint w.r.t the successor body
%   q;      % Vector of variables describing the current joint 
%             configuration
%
%   C. David Remy cdremy@umich.edu
%   Matlab R2012b
%   9/12/2013
%   v11
%
classdef PlanarJointCLASS < TreeJointCLASS
     
    % Public properties
    properties (SetAccess = public, GetAccess = public)
        P_r_PDp = [0;0];  % Position of the joint in the predecessor body
        S_r_SDs = [0;0];  % Position of the joint in the successor body
        A_PDp = eye(2);   % Orientation of the joint w.r.t the predecessor 
                          % body
        A_SDs = eye(2);   % Orientation of the joint w.r.t the successor 
                          % body
        q;                % Vector of variables describing this joint   
    end
     
    % Public methods
    methods (Access = public)
        % Constructor
        function obj = PlanarJointCLASS(name, predBody, sucBody)
            % Call Constructor of the SuperCLASS
            obj = obj@TreeJointCLASS(name, predBody, sucBody);
        end
        
        function recursiveForwardKinematics(obj, P_r_IP, A_IP)
            % Compute the translation and rotation created by the joint
            % (the outcome of this computation depends on the type of the
            % joint!)
            [Dp_r_DpDs, A_DpDs] = obj.JointFunction();
            
            % Use this to compute the displacement and orientation of the
            % successor body:
            
            % Compute the overall rotation first:
            A_PS = obj.A_PDp * A_DpDs * obj.A_SDs';
            A_SP = A_PS';
            A_IS = A_IP * A_PS;
            
            % Now the translation is computed just by adding up all 
            % individual translations in the S-frame:
            S_r_IS = A_SP*((P_r_IP + obj.P_r_PDp) + obj.A_PDp*Dp_r_DpDs) - obj.S_r_SDs;

            % Pass this information on to the successor body:
            obj.sucBody.recursiveForwardKinematics(S_r_IS, A_IS);
        end
         
        function recursiveGraphics(obj)
            % Draw a line from the predecessor to the successor body:
            % Position of the predecessor body:
            posP = obj.predBody.A_IB * obj.predBody.B_r_IB;  
            % Position of the joint on the predecessor body:
            posDP = obj.predBody.A_IB * (obj.predBody.B_r_IB + obj.P_r_PDp);  
            % Position of the successor body:
            posS = obj.sucBody.A_IB * obj.sucBody.B_r_IB;  
            % Position of the joint on the successor body:
            posDS = obj.sucBody.A_IB * (obj.sucBody.B_r_IB + obj.S_r_SDs);  
            % Draw a line between these four points:
            line([posP(1),posDP(1),posDS(1),posS(1)],[posP(2),posDP(2),posDS(2),posS(2)],'Marker','o');
            
            % Recursively call the successor body:
            obj.sucBody.recursiveGraphics();
         end
    end
    
    % Protected methods
    % These methods cannot be accessed from the outside of the function,
    % but from within any class that inherited from PlanarJointCLASS    
    methods (Access = protected)
        function [Dp_r_DpDs, A_DpDs] = JointFunction(obj)
            % Since this is a generic joint, it wouldn't make sense to
            % implement a specific type of joint (e.g., rotational) this
            % will happen in all the classes that inherit from
            % PlanarJointCLASS.  So the functionality in this SuperCLASS is
            % only a constant function.
            Dp_r_DpDs = [0;0];
            A_DpDs    = eye(2);
        end
    end
      
    % Private methods
    methods (Access = private)
        % none 
    end  
end

