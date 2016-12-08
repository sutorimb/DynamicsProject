% BodyJVACLASS < TreeBodyCLASS & RigidBodyDynamicsCLASS     
%
% Defines a rigid body with all dynamic and kinematic properties. 
% It is embedded in a kinematic tree that is implemented as a set of linked
% objects to contain position and orientation data and compute this in a
% recursive fashion. 
% 
% Methods:
%  B = BodyJVACLASS(env, name)
%            Creates a body in the graphical environment 'env'. The body's
%            initial position, velocity, and acceleration are set to
%            standard values.  It has a name 'name'.
%  B.recursiveForwardKinematics(obj, B_r_IB, A_IB)
%            This function recieves the bodies position and orientation
%            from the parent joint, and passes it on to the child joints
%  B.recursiveGraphics(obj)
%            This function is called recursively to update the graphics.
%            After updating the graphical representation of this body, the
%            function calls the recursive graphics function of all child
%            joints.
% 
% Properties:
%  
%  Constraint Jacobians:
%     B_J_S;         % Translational Jacobian of the body
%     B_J_R;         % Rotational Jacobian of the body
%        
%   C. David Remy cdremy@umich.edu
%   Matlab R2012b
%   9/12/2013
%   v11
%
classdef BodyJVACLASS < TreeBodyCLASS & RigidBodyDynamicsCLASS 
   % Public properties
    properties
        % Kinematic values:
        % All properties are given in body-fixed coordinates B and
        % intialized to standard values (as they're not set in the
        % constructor):
        B_J_S         = [];      % Translational Jacobian of the body
        B_J_R         = [];      % Rotational Jacobian of the body
    end
    % Methods
    methods
        % Constructor creates the graphical objects
        function obj = BodyJVACLASS(env, name)
            % Call Constructor of the SuperCLASS
            obj = obj@TreeBodyCLASS(name);
            obj = obj@RigidBodyDynamicsCLASS(env);
            obj.bodyName = name;
        end
        function recursiveForwardKinematics(obj,  q, qDot, qDDot, B_r_IB, A_IB, B_omega_B, B_v_B, B_omegaDot_B, B_a_B,  B_J_S, B_J_R)
            % Position and orientation, as well as velocities and
            % accelerations are given by the parent joint and passed in its
            % call of 'recursiveForwardKinematics' 
            if obj.isRoot == false
                obj.A_IB          = A_IB;
                obj.B_omega_B     = B_omega_B;
                obj.B_omegaDot_B  = B_omegaDot_B;
                obj.B_r_IB        = B_r_IB;
                obj.B_v_B         = B_v_B;
                obj.B_a_B         = B_a_B;
                obj.B_J_S         = B_J_S;
                obj.B_J_R         = B_J_R;
            else
                obj.A_IB          = eye(3);
                obj.B_omega_B     = zeros(3,1);
                obj.B_omegaDot_B  = zeros(3,1);
                obj.B_r_IB        = zeros(3,1);
                obj.B_v_B         = zeros(3,1);
                obj.B_a_B         = zeros(3,1);
                obj.B_J_S         = zeros(3,length(q));
                obj.B_J_R         = zeros(3,length(q));
            end
            
            if obj.isLeaf == false
                % If this is no leaf, recursively call the child joints:
                for i = 1:obj.nChildren
                    obj.childJoints{i}.recursiveForwardKinematics( q, qDot, qDDot, obj.B_r_IB, obj.A_IB, obj.B_omega_B, obj.B_v_B, obj.B_omegaDot_B, obj.B_a_B,  obj.B_J_S, obj.B_J_R);
                end
            end
        end
        function recursiveGraphics(obj)
            obj.update();
            if obj.isLeaf == false
                % If this is no leaf, recursively call the child joints:
                for i = 1:obj.nChildren
                    obj.childJoints{i}.recursiveGraphics();
                end
            end
            if obj.isRoot
                drawnow();
            end
        end
    end
end
