% classdef RotationalJointJVACLASS < JointJVACLASS
%
% Defines a rotational joint in a kinematic tree that is implemented as a
% set of linked objects.  
% 
% Methods:
%  joint = RotationalJointJVACLASS(name, predBody, sucBody) 
%            Creates a rotational joint object with name 'name' that links
%            the body specified in 'predBody' with the body specified in
%            'sucBody'. Both are objects of the type 'PlanarBodyCLASS',
%            and function as predecessor and successor in a kinematic tree.
%
% Properties:
%     - none -
%
%   C. David Remy cdremy@umich.edu
%   Matlab R2012b
%   9/12/2013
%   v11
%
classdef RotationalJointJVACLASS < JointJVACLASS
    
    % Public methods
    methods (Access = public)
        % The constructor only calls the constructor of the superclass.  We
        % have to do this by hand, since otherwise an empty constructor is
        % invoked.
        function obj = RotationalJointJVACLASS(env, name, predBody, sucBody)
            obj = obj@JointJVACLASS(env, name, predBody, sucBody);
        end
    end
    
    % Protected methods
    methods (Access = protected)
        % This method overwrites the 'displayType' method of the class
        % JointJVACLASS.  By doing so, we can change the behavior of this
        % specific joint 
        function s = getType(obj)
            % Use the code below to invoke displayType of the superclass
            % "JointCLASS"
            % s = displayType@JointCLASS(obj);
            s = 'Rotational Joint';
        end
        
        function [Dp_r_DpDs, A_DpDs] = JointFunction(obj, q)
            alpha = q(obj.q_index);
            Dp_r_DpDs = [0;0;0];
            A_DpDs    = [+cos(alpha), -sin(alpha), 0;
                         +sin(alpha), +cos(alpha), 0;
                         +0         , +0         , 1];
        end
        function [Dp_J_S, Dp_J_R] = JointJacobians(obj, q)
            Dp_J_S = zeros(3,length(q));
            Dp_J_R = zeros(3,length(q));
            Dp_J_S(:,obj.q_index) = [0;0;0];
            Dp_J_R(:,obj.q_index) = [0;0;1];
        end
        function [Dp_rDot_DpDs, Dp_omega_DpDs] = JointVelocity(obj, q, qDot)
            Dp_rDot_DpDs  = zeros(3,1);
            Dp_omega_DpDs = [0; 0; qDot(obj.q_index)];
        end
        function [Dp_rDDot_DpDs, Dp_omegaDot_DpDs] = JointAcceleration(obj, q, qDot, qDDot)
            Dp_rDDot_DpDs    = zeros(3,1);
            Dp_omegaDot_DpDs = [0; 0; qDDot(obj.q_index)];
        end
    end
end

