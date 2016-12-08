% classdef JointJVACLASS < TreeJointCLASS
%
% Defines a joint in a kinematic tree that is implemented as a set of
% linked objects. 
% 
% Methods:
%  joint = JointJVACLASS(name, predBody, sucBody) 
%            Creates a joint object with name 'name' that links the body
%            specified in 'predBody' with the body specified in 'sucBody'.
%            Both are objects of the type 'BodyCLASS', and function
%            as predecessor and successor in a kinematic tree. 
%  recursiveForwardKinematics(B_r_IB, A_IB) 
%            Recursivley computes the position and position and orientation
%            variables of the successor body from the provided values of
%            the predecessor body.  Recursively calls the successor body to
%            store this information and pass it on. 
%  recursiveGraphics() 
%            This function is called recursively to update the graphics.
%            After updating the graphical representation of this joint, the
%            function calls the recursive graphics function of it's
%            successor body 
%
% Properties:
%   P_r_PDp % Position of the joint in the predecessor body
%   S_r_SDs % Position of the joint in the successor body
%   A_PDp   % Orientation of the joint w.r.t the predecessor body
%   A_SDs   % Orientation of the joint w.r.t the successor body
%   q_index % Vector of indices that define where in the vector of
%             generalized coordinates, the variables describing this joint
%             are located.  
%   autoUpdate    
%           % If this property is set to 'true', the graphical output will
%             be updated every time another public  property changes.  If 
%             it is set to false, the user has to force the graphical
%             update, by calling the function 'B.updateGraphics()' 
%   scale   % This value can be used to scale the size of the coordinate
%             systems for Dp and Ds. 
%
%   C. David Remy cdremy@umich.edu
%   Matlab R2012b
%   9/12/2013
%   v11
%
classdef JointJVACLASS < TreeJointCLASS
     
    % Public properties
    properties (SetAccess = public, GetAccess = public)
        P_r_PDp = [0;0;0];% Position of the joint in the predecessor body
        S_r_SDs = [0;0;0];% Position of the joint in the successor body
        A_PDp = eye(3);   % Orientation of the joint w.r.t the predecessor 
                          % body
        A_SDs = eye(3);   % Orientation of the joint w.r.t the successor 
                          % body
        q_index;          % Vector of indices that define where in the 
                          % vector of generalized coordinates, the
                          % variables describing this joint are located. 
        autoUpdate = 'true'  % In the default configuration, the 
                          % graphical output is updated every time a
                          % variable changes.  This is convenient, but can
                          % really slow down everything.
        scale = 1;        % To draw the coordinate systems of the joint smaller     
end
    
    % Private properties
    properties (SetAccess = private, GetAccess = private)
        % Objects for graphical visualization:
        env;       % Graphical environment
        Dp;        % A bound coordinate system, attached to the joint at 
                   % the predecessor
        Ds;        % A bound coordinate system, attached to the joint at
                   % the sucessor
    end
    
    % Public methods
    methods 
        % Constructor
        function obj = JointJVACLASS(env, name, predBody, sucBody)
            % Call Constructor of the SuperCLASS
            obj = obj@TreeJointCLASS(name, predBody, sucBody);
            obj.env = env;
            obj.Dp        = BoundCoSysCLASS(env, predBody.A_IB*obj.A_PDp,  predBody.A_IB*(predBody.B_r_IB + obj.P_r_PDp));
            obj.Dp.color  = [1 0 0];
            obj.Dp.name   = [obj.name,' DP'];
            obj.Dp.scale  = obj.scale;
            obj.Ds        = BoundCoSysCLASS(env, sucBody.A_IB*obj.A_SDs, sucBody.A_IB*(sucBody.B_r_IB + obj.S_r_SDs));
            obj.Ds.color  = [0 1 0];
            obj.Ds.name   = [obj.name,' DS'];
            obj.Ds.scale  = obj.scale;
            
        end
        function set.P_r_PDp(obj, P_r_PDp)
            obj.P_r_PDp = P_r_PDp;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.S_r_SDs(obj, S_r_SDs)
            obj.S_r_SDs = S_r_SDs;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.A_PDp(obj, A_PDp)
            obj.A_PDp = A_PDp;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.A_SDs(obj, A_SDs)
            obj.A_SDs = A_SDs;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.q_index(obj, q_index)
            obj.q_index = q_index;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.scale(obj, scale)
            obj.scale = scale;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        % Remove everything upon deletion
        function delete(obj)
            delete(obj.Dp);
            delete(obj.Ds);
        end
        function recursiveForwardKinematics(obj, q, qDot, qDDot, P_r_IP, A_IP, P_omega_P, P_v_P, P_omega_dot_P, P_a_P, P_J_S, P_J_R)
            % Rotation and displacement about the joint:
            [Dp_r_DpDs, A_DpDs] = obj.JointFunction(q);
             % Angular rate and translational velocity accross the joint:
            [Dp_rDot_DpDs, Dp_omega_DpDs] = obj.JointVelocity(q, qDot);
            % Angular andtranslational acceleration accross the joint:
            [Dp_rDDot_DpDs, Dp_omegaDot_DpDs] = obj.JointAcceleration(q, qDot, qDDot);
            % Compute the jacobians that define the velocity accross the
            % joint as a linear function of q_dot: 
            [Dp_J_S, Dp_J_R]    = obj.JointJacobians(q);
            
            % Compute the position, velocity, and acceleration of each
            % successing coordinate system:
            A_IDp           = A_IP * obj.A_PDp;
            Dp_r_IDp        = obj.A_PDp' * (P_r_IP + obj.P_r_PDp);
            Dp_omega_Dp     = obj.A_PDp' * (P_omega_P + 0);
            Dp_v_Dp         = obj.A_PDp' * (P_v_P + 0 + obj.skew(P_omega_P)*obj.P_r_PDp);
            Dp_omegaDot_Dp  = obj.A_PDp' * (P_omega_dot_P + 0 + 0);
            Dp_a_Dp         = obj.A_PDp' * (P_a_P + 0 + 0 + (obj.skew(P_omega_dot_P) + obj.skew(P_omega_P)^2)*obj.P_r_PDp);
            
            A_IDs           = A_IDp * A_DpDs;
            Ds_r_IDs        = A_DpDs' * (Dp_r_IDp + Dp_r_DpDs);
            Ds_omega_Ds     = A_DpDs' * (Dp_omega_Dp + Dp_omega_DpDs);
            Ds_v_Ds         = A_DpDs' * (Dp_v_Dp + Dp_rDot_DpDs + obj.skew(Dp_omega_Dp)*Dp_r_DpDs);
            Ds_omegaDot_Ds  = A_DpDs' * (Dp_omegaDot_Dp + Dp_omegaDot_DpDs + obj.skew(Dp_omega_Dp)*Dp_omega_DpDs);
            Ds_a_Ds         = A_DpDs' * (Dp_a_Dp + Dp_rDDot_DpDs + 2*obj.skew(Dp_omega_Dp)*Dp_rDot_DpDs + (obj.skew(Dp_omegaDot_Dp) + obj.skew(Dp_omega_Dp)^2)*Dp_r_DpDs);
            
            A_IS            = A_IDs * obj.A_SDs';
            S_r_IS          = obj.A_SDs * Ds_r_IDs - obj.S_r_SDs;
            S_omega_S       = obj.A_SDs * (Ds_omega_Ds + 0);
            S_v_S           = obj.A_SDs * (Ds_v_Ds + 0) - obj.skew(S_omega_S)*obj.S_r_SDs;
            S_omegaDot_S    = obj.A_SDs * (Ds_omegaDot_Ds + 0 + 0);
            S_a_S           = obj.A_SDs * (Ds_a_Ds + 0 + 0) - (obj.skew(S_omegaDot_S) + obj.skew(S_omega_S)^2)*obj.S_r_SDs;
            
            % Compute the displacement and orientation of the successor body:
            % Compute the overall rotation first:
            A_PS = obj.A_PDp * A_DpDs * obj.A_SDs';
            A_SP = A_PS';
            % Compute the rotational Jacobian of the successor body:
            S_J_R = A_SP*(P_J_R + obj.A_PDp*Dp_J_R);
            % Compute the translational Jacobian of the successor body:
            S_J_S = A_SP*(P_J_S + obj.A_PDp*Dp_J_S + obj.skew(obj.P_r_PDp + obj.A_PDp*Dp_r_DpDs)'*P_J_R)-obj.skew(obj.S_r_SDs)'*S_J_R;

            % Pass this information on to the successor body:
            obj.sucBody.recursiveForwardKinematics(q, qDot, qDDot, S_r_IS, A_IS, S_omega_S, S_v_S, S_omegaDot_S, S_a_S, S_J_S, S_J_R);
        end
        
        function recursiveGraphics(obj)
            % Update Graphics:
            obj.update();
            % Recursively call the successor body:
            obj.sucBody.recursiveGraphics();
         end
    end
    
	% Protected methods
    % These methods cannot be accessed from the outside of the function,
    % but from within any class that inherited from JointJVACLASS    
    methods (Access = protected)
        function [Dp_r_DpDs, A_DpDs] = JointFunction(obj, q)
            % Since this is a generic joint, it wouldn't make sense to
            % implement a specific type of joint (e.g., rotational). This
            % will happen in all the classes that inherit from
            % JointJVACLASS.  So the functionality in this SuperCLASS is
            % only a constant function.
            Dp_r_DpDs = [0;0;0];
            A_DpDs    = eye(3);
        end
        function [Dp_J_S, Dp_J_R] = JointJacobians(obj, q)
            % Since this is a generic joint, it wouldn't make sense to
            % implement a specific type of joint (e.g., rotational). This
            % will happen in all the classes that inherit from
            % JointJVACLASS.  So the functionality in this SuperCLASS is
            % only a constant function.
            Dp_J_S = zeros(3,length(q));
            Dp_J_R = zeros(3,length(q));
        end
        function [Dp_rDot_DpDs, Dp_omega_DpDs] = JointVelocity(obj, q, qDot)
            % Since this is a generic joint, it wouldn't make sense to
            % implement a specific type of joint (e.g., rotational). This
            % will happen in all the classes that inherit from
            % JointJVACLASS.  So the functionality in this SuperCLASS is
            % only a constant function.
            Dp_rDot_DpDs  = zeros(3,1);
            Dp_omega_DpDs = zeros(3,1);
        end
        function [Dp_rDDot_DpDs, Dp_omegaDot_DpDs] = JointAcceleration(obj, q, qDot, qDDot)
            % Since this is a generic joint, it wouldn't make sense to
            % implement a specific type of joint (e.g., rotational). This
            % will happen in all the classes that inherit from
            % JointJVACLASS.  So the functionality in this SuperCLASS is
            % only a constant function.
            Dp_rDDot_DpDs    = zeros(3,1);
            Dp_omegaDot_DpDs = zeros(3,1);
        end
        function M = skew(obj, w)
           % Generates a skew-symmetric matrix given a vector w
           M(1,2) = -w(3);
           M(1,3) =  w(2);
           M(2,3) = -w(1);
           M(2,1) =  w(3);
           M(3,1) = -w(2);
           M(3,2) =  w(1);
       end
    end
      
   % Private methods
    methods (Access = private)
         % Update the graphic objects, if a value has changed
        function update(obj)
            obj.Dp.A_IC   = obj.predBody.A_IB*obj.A_PDp;
            obj.Dp.I_r_IC = obj.predBody.A_IB*(obj.predBody.B_r_IB + obj.P_r_PDp);
            obj.Dp.scale  = obj.scale;
            obj.Ds.A_IC   = obj.sucBody.A_IB*obj.A_SDs;
            obj.Ds.I_r_IC = obj.sucBody.A_IB*(obj.sucBody.B_r_IB + obj.S_r_SDs);
            obj.Ds.scale  = obj.scale;
        end
    end  
end

