function ME543_Project_TEMPLATE_v11_V4()
    % This function runs the multi body simulation of a three segment
    % robot.  
    %
    % The robot's trajectory is defined as a linear motion (with a
    % trapezoidal velocity profile) in end-effector space.  It contains
    % desired Cartesian positions x_des, velocities d_x_des, and
    % accelerations dd_x_des. 
    %
    % An inverse kinematics routine is used and tested that computes
    % generalized joint angles q_des, generalized velocities d_q_des, and
    % generalized accelerations dd_q_des.  A forward kinematic animation is
    % performed to show that the joint-angle trajectories are correct.
    %
    % The robot is using a feed-back controller to follow these pre-defined
    % trajectories in joint-space.  Three different controllers are
    % implemented:   
    % (1) Using only PD-feedback on joint-level. 
    % (2) Using PD-feedback on joint-level and gravity compensation. 
    % (3) Using PD-feedback on joint-level and the method of computed
    %     torques.      
    %
    % Finally, the robot's motion is interrupted since the robot gets
    % entangled in a wire. This is detected as a unilateral inequality
    % constraint.  A collision is processed and the simulation is continued
    % with the constrained being active.
    
    %% Define all parameters:
    %
    % Timing:
    t_end   = 2.5;  % [s]
    t_rest  = 0.25; % [s]
    t_acc   = 0.5;  % [s]
    delta_t = 0.01; % [s]
    t_show  = 0.05; % [s]
    % Constraints:
    anchorPoint = [-1; -1; -1]; % [m]
    wireLength  = 2;            % [m]
    % Controller parameters:
    P = 10; % [Nm/rad] 
    D = 50; % [Nms/rad] 
    % Baumgarte Stabilization:
    P_Baumgarte = 100; % [1/s^2]
    D_Baumgarte = 20;  % [1/s]
    % Gravity:
    grav = 9.81; % [N/Kg]
        
    %% Prepare graphical output:
    %
    close all
    % Create a 'physical' (graphical) environment:
    Env = EnvironmentCLASS();
    f1 = gcf();
    set(f1, 'name', '3D Graphics Window');
    hold on;
    % Prepare figures to show end-effector trajectories:
    f2 = figure();
    set(f2, 'name', 'Forward Kinematics');
    hold on; grid on; box on
    axis([0,t_end,-3,3]);
    f3 = figure();
    set(f3, 'name', 'Forward Dynamics');
    hold on; grid on; box on
    axis([0,t_end,-2.5,2.5]);
    f4 = figure();
    set(f4, 'name', 'Forward Dynamics with Collision and Constraints');
    hold on; grid on; box on
    axis([0,t_end,-2.5,2.5]);
    f5 = figure();
    set(f5, 'name', 'Constraint Violation');
    hold on; grid on; box on
    axis([0,t_end,1.5,2.5]);
    
    %% Define the multi body system that represents the robot:
    %
    % Define the bodies:
    % Ground:
    ground = BodyJVACLASS(Env, 'Ground');
    ground.m_B   = 0; % [Kg] (ground has no mass)
    ground.scale = 0.4;
    % Actual robot segments with dynamic properties:
    link1 = BodyJVACLASS(Env, 'Link 1');
    link1.m_B   = 3; % [Kg]
    link1.B_I_B = diag([0.2; 0.02; 0.2]); % [Kg m^2]
    link1.scale = 0.5;
    link2 = BodyJVACLASS(Env, 'Link 2');
    link2.m_B   = 3; % [Kg]
    link2.B_I_B = diag([0.2; 0.02; 0.2]); % [Kg m^2]
    link2.scale = 0.5;
    link3 = BodyJVACLASS(Env, 'Link 3');
    link3.m_B   = 3; % [Kg]
    link3.B_I_B = diag([0.2; 0.02; 0.2]); % [Kg m^2]
    link3.scale = 0.5;
    % The end effector doesn't add any dynamics nor degrees of freedom, but
    % by having it in the kinematic tree, we can easily compute the
    % position, velocity, and acceleration of the end-effector:  
    endEffector = BodyJVACLASS(Env, 'End Effector');
    endEffector.m_B   = 0; % [Kg] (end-effector has no mass)
    endEffector.scale = 0.4;
    %
    % Define the joints:
    % Rotational joints 1-3
    joint1 = RotationalJointJVACLASS(Env, 'Joint 1', ground, link1);
    joint1.P_r_PDp = [0;0;0];
    joint1.A_PDp   = eye(3);
    joint1.S_r_SDs = [0;+0.5;0];
    joint1.A_SDs   = eye(3);
    joint1.q_index = 1;
    joint1.scale   = 0.3;
    joint2 = RotationalJointJVACLASS(Env, 'Joint 2', link1, link2);
    joint2.P_r_PDp = [0;-0.5;0];
    joint2.A_PDp   = [0,0,+1;0,1,0;-1,0,0];
    joint2.S_r_SDs = [0;+0.5;0];
    joint2.A_SDs   = eye(3);
    joint2.q_index = 2;
    joint2.scale   = 0.3;
    joint3 = RotationalJointJVACLASS(Env, 'Joint 3', link2, link3);
    joint3.P_r_PDp = [0;-0.5;0];
    joint3.A_PDp   = [0,0,-1;0,1,0;+1,0,0];
    joint3.S_r_SDs = [0;+0.5;0];
    joint3.A_SDs   = eye(3);
    joint3.q_index = 3;
    joint3.scale   = 0.3;
    % This '0 degree of freedom' joint connects the end-effector to link3.
    % It is only used to compute the position of the end effector: 
    endEffectorConnection = JointJVACLASS(Env, 'End Effector Connection', link3, endEffector);
    endEffectorConnection.P_r_PDp = [0;-0.5;0];
    endEffectorConnection.A_PDp   = eye(3);
    endEffectorConnection.S_r_SDs = [0;0;0];
    endEffectorConnection.A_SDs   = eye(3);
    endEffectorConnection.scale   = 0.2;
    %
    % Total number of degrees of freedom:
    n_q = 3;
    %
    % Show the robot in the 'zero' configuration
    ground.recursiveForwardKinematics(zeros(n_q,1), zeros(n_q,1), zeros(n_q,1));
    ground.recursiveGraphics();
    Env.resetOutput
%     print(gcf,'-r600','-djpeg','Robot.jpg','-opengl');
    
    %% Trajectory generation (QUESTION 1)
    %
    % Compute and show the desired trajectory in end-effector space:
    [t, x_des, d_x_des, dd_x_des] = EndEffectorTrajectory();
%     save('x_dx_ddx.mat', 't', 'x_des', 'd_x_des', 'dd_x_des'); 
    figure(f2); % To compare with forward kinematics
    plot(t, [x_des; d_x_des; dd_x_des], ':');
    figure(f3); % To compare with forward dynamics
    plot(t, [x_des; d_x_des], ':');
    figure(f4); % To compare with forward dynamics w/ collisions and constraints
    plot(t, [x_des; d_x_des], ':');
    %
    % Compute the associated trajectory in generalized coordinate space via
    % numerical inverse kinematics:
    [q_des, d_q_des, dd_q_des] = InverseKinematics(t, x_des, d_x_des, dd_x_des);
    q_des3=q_des;
    d_q_des3=d_q_des;
    dd_q_des3=dd_q_des;
    %
    disp(['Maximum value for q_des: ',num2str(max(q_des,[],2)')]);
    disp(['Maximum value for d_q_des: ',num2str(max(d_q_des,[],2)')]);
    disp(['Maximum value for dd_q_des: ',num2str(max(dd_q_des,[],2)')]);
    % Store results in file 
    save('trajectoryV3.mat', 't', 'q_des3', 'd_q_des3', 'dd_q_des3'); 
    q_des=q_des3;
    d_q_des=d_q_des3;
    dd_q_des=dd_q_des3;
   
    % Alternatively, load results from file
%     res = load('trajectory.mat');
%     t        = res.t;
%     q_des    = res.q_des;
%     d_q_des  = res.d_q_des;
%     dd_q_des = res.dd_q_des;
    %
    % Create a kinematic animation and record the position of the
    % end-effector to check if the inverse kinematics routine is working
    % properly:  
    figure(f1);
    t_next = t(1);
    pos = zeros(3, length(t));
    vel = zeros(3, length(t));
    acc = zeros(3, length(t));
    % For recording only
    %frameNr = 0;
    for i = 1:length(t)
        % Compute forward kinematics using the recursive outward pass
        ground.recursiveForwardKinematics(q_des(:, i), d_q_des(:, i), dd_q_des(:, i));
        pos(:, i) = endEffector.A_IB *endEffector.B_r_IB;
        vel(:, i) = endEffector.A_IB *endEffector.B_v_B;
        acc(:, i) = endEffector.A_IB *endEffector.B_a_B;
        if t_next<t(i)
            % Draw the system using a recursive outward pass:
            ground.recursiveGraphics();
            % Draw the position of the end-effector
            plot3(pos(3,i), pos(1,i), pos(2,i),'b.');
            t_next = t_next + t_show;
            % Uncomment to save animation frames:
            %print(gcf,'-r600','-djpeg',['Frames/AnimationFrameA',num2str(frameNr,'%04d.jpg')],'-opengl');
            %frameNr = frameNr + 1;
        end
    end
    figure(f2)
    plot(t, [pos; vel; acc], '-');
    pos3=pos;
    vel3=vel;
    acc3=acc;
%     save('pos_vel_acc3.mat', 't', 'pos3', 'vel3', 'acc3'); 
    
%      print(gcf,'-r600','-djpeg','KinematicTrajectories.jpg','-opengl');

    %% Run forward dynamic simulation with the different controllers  (QUESTION 2)
  
    type = {'PD_only', 'GravityComp', 'CompTorques'};
    for j = 1:3
        y_0 = [q_des(:, 1); d_q_des(:, 1)];
        controllerType = type{j};
        [~, y] = ode45(@ODE, t, y_0);
        % Record the position of the end-effector:
        pos = zeros(3, length(t));
        vel = zeros(3, length(t));
        for i = 1:length(t)
            % Set the joint variables to the values from the trajectory:
            q    = y(i,1:n_q);
            d_q  = y(i,n_q+1:2*n_q);
            % Compute forward kinematics using the recursive outward pass
            ground.recursiveForwardKinematics(q, d_q, zeros(n_q,1));
            pos(:, i) = endEffector.A_IB *endEffector.B_r_IB;
            vel(:, i) = endEffector.A_IB *endEffector.B_v_B;
        end
        figure(f3)
        lineStyle = {'-.', '--', '-'};
        plot(t, [pos; vel], lineStyle{j});
        disp(['Controller ',controllerType,': Final values are: ',num2str(y(end,1:n_q))]);
    end
     print(gcf,'-r600','-djpeg','DynamicTrajectories.jpg','-opengl');
   
    
    %% Run a forward dynamic simulation while taking into account the 
   
    % constraint of the entangled wire  (QUESTION 3, 4, and 5)
    %
    % Run simulation with an event handler:
    y_0 = [q_des(:, 1); d_q_des(:, 1)];
    controllerType = 'CompTorques';
    [t_one, y_one] = ode45(@ODE, t, y_0,odeset('Events',@wireStretched));
    disp(['Wire strechted at t = ',num2str(t_one(end))]);

    %
    % Compute collision
    q_MINUS   = y_one(end,1:n_q)';
    d_q_MINUS = y_one(end,n_q+1:end)';
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% COMPLETE CODE HERE %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    % Calculate post-impact generalized velocities
    [J_lambda, sigma_bar_lambda] = GetConstraintTerms()
    [M, ~, ~] = GetODETerms();
    M_lambda=inv(J_lambda*inv(M)*J_lambda');
    collisions_A=eye(length(J_lambda))-inv(M)*J_lambda'*M_lambda*J_lambda;
    q_PLUS = y_one(end,1:n_q)';
    d_q_PLUS = collisions_A*d_q_MINUS;
    %
    disp(['q_PLUS: ',num2str(q_PLUS')]);
    disp(['d_q_PLUS: ',num2str(d_q_PLUS')]);
    y_PLUS = [q_PLUS; d_q_PLUS];
    %
    % Continue simulation
    [t_two, y_two] = ode45(@ODE_Constrained, [t_one(end),t(t>t_one(end))], y_PLUS, odeset('AbsTol',1e-6,'RelTol',1e-3));
    %
    % Combine the two parts:
    t_full = [t_one;t_two]';
    y_full = [y_one;y_two];
    %
    % Create animation and record the position of the end-effector:
    figure(f1);
    [x_, y_, z_] = sphere;
    [faces, vertices] = surf2patch(x_, y_, z_, z_);
    vertices = vertices*wireLength + repmat(anchorPoint',size(vertices,1),1);
    patch('faces', faces, 'vertices', vertices, 'FaceColor', [0;0;0],'EdgeColor', 'none','FaceAlpha',0.2);
    wire = line([anchorPoint(1), 0],[anchorPoint(2), 0],[anchorPoint(3), 0]);
    t_next = t(1);
    pos = zeros(3, length(t_full));
    vel = zeros(3, length(t_full));
    % For recording only
    %frameNr = 0;
    for i = 1:length(t_full)
        % Set the joint variables to the values from the trajectory:
        q    = y_full(i,1:n_q);
        d_q  = y_full(i,n_q+1:2*n_q);
        % Compute forward kinematics using the recursive outward pass
        % (since we just do this for graphics, we don't care about the
        % accelerations and velocities):
        ground.recursiveForwardKinematics(q, d_q, zeros(n_q,1));
        pos(:, i) = endEffector.A_IB *endEffector.B_r_IB;
        vel(:, i) = endEffector.A_IB *endEffector.B_v_B;
        if t_next<t_full(i)
            % Set velocities and accelerations to zero, so the output
            % doesn't get too cluttered:
            ground.recursiveForwardKinematics(q, zeros(n_q,1), zeros(n_q,1));
            % Draw the system using a recursive outward pass:
            ground.recursiveGraphics();
            % Draw the position of the end-effector
            plot3(pos(3,i), pos(1,i), pos(2,i),'r.');
            % Update 'wire'
            set(wire,'xData',[anchorPoint(1), pos(3,i)],'yData',[anchorPoint(2), pos(1,i)],'zData',[anchorPoint(3), pos(2,i)], 'color', [0;0;0]);
            t_next = t_next + t_show;
            % Uncomment to save animation frames:
            %print(gcf,'-r600','-djpeg',['Frames/AnimationFrameB',num2str(frameNr,'%04d.jpg')],'-opengl');
            %frameNr = frameNr + 1;
        end
    end
    figure(f4)
    plot(t_full, [pos; vel], '-');
    print(gcf,'-r600','-djpeg','DynamicTrajectoriesCollision.jpg','-opengl');
    
    figure(f5)
    plot(t_full, sqrt((pos(1,:)+1).^2 + (pos(2,:)+1).^2 + (pos(3,:)+1).^2)-wireLength,'r')
    % Zoom in, to show effect of Baumgarte Stabilization:
    axis([0,t_end,-0.0005,0.0005]);
    print(gcf,'-r600','-djpeg','ConstraintViolation.jpg','-opengl');
    disp(['Final Constraint Violation: ',num2str(sqrt((pos(1,end)+1).^2 + (pos(2,end)+1).^2 + (pos(3,end)+1).^2)-wireLength)]);
     
  
    
    %% Functions used in the integration.
    
   
    % This includes the right hand sides of the ODDEs for unconstrained and
    % constrained motion, event handling, and control  
    %
    % The right hand side of the ordinary differential equation for the
    % unconstrained system. 
    function d_y_ = ODE(t_, y_)
        % Extract the generalized coordinates q and velocities d_q.
        q_   = y_(1:n_q);
        d_q_ = y_(n_q+1:end);
        % Recursively compute the Jacobians, positions, velocities,
        % and bias-accelerations (by setting q_ddot to 0).  They are stored
        % in the individual BodyJVA-objects that are then access from the
        % subsequent routines.
        % Note:  Since we did set q_ddot to zero, B_a_B, and B_omegaDot_B
        % will be the BIAS ACCELERATIONS, not the full accelerations!!!!
        ground.recursiveForwardKinematics(q_, d_q_, zeros(n_q,1));
        % Compute the components of the equations of motion:
        [M, f, g] = GetODETerms();
        % Call the controller function to get the actuator torques:
        tau = controller(t_, q_, d_q_);
        % Solve for the accelerations:
        dd_q = M\(f + g + tau);
        % Prepare the output variable d_y:
        d_y_ = zeros(2*n_q,1);
        % Compute the derivative of the positions from the velocities.
        d_y_(1:n_q)     = d_q_;
        % The derivative of the velocities are the generalized
        % accelerations: 
        d_y_(n_q+1:end) = dd_q;
    end
    %%
    
    % The right hand side of the ordenary differential equation for the
    % constrained system. 
    function d_y_ = ODE_Constrained(t_, y_)
        % Extract the generalized coordinates q and velocities d_q.
        q_   = y_(1:n_q);
        d_q_ = y_(n_q+1:end);
        % Recursively compute the Jacobians, positions, velocities,
        % and bias-accelerations (by setting q_ddot to 0).  They are stored
        % in the individual BodyJVA-objects that are then access from the
        % subsequent routines.
        % Note:  Since we did set q_ddot to zero, B_a_B, and B_omegaDot_B
        % will be the BIAS ACCELERATIONS, not the full accelerations!!!!
        ground.recursiveForwardKinematics(q_, d_q_, zeros(n_q,1));
        % Compute the components of the equations of motion:
        [M, f, g] = GetODETerms();
        % Get the constraint Jacobian:
        [J_lambda, sigma_bar_lambda] = GetConstraintTerms();
        % Call the controller function to get the actuator torques:
        tau = controller(t_, q_, d_q_);
        % Set up the equation A*x = b for the constrained system:
        n_lambda = size(J_lambda,1);
        A = [M,       -J_lambda';
             J_lambda, zeros(n_lambda)];
        b = [f + g + tau;
             -sigma_bar_lambda];
        % Solve for the accelerations and constraint forces:
        x = A\b;
        % Extract accelerations:
        dd_q = x(1:n_q);
        % Prepare the output variable d_y:
        d_y_ = zeros(2*n_q,1);
        % Compute the derivative of the positions from the velocities.
        d_y_(1:n_q)     = d_q_;
        % The derivative of the velocities are the generalized
        % accelerations: 
        d_y_(n_q+1:end) = dd_q;
    end
  
    %
    % This event-handler detects when the wire in which the robot arm got
    % entangled is fully stretched:  (finished)
    function [value, isterminal, direction] = wireStretched(~, y_)

        % Compute constraint-violation:
        ground.recursiveForwardKinematics(y_(1:3), [0;0;0], [0;0;0]);
        pass_ = endEffector.A_IB *endEffector.B_r_IB;
        value      = norm(pass_-anchorPoint)-2;
        isterminal = 1;
        direction  = +1;
    end
 
    %
    % This is the implementation of a controller.  It returns motor torques
    % that act on the joints to make them follow the desired trajectory:
    % Three different controllers are implemented:  
    % (1) Using only PD-feedback on joint-level. 
    % (2) Using PD-feedback on joint-level and gravity compensation. 
    % (3) Using PD-feedback on joint-level and the method of computed
    %     torques.       
    function tau = controller(t_, q_, d_q_)
        q_des_    = interp1(t, q_des', t_)';
        d_q_des_  = interp1(t, d_q_des', t_)';
        dd_q_des_ = interp1(t, dd_q_des', t_)';
        [M, f, g] = GetODETerms();      
        switch controllerType 
            case 'PD_only'
                tau = P*(q_des_-q_) + D*(d_q_des_-d_q_);
            case 'GravityComp' 
%                 g=link1.B_J_S' * link1.A_IB' * [0; -grav * link1.m_B; 0] +link2.B_J_S' * link2.A_IB' * [0; -grav * link2.m_B; 0]+link3.B_J_S' * link3.A_IB' * [0; -grav * link3.m_B; 0];
                tau = P*(q_des_-q_) + D*(d_q_des_-d_q_) - g;
            case 'CompTorques'
%                 q_ddd=inv(M)*(f+g+)
                 add=M*dd_q_des_-f-g;
                tau = P*(q_des_-q_) + D*(d_q_des_-d_q_)+add;
            otherwise
                tau = zeros(n_q, 1);
        end
    end
    

    %% Functions to compute the components of the Equations of Motion
   
    % This function computes the Mass Matrix M, the vector of Coriolis and
    % Centrifugal Forces f, and the vector of Gravitational Forces g.
    function [M, f, g] = GetODETerms()
        % Create a list of all dynamically contributing bodies (i.e.,
        % exclude the ground and the end effector)
        bodies = {link1, link2, link3};
        M = zeros(n_q, n_q);
        f = zeros(n_q, 1);
        g = zeros(n_q, 1);
        % Compose the equations of motion.  We do this numerically in each
        % integration step.  (This could also be done analytically in
        % advance and then just evaluated with the current values for q and
        % q_do) 
        for i_ = 1:length(bodies) % Iterate over all DYNAMICALLY CONTRIBUTING bodies in the tree
            B = bodies{i_}; % for each body:
            M = M + B.B_J_S' * B.m_B   * B.B_J_S + ...
                    B.B_J_R' * B.B_I_B * B.B_J_R;  
            f = f - B.B_J_S' * B.m_B * B.B_a_B - ...
                    B.B_J_R' * (B.B_I_B * B.B_omegaDot_B + skew(B.B_omega_B) * B.B_I_B * B.B_omega_B);
            g = g + B.B_J_S' * B.A_IB' * [0; -grav * B.m_B; 0] + ...
                    B.B_J_R' * B.A_IB' * [0; 0; 0] ;
            % It's important that all F_A are expressed in B-coordinates.
            % They must hence be transformed from the inertial frame:
        end
        function M = skew(w)
            % Generates a skew-symmetric matrix given a vector w
            M(1,2) = -w(3);
            M(1,3) =  w(2);
            M(2,3) = -w(1);
            M(2,1) =  w(3);
            M(3,1) = -w(2);
            M(3,2) =  w(1);
        end
    end

    %
    % Constraint terms.  Since motion is constrained to lie on a sphere of
    % radius 'wireLength' around the 'anchorPoint', both the Jacobian and
    % the bias accelerations are 1-dimensional.  They are obtained by
    % projecting the velocity of the end-effector onto a unit vector in the
    % direction  'end-effector' -> 'anchorPoint'.
    % To obtain the acceleration, we have to additionally compute the
    % derivative of this unit vector
    function [J_lambda, sigma_bar_lambda] = GetConstraintTerms()
        %
        %%%%%%%%%%%%%%%%%%%%%
        %%% EXPLAIN THIS! %%%
        %%%%%%%%%%%%%%%%%%%%%
        %
        dir = endEffector.A_IB * endEffector.B_r_IB - anchorPoint;
        dir = dir/norm(dir);
        J_lambda         = -dir'*endEffector.A_IB * endEffector.B_J_S;
        sigma_bar_lambda = -dir'*endEffector.A_IB * endEffector.B_a_B - norm(cross(dir, endEffector.A_IB * endEffector.B_v_B))^2/wireLength;
        %
        % For Baumgarte-stabilization, add the corrective terms to
        % sigma_bar_lambda here. 
        %
        %%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% COMPLETE CODE HERE %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%
        %
%          D_d_C= 2*D_Baumgarte*J_lambda*inv(endEffector.A_IB*endEffector.B_J_S)*endEffector.A_IB*endEffector.B_v_B;
%         sigma_bar_lambda = sigma_bar_lambda +D_d_C;

        sigma_bar_lambda = sigma_bar_lambda +...
                                D_Baumgarte*-dir'*endEffector.A_IB*endEffector.B_v_B+...
                                P_Baumgarte*(wireLength-norm(endEffector.A_IB * endEffector.B_r_IB - anchorPoint));
                              
    end
 
    
%% Functions for trajectory generation:
    % Defines the end-effector trajectory in Cartesian I-coordinates.  The
    % trajectory is given in position, velocity, and acceleration.
    % Using a trapezoidal motion profile (i.e., constant accelerations),
    % the end-effector moves along a straight line from [-1; -2; +0.5] to
    % [+1; -2; +0.5] in the time from t=0 to t=t_end.
    % 0.1 [s] is spent accelerating and decelerating and the required
    % acceleration a is computed accordingly.
    function [t, x_des_, d_x_des_, dd_x_des_] = EndEffectorTrajectory()
        t = 0:delta_t:t_end;
        t = [t, t_rest-1e-6, t_rest+1e-6, t_rest+t_acc-1e-6, t_rest+t_acc+1e-6, t_end-t_rest-t_acc-1e-6, t_end-t_rest-t_acc+1e-6, t_end-t_rest-1e-6, t_end-t_rest+1e-6];
        t=sort(t);
        % Compute required acceleration:
        a = ((+1) - (-1))/((t_end-t_acc-2*t_rest)*t_acc);
        x_0 = [-1; -2; +0.5];
        dd_x_des_ = zeros(3,length(t));
        d_x_des_ = zeros(3,length(t));
        x_des_ = zeros(3,length(t));
        for i_ = 1:length(t)
            if t(i_)<t_rest
                dd_x_des_(:, i_) = [+0; +0; +0];
                d_x_des_(:, i_)  = [+0; +0; +0];
                x_des_(:, i_)    = x_0;
            elseif t(i_)<t_rest+t_acc
                dd_x_des_(:, i_) = [+a; +0; +0];
                d_x_des_(:, i_)  = [(t(i_)-t_rest)*a; 0;0];
                x_des_(:, i_)    = x_0 + [0.5*(t(i_)-t_rest)^2*a; 0; 0];
            elseif t(i_)<t_end-t_rest-t_acc
                dd_x_des_(:, i_) = [+0; +0; +0];
                d_x_des_(:, i_)  = [+t_acc*a; 0; 0];
                x_des_(:, i_)    = x_0 + [0.5*t_acc*t_acc*a + (t(i_)-t_rest-t_acc)*t_acc*a;
                                     0;
                                     0];
            elseif t(i_)<t_end-t_rest
                dd_x_des_(:, i_) = [-a; +0; +0];
                d_x_des_(:, i_)  = [(t_end-t_rest-t(i_))*a; 0; 0];
                x_des_(:, i_)    = x_0 + [(t_end-t_rest*2-t_acc)*t_acc*a - 0.5*(t_end-t_rest-t(i_))^2*a;
                                     0;
                                     0];
            else
                dd_x_des_(:, i_) = [+0; +0; +0];
                d_x_des_(:, i_)  = [+0; +0; +0];
                x_des_(:, i_)    = x_0 + [2;0;0];
            end
        end
    end
 
    % This function numerically solves the inverse kinematics problem.  It
    % defines a number of residual functions (in position, velocity, and
    % acceleration).  Each residual function computes a forward kinematic
    % solution for a given set of q, d_q, and dd_q.  The obtained
    % difference in Cartesian coordinates is converted into a residual.
    % Numerically, values for q, d_q, and dd_q, are identified that drive
    % the residual to 0.  The solution of an iteration is stored and used
    % as starting point in subsequent calls. 
    
   %% InverseKinematics (finished)
  
    function [q_des_, d_q_des_, dd_q_des_] = InverseKinematics(t, x_des_, d_x_des_, dd_x_des_)
        q_des_    = zeros(n_q, length(t));
        d_q_des_  = zeros(n_q, length(t));
        dd_q_des_ = zeros(n_q, length(t));
        % The initial guess for the implicit inverse kinematics.  This is
        % later continuously updated with the previous solution:
        X_INIT = [0; 0; 0]; 
        for i_ = 1:length(t)
            X = fsolve(@residual_X, X_INIT(:,1), optimset('Display','off'));
            q_des_(:, i_) = X;
            X_INIT(:,1) = X;

            ground.recursiveForwardKinematics(q_des_(:, i_), [0;0;0], [0;0;0]);
            d_q_des_(:, i_)  = inv(endEffector.A_IB*endEffector.B_J_S)*d_x_des_(:, i_) ;

            ground.recursiveForwardKinematics(q_des_(:, i_), d_q_des_(:, i_), [0;0;0]);
            dd_q_des_(:, i_) =inv(endEffector.A_IB*endEffector.B_J_S)*(dd_x_des_(:, i_)-endEffector.A_IB*endEffector.B_a_B);

        end
        function r = residual_X(q_test_)
            ground.recursiveForwardKinematics(q_test_, [0;0;0], [0;0;0]);
            r = x_des_(:, i_) - endEffector.A_IB *endEffector.B_r_IB;
        end
        
    end
   
end