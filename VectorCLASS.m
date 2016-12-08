% VectorCLASS < handle
%
% Defines a vector that is shown at the origin of its environment graphics
% axis 
% 
% Methods:
%  v = VectorCLASS(env, C, C_v) % 
%                          Creates a vector with the components 'C_v' (a
%                          numerical 3x1 column-vector) given in the
%                          coordinate system 'C' (of the type CoSysClass).
%  v.setCoords(C, C_v)         
%                          Sets the coordinates to new values defined in
%                          components 'C_v' of the coordinate system 'C'
%                          (of the type CoSysClass). 
%  C_v = v.getCoords(C)    Gets the components of this vector in
%                          coordinates of the CoSys 'C'
%  v.delete()              Removes the vector from the graphics output and
%                          the memory 
% 
% Properties:
%  name   % A string with the name of the vector
%  color  % A 3-vector of RGB values (between 0 and 1) defines the color of
%           this coordinate system in the graphical representation
%
%
%   C. David Remy cdremy@umich.edu
%   Matlab R2012b
%   9/12/2013
%   v11
%
classdef VectorCLASS < handle
    % Private properties
    properties (SetAccess = private, GetAccess = private)
        ENV_v;  % Vector coordinates in the environment-frame
        patchHandle;
        env;
        labelText;
    end
    % Public properties
    properties
        name = '';
        color = [0;0;0];
    end
    % methods
    methods
        function obj = VectorCLASS(env, C, C_v)
            obj.env = env;
            resetOutput(env);
            obj.ENV_v = C.A_IC*C_v;
            [f,v] = createGraphicsData(obj);
            obj.patchHandle = patch('faces', f, 'vertices', v, 'FaceColor', obj.color,'EdgeColor', 'none');
            % Add name label:
            % Transform into graphical CoSys, since Matlab uses a convention in which Z points up:
            % Z -> 2-axis
            % Y -> 1-axis
            % X -> 3-axis
            ROT = [0,0,1;1,0,0;0,1,0];
            pos = ROT*obj.ENV_v;
            pos = pos + pos/norm(pos)*0.15;
            obj.labelText = text(pos(1),pos(2),pos(3),obj.name);
        end
        function setCoords(obj, C, C_v)
            obj.ENV_v = C.A_IC*C_v;
            updateGraphics(obj);
        end
        function C_v = getCoords(obj, C)
            C_v = C.A_IC'*obj.ENV_v;
        end
        function set.color(obj, color)
            obj.color = color;
            updateGraphics(obj);
        end
        function set.name(obj, name)
            obj.name = name;
            updateGraphics(obj);
        end
        % Remove from graphics upon deletion
        function delete(obj)
            delete(obj.patchHandle);
            delete(obj.labelText);
        end
    end
    methods (Access = private)
        function updateGraphics(obj)
            [~,v] = createGraphicsData(obj);
            set(obj.patchHandle,'vertices',v);
            set(obj.patchHandle,'FaceColor',obj.color');
            % Add name label:
            % Transform into graphical CoSys, since Matlab uses a convention in which Z points up:
            % Z -> 2-axis
            % Y -> 1-axis
            % X -> 3-axis
            ROT = [0,0,1;1,0,0;0,1,0];
            pos = ROT*obj.ENV_v;
            pos = pos + pos/norm(pos)*0.15;
            set(obj.labelText,'position',pos,'String',obj.name,'Color', obj.color);
        end
        function [f,v] = createGraphicsData(obj)
            N = 4;%20;
            vec = obj.ENV_v;
            % Create a unit vector in z-direction:
            [x,y,z] = cylinder([0.02,0.02],N);
            % scale:
            abs_vec = norm(vec);
            z = z*abs_vec;
            [f,v] = surf2patch(x,y,z,z);
            [x,y,z] = cylinder([0.4,0],N);
            [f1,v1] = surf2patch(x,y,z,z);
            v1=v1*0.1;
            v1 = v1 + repmat([0,0,abs_vec],size(v1,1),1);
            [v,f]=addPatches(v,f,v1,f1);
            
            % figure out which transformation to apply to rotate this vector into
            % v:
            vec1 = vec/abs_vec;
            k = cross([0;0;1],vec1);
            if norm(k)~=0
                % Rodrigues's formula:
                costheta = dot([0;0;1],vec1);
                R =[ 0    -k(3)  k(2);
                    k(3)  0    -k(1);
                    -k(2)  k(1)  0];
                R = costheta*eye(3) + R + k*k'*(1-costheta)/sum(k.^2);
            else
                if vec1(3)>0
                    R = eye(3);
                else
                    R = -eye(3);
                end
            end
            % Transform into graphical CoSys, since Matlab uses a convention in which Z points up:
            % Z -> 2-axis
            % Y -> 1-axis
            % X -> 3-axis
            ROT = [0,0,1;1,0,0;0,1,0];
            v = transformVertices(v,ROT*R,[0;0;0]);
        end
    end
end
function [v_, f_] = addPatches(v1_,f1_,v2_,f2_)
    % function [v, f] = addPatches(v1,f1,v2,f2)
    %
    % Assuming that v1, f1, v2, and f2 define vertices and faces of two patch
    % objects, this function returns the vertices and faces of a new combined
    % patch object.
    f2_ = f2_ + repmat(size(v1_,1),size(f2_));
    v_ = [v1_; v2_];
    f_ = [f1_; f2_];
end
function vTrans_ = transformVertices(v_,dirCosine_,translation_)
    % function vTrans = transformVertices(v,dirCosine,translation)
    %
    % This function transforms the coordinates of the vertices given in 'v'.
    % 'dirCosine' is a rotation 3 x 3 matrix, 'translation' is a translational
    % 3-vector. Both are applied to every element in 'v'.
    % 'v' is a matrix containing vertices, as they are used in patch objects.
    % The reutrn value vTrans contains the coordinates of the transformed
    % vertices.
    if isempty(v_)
        vTrans_ = [];
        return
    end
    % rotation
    vTrans_ = (dirCosine_*v_')';
    % translation
    vTrans_ = vTrans_ + repmat(translation_',size(vTrans_,1),1);
end