% EnvironmentCLASS < handle
%
% Defines a 'physical' environment in which all further components are
% embedded.  Since this is a purely numerical tool, all 'physical'
% definitions refer to vectors and coordinates given in the (inertial)
% coordinates of this graphics window.
% 
% Methods:
%  Env = EnvironmentCLASS() % Creates a new physical (graphical)
%                             environment
%  Env.toggleAxis()         % Toggles between showing and hiding the axis 
%                             of the environment
%  Env.resetOutput()        % Fits the axis to the size of the objects in
%                             it and resets all view parameter
%  Env.delete()             % Closes the environment and removes it from
%                             the memory 
%
%
%   C. David Remy cdremy@umich.edu
%   Matlab R2012b
%   9/12/2013
%   v11
%
classdef EnvironmentCLASS < handle
    % Private properties
    properties (SetAccess = private, GetAccess = private)
        % Handles to the graphical output
        fig
        ax
        light_handle
        % Indicates whether the graphical (inertial) axis are visible or
        % not.
        axVisible = 1;
    end
    
    methods 
        % Constructor
        function obj = EnvironmentCLASS()
            obj.fig = figure;
            set(obj.fig, 'Color', [0.95 0.95 0.95]);
            obj.ax  = axes;
            obj.light_handle = camlight('right');
            resetOutput(obj);
        end
        % Update output
        function resetOutput(obj)
            figure(obj.fig);
            if obj.axVisible
                axis(obj.ax, 'on');
            else
                axis(obj.ax, 'off');
            end
            axis(obj.ax, 'equal');
            view(obj.ax, [-37.5,30])
            axis(obj.ax, 'tight');
            a = axis(obj.ax);
            axis(obj.ax, a+[-1,1,-1,1,-1,1]);
            box(obj.ax, 'on')
            grid(obj.ax, 'on')
            camproj(obj.ax, 'perspective');
            camlight(obj.light_handle, 'right')
            view(obj.ax, [125, 25])
        end
        % Toggle the visibility of the graphical axis:
        function toggleAxis(obj)
            obj.axVisible = 1-obj.axVisible;
            resetOutput(obj);
        end
        % Close the figure upon deletion
        function delete(obj)
            close(obj.fig);
        end
    end
end