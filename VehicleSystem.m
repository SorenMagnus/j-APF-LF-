classdef VehicleSystem < matlab.System
    % VehicleSystem - Simulink System object for simplified vehicle dynamics
    % Inputs: [steer, v_ref] (1x2)
    % Outputs: [x, y, theta, v]
    
    properties
        id = 1;
        radius = 0.6;
        L = 1.0; % wheelbase for kinematic turning
        max_steer = pi/4;
        max_acc = 2.0;
        max_speed = 6.0;
    end
    
    properties (Hidden)
        x
        y
        theta
        v
    end
    
    methods
        function obj = VehicleSystem(varargin)
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj, ~)
            % initialize states
            obj.x = 0;
            obj.y = 0;
            obj.theta = 0;
            obj.v = 0;
        end
        
        function [x,y,theta,v] = stepImpl(obj, u, dt)
            % u: vector [steer, v_ref]
            steer = min(max(u(1), -obj.max_steer), obj.max_steer);
            v_ref = min(max(u(2), 0), obj.max_speed);
            dv = v_ref - obj.v;
            acc = min(max(dv, -obj.max_acc), obj.max_acc);
            obj.v = obj.v + acc * dt;
            % kinematic update (unicycle-like with steering)
            obj.theta = obj.theta + (obj.v * tan(steer) / obj.L) * dt;
            obj.x = obj.x + obj.v * cos(obj.theta) * dt;
            obj.y = obj.y + obj.v * sin(obj.theta) * dt;
            x = obj.x; y = obj.y; theta = obj.theta; v = obj.v;
        end
        
        function resetImpl(obj)
            obj.x = 0; obj.y = 0; obj.theta = 0; obj.v = 0;
        end
    end
end