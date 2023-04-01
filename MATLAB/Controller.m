classdef Controller
    properties
        
        %Gain
        N
        
        %heading of moving obj and target
        phi_m
        phi_t 

        %position of moving obj and target
        x0_m
        x0_t
        
        %Velocity of object and target
        V_m
        V_t


        x_m
        x_t
        
        phi_mdot
      
    end
    methods
        function obj = Controller(target_orentation,obj_orientation,gain,target_pos,obj_pos)
            obj.N = gain;

            obj.phi_m = obj_orientation;
            obj.phi_t = target_orentation;

            %position of moving obj
            obj.x0_m = [0;0];
            obj.x0_t = [9;0];
            
            %Initial conditions
            obj.V_m = 0.3;
            obj.V_t = 0.5;
            obj.x_m = x0_m;
            obj.x_t = x0_t;
            
            obj.phi_mdot = 0;

        end
        function obj = update(obj.idx)
            termination = false;

            R = sqrt((obj.x_t(1,1) - obj.x_m(1,1))^2 + (obj.x_t(2,1) - obj.x_m(2,1))^2);
            Rd = -1;
            
            while termination == false
                t = t + dt;
                if R < 1000
                    if Rd > 0
                        termination = true;
                    end
                elseif t > 530
                    termination = true;
                end
           
            R = sqrt((obj.x_t(1,1) - obj.x_t(1,1))^2 + (obj.x_t(2,1) - obj.x_m(2,1))^2);
            
            x_mdot = [obj.V_m*cos(obj.phi_m); obj.V_m*sin(obj.phi_m)];
            x_tdot = [obj.V_t*cos(obj.phi_t); obj.V_t*sin(obj.phi_t)];
            
            
            
            x_dif = obj.x_t - obj.x_m;
            x_dotDif = x_tdot - x_mdot;
            
            [x_difRot , rot] = rotate(x_dif,obj.phi_m);
            x_dotDifRot = rotate(x_dotDif,obj.phi_m);
            
            Rd = ((x_dif(1,1))*(x_dotDif(1,1)) + (x_dif(2,1))*(x_dotDif(2,1)))/R;
            Vc = -Rd;
            
            y = atan((x_difRot(2,1))/(x_difRot(1,1)));
            
            %sightline rate
            y_dot = (x_dotDifRot(2,1)*x_difRot(1,1) - x_dotDifRot(1,1)*x_difRot(2,1))/(x_difRot(1,1)^2 * (1/cos(y))^2);
            
            
            
            
            obj.x_m = obj.x_m + dt*x_mdot;
            x_t = x_t + dt*x_tdot;
            
            
            n = N*y_dot*Vc;
            phi_mdot = n/V_m;
            
            phi_m = phi_m + phi_mdot;
            
            x_mp = [x_mp x_m];
            x_tp = [x_tp x_t];
            end



        end


    end

end