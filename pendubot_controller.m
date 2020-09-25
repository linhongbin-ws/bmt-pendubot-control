classdef pendubot_controller < handle
    properties
        % basic param
        motor1
        motor2
        timeStart
        timeNow
        
        %task param
        dT_control = 0.002
        dT_control_lqr = 0.002
        dT_control_pilco = 0.02
        dT_Measure = 0.002
        dT_Sampling = 0.002
        
        dT_print = 0.05
        dT_plotter = 0.2
        dT_PID = 0.01

        
        taskMeasure
        taskControl
        taskSampling
        taskPlotter
        taskPrint
        taskPID
   
        
        isTaskPlotter = false
        isTaskPrinter = false
        isTaskPID = false
        isEnableLQR = false
        isEnablePILCO = false
        isEnableSafeTrip = false
        
        IS_SEND_TOR1 = true
        IS_SEND_TOR2 = false
        

        
        % plot param
        FigID = 20
        fixPlotWindowTime = 15
        
        
        % mearsure param
        gearRatio1 = 1/5
        gearRatio2 = 1
        q1_filRatio = 0
        q2_filRatio = 0
%         dq1_filRatio = 0.9
%         dq2_filRatio = 0.9
        dq1_filRatio = 0.6
        dq2_filRatio = 0.6
        
        isSetOriginMeasure = false
        
        % control param
        PID_p1 = 0.5
        PID_d1 = 0
        PID_p2 = 0.4
        PID_d2 = 0
        
        maxTor1 = 1
        maxTor2 = 1
        
        maxVel1 = 20
        maxVel2 = 20
        
        % lqr
        LQR_a = 0.15
        LQR_b = 0.15
        LQR_p = 0.15
        LQR_d = 0.15
        LQR_wdw = 15

        % pilco
        pilco_policy = []
        load_policy_path = '/home/ben/code/bmt-pendubot-control/archive/data-2020-09-15-hz50/pendubot_13_H150.mat'
        
        
        % record param
        maxRecordBuffer = 20000
        
        % decode position-wraping params
        angThres = 190;
        
        
        % streaming variable
        record_buffer = cell(1, 7)
        origin_absPos1 = 0
        origin_absPos2 = 0
        isInitPlot = true
        desTor1 = 0
        desTor2 = 0 
        isVelExceed1 = false
        isVelExceed2 = false
        is_safeTrip = true
        prevPos1 = 0 
        prevPos2 = 0
        pos1 = 0
        pos2 = 0
        absPos1 = 0 
        absPos2 = 0 
        absC1 = 0
        absC2 = 0
        dq1_fil = 0
        dq2_fil = 0         
        prev_q1 = 0
        q1 = 0
        prev_q2 = 0 
        q2 = 0
        q1_fil = 0 
        q2_fil = 0
        elapseTime
        prev_mTime
        mTime
        des_q1
        des_q2
        des_dq1_fil
        des_dq2_fil
        sub_tor
        pub_pos
        sampling_counter
        test_buffer = []
        test_buffer2 = []
        counter_control = 0
        
        isStopTimer = false
        controller_timer = []
        controller_type = 0
    end
    
    methods
        function obj = pendubot_controller()
            % add path to mx_vesc
            addpath('./mx_vesc')
            
            % create communication with VESCs
            obj.motor1 = mx_vesc('/dev/VESC_001');
            obj.motor2 = mx_vesc('/dev/VESC_002');
            
%             obj.pub_pos = rospublisher('/bmt-pendubot/robot_state');
%             obj.sub_tor = rospublisher('/bmt-pendubot/desired_torque');
%              
        end
        
        function obj = start(obj)
            
            obj.reset_streamVar();
            
            obj.set_isSafeTrip();
            
            % basic task that must run
            obj.taskControl =  mx_task(@()obj.task_control, obj.dT_control); 
            obj.taskMeasure =  mx_task(@()obj.task_measure, obj.dT_Measure); 
            obj.taskSampling =  mx_task(@()obj.task_sampling, obj.dT_Sampling);
            
            % alternative tasks
            if obj.isTaskPrinter
                obj.taskPrint =  mx_task(@()obj.task_printer, obj.dT_print);
            end
            if obj.isTaskPlotter
                obj.taskPlotter =  mx_task(@()obj.task_plotter, obj.dT_plotter);
                figure(obj.FigID)
            end
            if obj.isTaskPID
                obj.taskPID =  mx_task(@()obj.task_PID, obj.dT_PID);
            end


            % start communication
            obj.motor1.open();
            obj.motor2.open();
            
            
            % set current joint position as initial position
            obj.set_measureInitial();
  
            obj.timeStart = mx_sleep(0);
            obj.timeNow = obj.timeStart;
            
            try
              rd = './pilco-matlab/';
              addpath([rd 'base'],[rd 'util'],[rd 'gp'],[rd 'control'],[rd 'loss']);
            catch
            end
            
            fprintf('controller start..\n');
        end
        
        function obj = start_timer(obj)
            obj.isStopTimer = false;
            obj.start();
            obj.controller_timer = timer('ExecutionMode','fixedRate', 'Period', 0.001, 'TimerFcn', @myfun, 'UserData', obj);
            start(obj.controller_timer);
        end
        
        function obj = stop_timer(obj)
            obj.isStopTimer = true;
            obj.controller_timer = [];
        end

            
        
        function stop(obj)
            obj.set_zeroTor();
            obj.motor1.close();
            obj.motor2.close();
            fprintf('controller stop..\n');
        end
        
        function obj = run(obj)
            
            obj.timeNow = mx_sleep(0); % sleeps thread for 10us
            
            % basic task that must run
            obj.taskMeasure.run(obj.timeNow);
            obj.taskControl.run(obj.timeNow);
            obj.taskSampling.run(obj.timeNow);
            
            % alternative tasks
            if obj.isTaskPrinter
                obj.taskPrint.run(obj.timeNow);
            end
            if obj.isTaskPlotter
                obj.taskPlotter.run(obj.timeNow);
            end
            if obj.isTaskPID
                obj.taskPID.run(obj.timeNow);
            end
            
            obj.elapseTime = obj.timeNow - obj.timeStart;
        end
        
        function obj = setTaskPrinter(obj, isTaskPrinter)
            obj.isTaskPrinter = isTaskPrinter;
            if(isTaskPrinter)
                obj.taskPrint =  mx_task(@()obj.task_printer, obj.dT_print);
            else
                obj.taskPrint = [];
            end
        end
        
        function obj = setEnableSafeTrip(obj, isEnableSafeTrip)
            obj.isEnableSafeTrip =  isEnableSafeTrip;
            obj.is_safeTrip = true;
        end
        
        function obj = setTaskPlotter(obj, isTaskPlotter)
            obj.isTaskPlotter = isTaskPlotter;
            if isTaskPlotter
                obj.taskPlotter =  mx_task(@()obj.task_plotter, obj.dT_plotter);
            else
                obj.taskPlotter = [];
            end
        end
        
        
        function obj = setTaskPID(obj, isTaskPID)
            obj.isTaskPID = isTaskPID;
            if isTaskPID
                obj.taskPID = mx_task(@()obj.task_PID, obj.dT_PID);
            else
                obj.taskPID = [];
            end
        end
        
        
        function set_measureInitial(obj)
% 
%             mTime = prev_mTime + obj.dT_control;
            obj.isSetOriginMeasure = true;
 
            % iterate steps to ensure the measurement is steady
            for i = 1:10
                obj.measurement();
                tmp = mx_sleep(0.05);
            end

            obj.origin_absPos1 = obj.absPos1;
            obj.origin_absPos2 = obj.absPos2;
            obj.dq1_fil = 0;
            obj.dq2_fil = 0;
      
            % update some previous variable like prevPos1
            for i = 1:10
                obj.measurement();
                tmp = mx_sleep(0.05);
            end
            
            obj.isSetOriginMeasure = false;
        end
        
        
        function set_zeroTor(obj)
            obj.desTor1 = 0;
            obj.desTor2 = 0;
        end
        
    
        
        function obj = measurement(obj)
            
            % record stream variables from last iteration loop before assignning new variables
            obj.prevPos1 = obj.pos1;
            obj.prevPos2 = obj.pos2;
            obj.prev_q1 = obj.q1;
            obj.prev_q2 = obj.q2;
            obj.prev_mTime = obj.mTime;
            


            % get readings from vesc
            try
                obj.motor1.get_sensors();
            catch
                warning('cannot read sensors from vesc 1');
            end
            try
                obj.motor2.get_sensors();
            catch
                warning('cannot read sensors from vesc 2');
            end
            obj.mTime = mx_sleep(0);
            obj.pos1 = obj.motor1.sensors.pid_pos;      
            obj.pos2 = obj.motor2.sensors.pid_pos;
%             obj.test_buffer = [obj.test_buffer, obj.pos1];
            %obj.test_buffer2 = [obj.test_buffer2, obj.mTime];
            %fprintf('pos1: %.4f\n', obj.pos1);
            

            

            %%%%%%%%%%%%%%%%% get joint position %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%'
            % decode wraps from -180 to 180 to linear readings
            % count wraps number for Joint 1
            if abs(obj.pos1-obj.prevPos1)>obj.angThres && obj.pos1-obj.prevPos1<0
                obj.absC1 = obj.absC1 +1;
            elseif abs(obj.pos1-obj.prevPos1)>obj.angThres && obj.pos1-obj.prevPos1>0
                obj.absC1 = obj.absC1 -1;
            end
            % count wraps number for Joint 2
            if abs(obj.pos2-obj.prevPos2)>obj.angThres && obj.pos2-obj.prevPos2<0
                obj.absC2 = obj.absC2 +1;
            elseif abs(obj.pos2-obj.prevPos2)>obj.angThres && obj.pos2-obj.prevPos2>0
                obj.absC2 = obj.absC2 -1;
            end
            % Convert from motor positions to robot joint position according to gear ratios
            obj.absPos1 = -1 * (obj.pos1 + obj.absC1*360) * obj.gearRatio1; %-1 is just for fliping joint position direction in my case
            obj.absPos2 = -1 * (obj.pos2 + obj.absC2*360) * obj.gearRatio2; % you can remove it
            % Relative joint positions w.r.t. initial zero-origin point
            relPos1 = obj.absPos1 - obj.origin_absPos1;
            relPos2 = obj.absPos2 - obj.origin_absPos2;
            % convert to radian unit
            obj.q1 = 0  + (relPos1 +180) / 180 * pi; 
            obj.q2 = obj.q1 + (relPos2 / 180 * pi); % q2 = q1 + (q2_w.r.t_q1)
             
%             q1_wrap = mod(q1 + pi, 2 * pi) - pi;
%             q2_wrap = mod(q2 + pi, 2 * pi) - pi;

            
            %%%%%%%%%%%%%%%%%% calculate velociy by differentiating joint positions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            dt = obj.dT_Measure;
            dq1 = (obj.q1 - obj.prev_q1) / dt;
            dq2 = (obj.q2 - obj.prev_q2) / dt;
            
            if ~obj.isSetOriginMeasure
                % filtered velocity
                obj.q1_fil = obj.q1_filRatio *  obj.q1_fil + (1-obj.q1_filRatio) * obj.q1; 
                obj.q2_fil = obj.q2_filRatio *  obj.q2_fil + (1-obj.q2_filRatio) * obj.q2;
                % filtered velocity
                obj.dq1_fil = obj.dq1_filRatio *  obj.dq1_fil + (1-obj.dq1_filRatio) * dq1;
                obj.dq2_fil = obj.dq2_filRatio *  obj.dq2_fil + (1-obj.dq2_filRatio) * dq2;
            else  
                obj.q1_fil =  obj.q1;
                obj.q2_fil =  obj.q2;
                obj.dq1_fil = 0;
                obj.dq2_fil = 0;           
            end
            

        end
        
       
        function send_torque(obj)
            
            %%%%%%%%%%%%%%%%%%% safe trip to prevent robot spinning too fast, (like a e-stop in software) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if obj.isEnableSafeTrip
                if obj.isVelExceed1 || obj.isVelExceed2
                    obj.is_safeTrip = false;
                end
                if ~obj.is_safeTrip
                    % send zero torque to stop robot
                    if obj.IS_SEND_TOR1
                        obj.motor1.send_current(0);
                    end
                    if obj.IS_SEND_TOR2
                        obj.motor2.send_current(0);
                    end
                    return
                end
            end
            
            %%%%%%%%%%%%%%%% send torque command to vesc %%%%%%%%%%%%%%%%%%%%%%%%%
            if abs(obj.desTor1) >= obj.maxTor1
                if obj.IS_SEND_TOR1
                    obj.motor1.send_current(sign(obj.desTor1) * obj.maxTor1);
                end
            else
                if obj.IS_SEND_TOR1
                    obj.motor1.send_current(obj.desTor1); % sends current command
                end
            end
            if abs(obj.desTor2) >= obj.maxTor2
                if obj.IS_SEND_TOR2
                    obj.motor2.send_current(sign(obj.desTor2) * obj.maxTor2);
                end
            else
                if obj.IS_SEND_TOR2
                    obj.motor2.send_current(obj.desTor2); % sends current command
                end
            end   
        end
  
        
        function sampling(obj)
            if ~isempty(obj.q1) && ~isempty(obj.q2) && ~isempty(obj.dq1_fil) && ~isempty(obj.dq2_fil) && ~isempty(obj.desTor1) &&  ~isempty(obj.desTor2) && ~isempty(obj.elapseTime) 
                obj.record_buffer{1} = [obj.record_buffer{1}, obj.q1];
                obj.record_buffer{2} = [obj.record_buffer{2}, obj.q2];
                obj.record_buffer{3} = [obj.record_buffer{3}, obj.dq1_fil];
                obj.record_buffer{4} = [obj.record_buffer{4}, obj.dq2_fil];
                obj.record_buffer{5} = [obj.record_buffer{5}, obj.desTor1];
                obj.record_buffer{6} = [obj.record_buffer{6}, obj.desTor2];
                obj.record_buffer{7} = [obj.record_buffer{7}, obj.controller_type];
                obj.record_buffer{8} = [obj.record_buffer{8}, obj.elapseTime];
                obj.sampling_counter =  obj.sampling_counter + 1;
            end
            
            % get rid of the part exceed the maxRecordBuffer size to save memory
            if size(obj.record_buffer{1},2) > obj.maxRecordBuffer  
                for i = 1:7
                    obj.record_buffer{i} = obj.record_buffer{i}(end-obj.maxRecordBuffer+1:end);
                end
            end
        end
            
        function task_printer(obj)
            fprintf("q1: %.2f rad \t q2: %.2f rad \t dq1_fil: %.2f rad\\s \t dq2_fil: %.2f rad\\s \t Tor1: %.2f \t Tor2: %.2f\n",...
                    obj.q1, obj.q2, obj.dq1_fil, obj.dq2_fil, obj.desTor1, obj.desTor2);
        end
        
        function task_plotter(obj)
            
            if ~isempty(obj.record_buffer{1})
                figure(obj.FigID);

                for i = 1:6
                    ax = subplot(3,2,i);
                    if size(obj.record_buffer{end},2)<=(obj.fixPlotWindowTime/obj.dT_control)
                        plot(ax, obj.record_buffer{end}, obj.record_buffer{i});
                    else
                        plot(ax, obj.record_buffer{end}(end - obj.fixPlotWindowTime/obj.dT_control:end), obj.record_buffer{i}(end - obj.fixPlotWindowTime/obj.dT_control : end));
                    end
                end
                drawnow
            end
        end
        
        function plot_sampling_data(obj, save_file_str)
            figure(obj.FigID);
            legend_list = {'q1', 'q2', 'dq1_fil', 'dq2_fil', 'tau1', 'tau2'};
            for i = 1:6
                ax = subplot(3,2,i);
                plot(ax, obj.record_buffer{end}, obj.record_buffer{i});
                legend(legend_list{i})
            end
            if ~isempty(save_file_str)
                try
                    saveas(gcf, save_file_str)
                catch
                    warning("cannot save figure to %s", save_file_str)
                end
            end
        end
        
        function obj = task_control(obj)
           
            if obj.isEnableLQR && ~obj.isEnablePILCO
                [obj.desTor1, ~] = obj.lqr_control();
                if obj.controller_type ~= 1
                    obj.taskControl =  mx_task(@()obj.task_control, obj.dT_control_lqr); 
                    obj.controller_type = 1;
                end
                
            elseif ~obj.isEnableLQR && obj.isEnablePILCO
                obj.desTor1 = obj.pilco_control();
                if obj.controller_type ~= 2
                    obj.taskControl =  mx_task(@()obj.task_control, obj.dT_control_pilco); 
                    fprintf('dt is %.4f\n\n',obj.dT_control_pilco)
                    obj.controller_type = 2;
                end
                
            elseif obj.isEnableLQR && obj.isEnablePILCO
                [obj.desTor1, is_in_shootingRange] = obj.lqr_control();  
                
                if ~is_in_shootingRange
                    obj.desTor1 = obj.pilco_control();
                    if obj.controller_type ~= 2
                        obj.taskControl =  mx_task(@()obj.task_control, obj.dT_control_pilco);
                        fprintf('Switch to pilco controller, dt: %.4f\n\n',obj.dT_control_pilco)
                        obj.controller_type = 2;
                    end
                else
                    if obj.controller_type ~= 1
                        obj.taskControl =  mx_task(@()obj.task_control, obj.dT_control_lqr); 
                        fprintf('Switch to lqr controller, dt: %.4f\n\n',obj.dT_control_lqr)
                        obj.controller_type = 1;
                    end
                end
              
            else
                obj.controller_type = 0;
            end
            
            obj.send_torque();
        end
        
        function desTor1 = pilco_control(obj)
%             if obj.counter_control == 100
%                 is_print = true;
%                 obj.counter_control = 0;
%             else
%                 is_print = false;
%                 obj.counter_control = obj.counter_control +1;
%             end
            if isempty(obj.pilco_policy)
                load(obj.load_policy_path, 'policy');
                obj.pilco_policy = policy;
            end
            u = pilco_control(obj.q1, obj.q2, obj.dq1_fil, obj.dq2_fil, obj.pilco_policy);
            desTor1 = u;
%             obj.desTor1 =  current;
 
        end
        
        function [desTor1, is_in_shootingRange] = lqr_control(obj)
            if obj.counter_control == 100
                is_print = true;
                obj.counter_control = 0;
            else
                is_print = false;
                obj.counter_control = obj.counter_control +1;
            end
            is_print = false;
            [u, is_in_shootingRange] = LQR_pendubot([obj.q1+pi, obj.q2-obj.q1, obj.dq1_fil,  obj.dq2_fil-obj.dq1_fil].', ...
                                obj.LQR_a, obj.LQR_b, obj.LQR_p, obj.LQR_d, obj.LQR_wdw, is_print);
            desTor1 = u(1)/(29.2e-3);
%             obj.desTor1 =  current;
 
        end
        
        function obj = task_measure(obj)
            obj.measurement();
            obj.updateSafeVelocity(); 
        end
              
        function obj = task_sampling(obj)
            obj.sampling();
            
%             %%%%%%%%%%%%%%%%% ROS Publisher %%%%%%%%%%%%%%%%
%             msg = rosmessage('sensor_msgs/JointState');
%             msg.Position = [obj.q1, obj.q2];
%             msg.Velocity = [obj.q1_fil, obj.q2_fil];
%             msg.Effort = [obj.desTor1, obj.desTor2]
%             send(obj.pub_pos, msg);
%             
        end
        
        function task_PID(obj)
            tor1 = (obj.des_q1 - obj.q1) * obj.PID_p1 + (obj.des_dq1_fil - obj.dq1_fil)* obj.PID_d1;
            obj.desTor1 = sign(tor1) * min(abs(tor1), obj.maxTor1);
            
            tor2 = (obj.des_q2 - obj.q2) * obj.PID_p2 + (obj.des_dq2_fil - obj.dq2_fil)* obj.PID_d2;
            obj.desTor2 = sign(tor2) * min(abs(tor2), obj.maxTor2);
        end
        
        function move_joint(obj, q1, q2)
            obj.des_q1 = q1;
            obj.des_q2 = q2;
            obj.des_dq1_fil = 0;
            obj.des_dq2_fil = 0;
          
        end
    
        
        function obj = delete_controller(obj)
            obj.motor1.delete();
            obj.motor2.delete();
        end
        
        function set_isSafeTrip(obj)
            obj.is_safeTrip = true;
        end
        
        function set_enable_lqr(obj, flag)
            obj.isEnableLQR = flag;
        end
        
        function set_enable_pilco(obj, flag)
            obj.isEnablePILCO = flag;
        end
        
        function updateSafeVelocity(obj)
            if abs(obj.dq1_fil) >= obj.maxVel1 
                obj.isVelExceed1 = true;
            else
                obj.isVelExceed1 = false;
            end
            
            if abs(obj.dq2_fil) >= obj.maxVel2
                obj.isVelExceed2 = true;
            else
                obj.isVelExceed2 = false;
            end
        end
        
        function reset_streamVar(obj)
            obj.sampling_counter = 0;
            obj.record_buffer = cell(1, 8);
            obj.origin_absPos1 = 0;
            obj.origin_absPos2 = 0;
            obj.isInitPlot = true;
            obj.desTor1 = 0;
            obj.desTor2 = 0;
            obj.isVelExceed1 = false;
            obj.isVelExceed2 = false;
            obj.is_safeTrip = true;
            obj.prevPos1 = 0;
            obj.prevPos2 = 0;
            obj.pos1 = 0;
            obj.pos2 = 0;
            obj.absPos1 = 0; 
            obj.absPos2 = 0; 
            obj.absC1 = 0;
            obj.absC2 = 0;
            obj.dq1_fil = 0;
            obj.dq2_fil = 0;        
            obj.prev_q1 = 0;
            obj.q1 = 0;
            obj.prev_q2 = 0; 
            obj.q2 = 0;
            obj.q1_fil = 0; 
            obj.q2_fil = 0;
            obj.elapseTime;
        end
 
    end
end

function myfun(obj, evt)
    controller = get(obj, 'UserData');
    controller.run();
    
    if controller.isStopTimer
        stop(obj);
        controller.set_zeroTor();
        controller.stop();
        controller.delete_controller();
        clearvars obj controller;
    end
end

