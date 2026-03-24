classdef VS050RobotDH
    %VS050RobotDH Denso VS050 robot representation
    
    methods (Static)
        function robot = kinematics()
            % DH parameters for Denso VS050
            DH_theta  = [0, 0, 0, 0, 0, 0];
            DH_d      = [0.345, 0, 0, 0.305, 0, 0.075];
            DH_a      = [0.075, 0.365, 0.035, 0, 0, 0];
            DH_alpha  = [pi/2, 0, pi/2, -pi/2, pi/2, 0];
            DH_type   = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL, 1, 6);
            
            DH_matrix = [DH_theta;
                         DH_d;
                         DH_a;
                         DH_alpha;
                         DH_type];
            
            robot = DQ_SerialManipulatorDH(DH_matrix, 'standard');
            robot.name = "Denso VS050";
            
            % Add 20cm end-effector along z-axis
            robot.set_effector(1 + DQ.E * 0.5 * DQ.k * 0.2);
        end
    end
end