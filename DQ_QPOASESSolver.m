classdef DQ_QPOASESSolver
    methods
        function obj = DQ_QPOASESSolver()
            % Constructor
        end
        
        function u = solve_quadratic_program(~, H, f, A, b, Aeq, beq)
            % Solve QP: min 0.5*u'*H*u + f'*u
            n = size(H, 1);
            
            % Add regularization
            H_reg = H + 1e-6 * eye(n);
            
            % Handle empty constraints
            if isempty(A) && isempty(Aeq)
                u = -pinv(H_reg) * f;
                return;
            end
            
            % Combine constraints
            if isempty(Aeq) && ~isempty(A)
                A_all = A;
                lbA = -inf(size(b));
                ubA = b;
            elseif isempty(A) && ~isempty(Aeq)
                A_all = Aeq;
                lbA = beq;
                ubA = beq;
            else
                A_all = [A; Aeq];
                lbA = [-inf(size(b)); beq];
                ubA = [b; beq];
            end
            
            % No bounds on variables
            lb = -inf(n, 1);
            ub = inf(n, 1);
            
            % qpOASES options
            options = qpOASES_options('default', ...
                'printLevel', 0, ...
                'enableRegularisation', 1);
            
            % Solve
            try
                [u, ~, exitflag] = qpOASES(H_reg, f, A_all, lb, ub, lbA, ubA, options);
                
                if exitflag ~= 0
                    warning('qpOASES failed, using damped least squares');
                    u = -pinv(H_reg) * f;
                end
            catch
                warning('qpOASES error, using damped least squares');
                u = -pinv(H_reg) * f;
            end
        end
    end
end