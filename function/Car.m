classdef Car < handle
    properties(GetAccess=public)
        ID = [];
    end
    properties
        LnID = [];
        OgLnID = []; % New: store original Lane ID
        LCng = 0;
        LcV = 20;
        Clr = 1;
        Lpf = 10;
        X = 0;
        Y = 0;
        V = 20;
        A = [0;0;0];
        Ac = 0;
        AcR = -99;
        IPV = 0;
        Vd = 25;
        Th = 1.3;
        R0 = 4.0;
        Cx = 0;
        TrvT = 0;
        Trj3 = [];
        Lpos = 0;
        CACC = 0;

        % Storage
        t = [];
        dt = 0.5;
        str_X = [];
        str_Y = [];
        str_V = [];
        str_Ac = [];

        % Noise
        noisy_X = 0;
        noisy_Y = 0;
        noisy_V = 0;
        noisy_Ac = 0;

        % Noise variance
        variance_X = 14.27141916;
        variance_Y = 21.61521545;
        variance_V = 0.01;
        variance_Ac = 0.01;
        % variance_X = 1e-6;
        % variance_Y = 1e-6;
        % variance_V = 1e-6;
        % variance_Ac = 1e-6;

        % Realistic noise control properties
        noise_timer = 0;            % Timer to track state duration
        noise_state_duration = 200; % Initial duration for each noise state
        noise_mode = true;          % Current noise mode (true = accurate, false = noisy)
        noise_cycle_length = 500;   % Total cycle length for sine wave noise variation

        % V2V_data
        v2v_data = struct();
        str_v2v_data = struct();
        str_v2v_data_short = struct();

        % Model
        model_v2v = struct();
        model_v2v_short = struct();

        % KF
        KF_v2v = struct();

        KF_v2v_short = struct();

        % Cred
        cred = struct();
        str_cred = struct();
    end

    methods
        function obj = Car(ID, Ln, X)
            global Vd;
            obj.ID = ID;
            obj.LnID = Ln;
            obj.OgLnID = Ln; % Set the original lane ID
            obj.Clr = Ln + 1;
            obj.Y = Ln;
            obj.Th = 1.2 + 1.2 * rand;
            % obj.Vd = 21 + 6 * rand + 2 - 4 * Ln;
            obj.Vd = 21 + 6 * rand + 2;
            obj.Lpf = 6;
            obj.R0 = 4;
            obj.X = X;

            % Store the initial values in str_* for the true data
            obj.str_X = obj.X;
            obj.str_Y = obj.Y;
            obj.str_V = obj.V;
            obj.str_Ac = obj.Ac;
            obj.t = 0;
        end
    end

    methods(Static)
        function Acel = IDM(H, P)
            S = P.X - H.X - H.R0;
            dV = H.V - P.V;
            S0 = 1.0;
            if(H.LnID == 2)
                S0 = S0 + 2;
            end
            if(S < 0.95)
                S = 0.95;
            end
            a = 1.5;
            b = 2.5;
            Ss = S0 + H.V * H.Th + H.V * dV / (2 * sqrt(a * b));
            f = a * (1 - (H.V / H.Vd)^4 - (Ss / S)^2);
            f = 8 * tanh(f / 8.0) * (1.0 + randn / 30.);

            if(H.CACC == 1) % Extensions needed
            end
            Acel = f;
        end
        function Xnew = fdx(obj)
            global dt;
            Xnew = obj.X + obj.V * dt + 0.5 * obj.Ac * dt^2;
            if(obj.V < 0.001 && obj.Ac < 0)
                Xnew = obj.X;
            end
        end
        function Vnew = fdv(obj)
            global dt;
            Vnew = obj.V + obj.Ac * dt;
            if(Vnew < 0.001)
                Vnew = 0;
            end
        end

        % Noise
        function Acel = IDMN(H, P)
            S = P.noisy_X - H.X - H.R0;
            dV = H.V - P.noisy_V;
            S0 = 1.0;
            if(H.LnID == 2)
                S0 = S0 + 2;
            end
            if(S < 0.95)
                S = 0.95;
            end
            a = 1.5;
            b = 2.5;
            Ss = S0 + H.V * H.Th + H.V * dV / (2 * sqrt(a * b));
            f = a * (1 - (H.V / H.Vd)^4 - (Ss / S)^2);
            f = 8 * tanh(f / 8.0) * (1.0 + randn / 30.);

            if(H.CACC == 1) % Extensions needed
            end
            Acel = f;
        end

        % Store normal data
        function storeNormalState(obj)
            obj.str_X = [obj.str_X; obj.X];            % Store original X
            obj.str_Y = [obj.str_Y; obj.Y];            % Store original Y
            obj.str_V = [obj.str_V; obj.V];            % Store original V
            obj.str_Ac = [obj.str_Ac; obj.Ac];          % Store original Ac
            obj.t = [obj.t; obj.t(end) + obj.dt];           % Store simulation time
        end

        % Reset short-term data for both V2V and model data
        function reset_str(obj, trackedVehicle)
            % Convert vehicle ID to a valid field name
            vehicleID = trackedVehicle.ID;
            fieldName = ['v', num2str(vehicleID)];  % Convert vehicleID to a string-based field name

            % Reset short-term V2V data
            if isfield(obj.str_v2v_data_short, fieldName)
                obj.str_v2v_data_short.(fieldName).t = 0;
                obj.str_v2v_data_short.(fieldName).X = obj.getLastElement(obj.str_v2v_data_short.(fieldName).X);
                obj.str_v2v_data_short.(fieldName).Y = obj.getLastElement(obj.str_v2v_data_short.(fieldName).Y);
                obj.str_v2v_data_short.(fieldName).V = obj.getLastElement(obj.str_v2v_data_short.(fieldName).V);
                obj.str_v2v_data_short.(fieldName).Ac = obj.getLastElement(obj.str_v2v_data_short.(fieldName).Ac);
            else
                % Initialize if missing
                obj.str_v2v_data_short.(fieldName) = struct('t', 0, 'X', 0, 'Y', 0, 'V', 0, 'Ac', 0);
            end

            % Reset short-term model data
            if isfield(obj.model_v2v_short, fieldName)
                obj.model_v2v_short.(fieldName).t = 0;
                obj.model_v2v_short.(fieldName).Ad = obj.getLast3DMatrix(obj.model_v2v_short.(fieldName).Ad);
                obj.model_v2v_short.(fieldName).Bd = obj.getLast3DMatrix(obj.model_v2v_short.(fieldName).Bd);
                obj.model_v2v_short.(fieldName).Cd = obj.model_v2v_short.(fieldName).Cd;  % Assume constant
                obj.model_v2v_short.(fieldName).Dd = obj.model_v2v_short.(fieldName).Dd;  % Assume constant
                obj.model_v2v_short.(fieldName).SigmaW = obj.getLast3DMatrix(obj.model_v2v_short.(fieldName).SigmaW);
                obj.model_v2v_short.(fieldName).SigmaV = obj.model_v2v_short.(fieldName).SigmaV;  % Assume constant
                obj.model_v2v_short.(fieldName).u = obj.getLastElement(obj.model_v2v_short.(fieldName).u);
                obj.model_v2v_short.(fieldName).x = obj.getLastColumn(obj.model_v2v_short.(fieldName).x);
                obj.model_v2v_short.(fieldName).z = obj.getLastColumn(obj.model_v2v_short.(fieldName).z);
            else
                % Initialize if missing
                obj.model_v2v_short.(fieldName) = struct('t', 0, 'Ad', [], 'Bd', [], 'Cd', eye(3), 'Dd', 0, ...
                    'SigmaW', [], 'SigmaV', eye(3), 'u', [], 'x', [], 'z', []);
            end

            % Reset short-term KF data
            if isfield(obj.KF_v2v_short, fieldName)
                obj.KF_v2v_short.(fieldName).t = 0;
                obj.KF_v2v_short.(fieldName).xhatstore = obj.getLastColumn(obj.KF_v2v_short.(fieldName).xhatstore);
                obj.KF_v2v_short.(fieldName).zhatstore = obj.getLastColumn(obj.KF_v2v_short.(fieldName).zhatstore);
            else
                % Initialize if missing
                obj.KF_v2v_short.(fieldName) = struct('t', [], 'xhatstore', [], 'zhatstore', []);
            end

            % Reset Credibility Data
            if isfield(obj.cred, fieldName)
                % Reset time to 0
                obj.cred.(fieldName).t = 0;

                % Ensure the credibility score and trustworthiness keep the last valid value
                if ~isempty(obj.cred.(fieldName).credibility_score)
                    obj.cred.(fieldName).credibility_score = obj.getLastElement(obj.cred.(fieldName).credibility_score);
                else
                    obj.cred.(fieldName).credibility_score = 1; % Default initial credibility score
                end

                if ~isempty(obj.cred.(fieldName).trustworthy)
                    obj.cred.(fieldName).trustworthy = obj.getLastElement(obj.cred.(fieldName).trustworthy);
                else
                    obj.cred.(fieldName).trustworthy = 1; % Default trustworthy value
                end
            else
                % Initialize if missing
                obj.cred.(fieldName) = struct('t', 0, 'credibility_score', 1, 'trustworthy', 1);
            end
        end

        % Helper function to get the last element of a 1D array
        function lastElement = getLastElement(array)
            if isempty(array)
                lastElement = 0;  % Default to 0 if the array is empty
            else
                lastElement = array(end);
            end
        end

        % Helper function to get the last column of a 2D matrix
        function lastColumn = getLastColumn(matrix)
            if isempty(matrix)
                lastColumn = [];
            else
                lastColumn = matrix(:, end);
            end
        end

        % Helper function to get the last 3D matrix along the 3rd dimension
        function lastMatrix = getLast3DMatrix(matrix)
            if isempty(matrix)
                lastMatrix = [];
            else
                lastMatrix = matrix(:, :, end);
            end
        end



        % Generate noise
        function generateNoise(obj)
            % Apply Gaussian noise to the car's properties
            % Constant noise
            % obj.noisy_X = obj.X + sqrt(obj.variance_X) * randn(1, 1);
            % obj.noisy_Y = obj.Y + sqrt(obj.variance_Y) * randn(1, 1);
            % obj.noisy_V = obj.V + sqrt(obj.variance_V) * randn(1, 1);
            % obj.noisy_Ac = obj.Ac + sqrt(obj.variance_Ac) * randn(1, 1);

            % No noise
            % obj.noisy_X = obj.X;
            % obj.noisy_Y = obj.Y;
            % obj.noisy_V = obj.V;
            % obj.noisy_Ac = obj.Ac;

            % Real Noise
            % Increment noise state duration timer
            obj.noise_timer = obj.noise_timer + 1;

            % Check if it's time to switch noise state
            if obj.noise_timer >= obj.noise_state_duration
                % Switch noise mode (toggle between accurate and inaccurate modes)
                obj.noise_mode = ~obj.noise_mode;

                % Reset timer and assign a new random state duration (e.g., 100 to 300 time steps)
                obj.noise_timer = 0;
                obj.noise_state_duration = randi([100, 300]);
            end

            % Apply noise based on the current mode
            if obj.noise_mode
                noise_factor = 0.1;  % Accurate mode (low noise)
            else
                noise_factor = 1.5;  % Inaccurate mode (high noise)
            end

            % Apply Gaussian noise with mode-dependent variance
            obj.noisy_X = obj.X + noise_factor * sqrt(obj.variance_X) * randn(1, 1);
            obj.noisy_Y = obj.Y + noise_factor * sqrt(obj.variance_Y) * randn(1, 1);
            obj.noisy_V = obj.V + noise_factor * sqrt(obj.variance_V) * randn(1, 1);
            obj.noisy_Ac = obj.Ac + noise_factor * sqrt(obj.variance_Ac) * randn(1, 1);
        end

        % Update V2V data
        function updateV2VState(obj, trackedVehicle)
            vehicleID = trackedVehicle.ID;
            fieldName = ['v', num2str(vehicleID)]; % Prepend 'v' to make a valid field name

            % Create single-value V2V data struct if it doesn't exist
            if ~isfield(obj.v2v_data, fieldName)
                obj.v2v_data.(fieldName) = struct('X', 0, 'Y', 0, 'V', 20, 'Ac', 0);
            end

            % Update single-value V2V data
            obj.v2v_data.(fieldName).X = trackedVehicle.noisy_X;
            obj.v2v_data.(fieldName).Y = trackedVehicle.noisy_Y;
            obj.v2v_data.(fieldName).V = trackedVehicle.noisy_V;
            obj.v2v_data.(fieldName).Ac = trackedVehicle.noisy_Ac;
        end

        % Store V2V data for rating
        function strV2VState(obj, trackedVehicle)
            vehicleID = trackedVehicle.ID;
            fieldName = ['v', num2str(vehicleID)]; % Prepend 'v' to make a valid field name

            % Create historical data struct if it doesn't exist, initialize
            % it
            if ~isfield(obj.str_v2v_data, fieldName)
                obj.str_v2v_data.(fieldName) = struct('t', trackedVehicle.t(1), ...
                    'X', trackedVehicle.str_X(1), ...
                    'Y', trackedVehicle.str_Y(1), ...
                    'V', trackedVehicle.str_V(1), ...
                    'Ac', trackedVehicle.str_Ac(1));
            end

            % Append to historical data
            obj.str_v2v_data.(fieldName).t = [obj.str_v2v_data.(fieldName).t; obj.t(end) + obj.dt];
            obj.str_v2v_data.(fieldName).X = [obj.str_v2v_data.(fieldName).X; trackedVehicle.noisy_X];
            obj.str_v2v_data.(fieldName).Y = [obj.str_v2v_data.(fieldName).Y; trackedVehicle.noisy_Y];
            obj.str_v2v_data.(fieldName).V = [obj.str_v2v_data.(fieldName).V; trackedVehicle.noisy_V];
            obj.str_v2v_data.(fieldName).Ac = [obj.str_v2v_data.(fieldName).Ac; trackedVehicle.noisy_Ac];


            % Short term
            if ~isfield(obj.str_v2v_data_short, fieldName)
                obj.str_v2v_data_short.(fieldName) = struct('t', trackedVehicle.t(1), ...
                    'X', trackedVehicle.str_X(1), ...
                    'Y', trackedVehicle.str_Y(1), ...
                    'V', trackedVehicle.str_V(1), ...
                    'Ac', trackedVehicle.str_Ac(1));
            end

            % Append to historical data
            obj.str_v2v_data_short.(fieldName).t = [obj.str_v2v_data_short.(fieldName).t; obj.str_v2v_data_short.(fieldName).t(end) + obj.dt];
            obj.str_v2v_data_short.(fieldName).X = [obj.str_v2v_data_short.(fieldName).X; trackedVehicle.noisy_X];
            obj.str_v2v_data_short.(fieldName).Y = [obj.str_v2v_data_short.(fieldName).Y; trackedVehicle.noisy_Y];
            obj.str_v2v_data_short.(fieldName).V = [obj.str_v2v_data_short.(fieldName).V; trackedVehicle.noisy_V];
            obj.str_v2v_data_short.(fieldName).Ac = [obj.str_v2v_data_short.(fieldName).Ac; trackedVehicle.noisy_Ac];
        end

        % Model
        function v2vmodel(obj, trackedVehicle)
            vehicleID = sprintf('v%d', trackedVehicle.ID);
            data = obj.str_v2v_data_short.(vehicleID);

            % disp(["data : ", mat2str(size(data.t)), " vehicleID: ", vehicleID]);

            % short-term storage
            if ~isfield(obj.model_v2v_short, vehicleID)
                obj.model_v2v_short.(vehicleID) = struct('t', [], 'Ad', [], 'Bd', [], 'Cd', [], 'Dd', [], 'SigmaW', [], 'SigmaV', [], ...
                    'u', [], 'x', [], 'z', []);
            end

            % extract dataz
            X_noisy = data.X;
            Y_noisy = data.Y;
            Vel_noisy = data.V;
            Acc_noisy = data.Ac;
            t = data.t;

            % smooth out the data using Savitzky-Golay filter

            % Check data length for filtering
            data_length = length(X_noisy);

            if data_length < 3
                % Skip filtering if data is too short
                % disp("Skipping filtering: insufficient data.");
                X = X_noisy;
                Y = Y_noisy;
                Vel = Vel_noisy;
                Acc = Acc_noisy;
            else
                % Define dynamic filtering parameters
                polynomial_order = min(3, data_length - 1);
                window_length = min(7, data_length);
                if mod(window_length, 2) == 0
                    window_length = window_length - 1;  % Ensure odd window length
                end

                % Apply Savitzky-Golay filtering
                X = sgolayfilt(X_noisy, polynomial_order, window_length);
                Y = sgolayfilt(Y_noisy, polynomial_order, window_length);
                Vel = sgolayfilt(Vel_noisy, polynomial_order, window_length);
                Acc = sgolayfilt(Acc_noisy, polynomial_order, window_length);
            end

            % % heading angle
            % dx = diff(X);
            % dy = diff(Y);
            % % Check if there is enough data to calculate theta
            % if isempty(dx) || isempty(dy)
            %     % disp('Insufficient data to calculate theta.');
            %     theta = zeros(size(X));  % Default fallback if no data for theta
            % else
            %     theta = atan2(dy, dx);
            %     theta = [theta; theta(end)];  % Extend theta to match the size of X or Y
            %     % disp(['Calculated theta: ', num2str(theta')]);
            %     theta = unwrap(theta);  % Ensure no sudden jumps
            % end

            % initialize matrices
            num_steps = length(t);

            % continuous-time
            % Ac = zeros(3, 3, num_steps); % Preallocate a 3D array for A matrices
            % 
            % % Calculate A matrix for each time step
            % for k = 1:num_steps
            %     Ac(:, :, k) = [0, 0, cos(theta(k));
            %         0, 0, sin(theta(k));
            %         0, 0, 0];
            % end
            % 
            % Bc = [0; 0; 1];
            % Cc = eye(3);
            % Dc = 0;

            Ac = [0, 1; 0, 0];  
            Bc = [0; 1];        
            Cc = eye(2);       
            Dc = 0;

            % discrete-time
            % Ad = zeros(3, 3, num_steps);
            % Bd = zeros(3, 1, num_steps);
            % for k = 1:num_steps
            %     Ad(:, :, k) = [1, 0, obj.dt * cos(theta(k));
            %         0, 1, obj.dt * sin(theta(k));
            %         0, 0, 1];
            %     Bd(:, :, k) = [0.5 * obj.dt^2 * cos(theta(k));
            %         0.5 * obj.dt^2 * sin(theta(k));
            %         obj.dt];
            % end
            % Cd = Cc;
            % Dd = Dc;
            Ad_c = [1, obj.dt;
                    0, 1   ];   % 2×2
            Bd_c = [0.5*obj.dt^2;
                    obj.dt    ]; % 2×1
            Ad = repmat(Ad_c,1,1,num_steps);
            Bd = repmat(Bd_c,1,1,num_steps);
            Cd = Cc;   % eye(2)
            Dd = 0;

            % Initilize control input and intial values and noise variable
            u = Acc;
            % x = zeros(3, length(t) + 1);
            % z = zeros(3, length(t));
            % 
            % % initial value
            % x(:,1) = [X(1); Y(1); Vel(1)];
            x = zeros(2, length(t)+1);
            z = zeros(2, length(t));
            x(:,1) = [X(1); Vel(1)]; % Y removed from state

            % % scaling
            % scaling_X = 1.0;
            % scaling_Y = 1.2;
            % scaling_V = 0.8;
            % 
            % % value for noise
            % Bw = Bd; % noise is the input noise
            % 
            % Sw = obj.variance_Ac; % continuous noise [acc]
            % Sv = diag([obj.variance_X * scaling_X, obj.variance_Y * scaling_Y, obj.variance_V * scaling_V]); % continuous noise [x, y, vel]
            % 
            % SigmaW = zeros(3, 3, num_steps); % discrete noise
            % epsilon = 1e-6; % Small positive value
            % 
            % for k = 1:num_steps
            %     Z = [-Ac(:, :, k) Bw(:, :, k)*Sw*Bw(:, :, k)'
            %         zeros(3) Ac(:, :, k)'];
            %     C = expm(Z * obj.dt);
            %     c12 = C(1:3, 4:6);
            %     c22 = C(4:6, 4:6);
            %     SigmaW(:, :, k) = c22' * c12 + epsilon * eye(3);
            % end
            % 
            % SigmaV = Sv/obj.dt;
            Sw = obj.variance_Ac;
            Sv = diag([obj.variance_X,
                       obj.variance_V]);
            
            % Closed-form SigmaW for CA model
            SigmaW_c = Bd_c * Sw * Bd_c';
            SigmaW = repmat(SigmaW_c,1,1,num_steps);
            
            % Bug 4 fix: Sv is already discrete variance
            % — do NOT divide by dt
            SigmaV = Sv;

            % Model

            for k = 1:length(t)-1 % Loop until the second-to-last element
                % x(:,k+1) = Ad(:,:,k) * x(:,k) + Bd(:, :, k) * u(k+1) + chol(SigmaW(:, :, k), 'lower') * randn(3, 1); % Use u(k+1) correctly
                x(:,k+1) = Ad(:,:,k) * x(:,k) + Bd(:, :, k) * u(k+1) + chol(SigmaW(:, :, k), 'lower') * randn(2, 1); % Use u(k+1) correctly
                % z(:,k) = Cd * x(:,k) + Dd * u(k) + chol(SigmaV, 'lower') * randn(3, 1); % Still use u(k) for the current state
                % z(:,k) = Cd * x(:,k) + Dd * u(k)
                z(:,k) = Cd * x(:,k)
            end

            % After the loop, handle the last state update and output calculation
            % x(:, end) = Ad(:,:,end) * x(:, end-1) + Bd(:, :, end) * u(end); % Last state update
            % x = x(:, 1:length(t));
            % z(:, end) = Cd * x(:, end) + Dd * u(end); % Last output
            x(:,end) = Ad(:,:,end)*x(:,end-1) + Bd(:,:,end)*u(end);
            x = x(:,1:length(t));
            z(:,end) = Cd*x(:,end);
            % Dd*u removed (Dd=0, and u not in z)


            % Save short-term data
            obj.model_v2v_short.(vehicleID) = struct(...
                't', t, 'Ad', Ad, 'Bd', Bd, 'Cd', Cd, 'Dd', Dd, ...
                'SigmaW', SigmaW, 'SigmaV', SigmaV, 'u', u, 'x', x, 'z', z);

            % disp(["model_v2v_short : ", mat2str(size(obj.model_v2v_short.(vehicleID).t)), " vehicleID: ", vehicleID]);


            % Save long-term data

            % Ensure correct initialization for model fields
            % if ~isfield(obj.model_v2v, vehicleID)
            %     obj.model_v2v.(vehicleID) = struct(...
            %         't', [], 'Ad', zeros(3, 3, 0), 'Bd', zeros(3, 1, 0), ...
            %         'Cd', Cd, 'Dd', Dd, 'SigmaW', zeros(3, 3, 0), ...
            %         'SigmaV', SigmaV, 'u', zeros(0, 1), 'x', zeros(3, 0), 'z', zeros(3, 0));
            % end
            if ~isfield(obj.model_v2v, vehicleID)
                obj.model_v2v.(vehicleID) = struct(...
                    't', [], 'Ad', zeros(2, 2, 0), 'Bd', zeros(2, 1, 0), ...
                    'Cd', Cd, 'Dd', Dd, 'SigmaW', zeros(2, 2, 0), ...
                    'SigmaV', SigmaV, 'u', zeros(0, 1), 'x', zeros(2, 0), 'z', zeros(2, 0));
            end

            % Append data to long-term model with consistency checks
            if isempty(obj.model_v2v.(vehicleID).t)
                % First append: Add the entire initial data (including initial t, Ad, Bd, etc.)
                obj.model_v2v.(vehicleID).t = obj.t;
                obj.model_v2v.(vehicleID).Ad = Ad;
                obj.model_v2v.(vehicleID).Bd = Bd;
                obj.model_v2v.(vehicleID).SigmaW = SigmaW;
                obj.model_v2v.(vehicleID).u = u;
                obj.model_v2v.(vehicleID).x = x;
                obj.model_v2v.(vehicleID).z = z;
            else
                % Subsequent appends: Add only new data to avoid duplicating the initial state
                obj.model_v2v.(vehicleID).t = obj.t;
                obj.model_v2v.(vehicleID).Ad = cat(3, obj.model_v2v.(vehicleID).Ad, Ad(:, :, 2:end));
                obj.model_v2v.(vehicleID).Bd = cat(3, obj.model_v2v.(vehicleID).Bd, Bd(:, :, 2:end));
                obj.model_v2v.(vehicleID).SigmaW = cat(3, obj.model_v2v.(vehicleID).SigmaW, SigmaW(:, :, 2:end));
                obj.model_v2v.(vehicleID).u = [obj.model_v2v.(vehicleID).u; u(2:end)];
                obj.model_v2v.(vehicleID).x = [obj.model_v2v.(vehicleID).x, x(:, 2:end)];
                obj.model_v2v.(vehicleID).z = [obj.model_v2v.(vehicleID).z, z(:, 2:end)];
            end

            % disp(["model_v2v_short : ", mat2str(size(obj.model_v2v.(vehicleID).t)), " vehicleID: ", vehicleID]);
        end

        % KF
        function KF(obj, trackedVehicle)
            vehicleID = sprintf('v%d', trackedVehicle.ID);
            data = obj.model_v2v_short.(vehicleID);

            % short-term
            if ~isfield(obj.KF_v2v_short, vehicleID)
                obj.KF_v2v_short.(vehicleID) = struct('t', [], 'xhatstore', [], 'zhatstore', []);
            end

            % extract data
            t = data.t;
            Ad = data.Ad;
            Bd = data.Bd;
            Cd = data.Cd;
            Dd = data.Dd;
            SigmaW = data.SigmaW;
            SigmaV = data.SigmaV;
            u = data.u;
            x = data.x;
            z = data.z;

            % Determine number of states and timesteps
            [nx ,nt] = size (x);
            [nz, nt_] = size (z);
            % Initialize state estimate and covariance
            xhat = x(:, 1); 
            % SigmaX = zeros(nx ,nx);
            SigmaX = diag([obj.variance_X, obj.variance_V]);
            % Initialize storage for state / bounds for plotting purposes
            xhatstore = zeros(nx ,nt);
            xhatstore(:, 1) = xhat;
            % Initialize storage for measurement output
            zhatstore = zeros (nz ,nt_);
            zhatstore(:, 1) = x(:, 1);
            for k = 2:length(t)
                % KF Step 1a: State prediction time update
                xhat = Ad(:,:,k-1) * xhat + Bd(:,:,k-1)*u(k-1); % use prior value of "u"
                % KF Step 1b: Prediction - error covariance time update
                SigmaX = Ad(:,:,k-1) * SigmaX * Ad(:,:,k-1)' + SigmaW(:,:,k-1);
                % KF Step 1c: Predict system output
                zhat = Cd* xhat + Dd*u(k); % use present value of "u"

                % KF Step 2a: Compute estimator matrix
                L = SigmaX * Cd' /( Cd* SigmaX *Cd'+ SigmaV);
                % KF Step 2b: State estimate measurement update
                % xhat = xhat + L*(z(:,k) - zhat);
                % z_meas = [obj.str_v2v_data_short.(vehicleID).X'; ...
                %           obj.str_v2v_data_short.(vehicleID).Y'; ...   % ← add this line back
                %           obj.str_v2v_data_short.(vehicleID).V'];
                z_meas = [obj.str_v2v_data_short.(vehicleID).X'; ...
                          obj.str_v2v_data_short.(vehicleID).V'];
                xhat = xhat + L * (z_meas(:,k) - zhat);
                % KF Step 2c: Estimation - error covariance measurement update
                % SigmaX = (eye(nx)-L*Cd)* SigmaX;
                ILC = eye(nx) - L * Cd;
                SigmaX = ILC * SigmaX * ILC' + L * SigmaV * L';
                % [ Store estimate and bounds for evaluation / plotting purposes ]
                xhatstore(:,k) = xhat;
                zhatstore(:,k) = zhat;
            end

            % Save short-term
            obj.KF_v2v_short.(vehicleID) = struct('t', t, 'xhatstore', xhatstore, 'zhatstore', zhatstore);

            % disp(["KF_v2v-short : ", mat2str(size(obj.KF_v2v_short.(vehicleID).xhatstore)), " vehicleID: ", vehicleID]);


            % Save long-term
            if ~isfield(obj.KF_v2v, vehicleID)
                obj.KF_v2v.(vehicleID) = struct('t', [], 'xhatstore', [], 'zhatstore', []);
            end

            % Append data to long-term model with consistency checks
            if isempty(obj.KF_v2v.(vehicleID).t)
                % First append: Add the entire initial data (including initial t, Ad, Bd, etc.)
                obj.KF_v2v.(vehicleID).t = obj.t;
                obj.KF_v2v.(vehicleID).xhatstore = xhatstore;
                obj.KF_v2v.(vehicleID).zhatstore = zhatstore;
            else
                % Subsequent appends: Add only new data to avoid duplicating the initial state
                obj.KF_v2v.(vehicleID).t = obj.t;
                obj.KF_v2v.(vehicleID).xhatstore = [obj.KF_v2v.(vehicleID).xhatstore, xhatstore(:, 2:end)];
                obj.KF_v2v.(vehicleID).zhatstore = [obj.KF_v2v.(vehicleID).zhatstore, zhatstore(:, 2:end)];
            end

            % disp(["KF_v2v : ", mat2str(size(obj.KF_v2v.(vehicleID).xhatstore)), " vehicleID: ", vehicleID]);
        end

        % credibility
        function cred_v2v(obj, trackedVehicle)
            vehicleID = sprintf('v%d', trackedVehicle.ID);
            dataKF = obj.KF_v2v_short.(vehicleID);
            dataV2V = obj.str_v2v_data_short.(vehicleID);

            if ~isfield(obj.cred, vehicleID)
                obj.cred.(vehicleID) = struct('t', 0, 'credibility_score', 1, 'trustworthy', 1);
            end

            % Parameters
            threshold = 0.70;                % Threshold for trustworthiness
            % wx = 1/3; wy = 1/3; wv = 1/3;    % Weights for X, Y, and V
            wx = 1/2; wv = 1/2; 

            % extract data
            X = dataV2V.X';
            % Y = dataV2V.Y';
            V = dataV2V.V';
            zhat = dataKF.zhatstore;

            % Compute error
            error_x = abs(X - zhat(1, :));
            % error_y = abs(Y - zhat(2, :));
            % error_v = abs(V - zhat(3, :));
            error_v = abs(V - zhat(2, :));

            % Calculate dynamic error sensitivity based on the range or standard deviation
            buffer = 1e-5;
            sensitivity_x = 3 * sqrt(obj.variance_X);
            % sensitivity_y = 3 * sqrt(obj.variance_Y);
            sensitivity_v = 3 * sqrt(obj.variance_V);

            % Avoid division by zero by adding a small buffer
            normalized_error_x = error_x / (sensitivity_x + buffer);
            % normalized_error_y = error_y / (sensitivity_y + buffer);
            normalized_error_v = error_v / (sensitivity_v + buffer);

            % Use exponential decay: error=0→1.0, error=∞→0.0
            cred_x = exp(-normalized_error_x);
            % cred_y = exp(-normalized_error_y);
            cred_v = exp(-normalized_error_v);


            % Weighted credibility score
            % cred = wx * cred_x + wy * cred_y + wv * cred_v;
            cred = wx * cred_x + wv * cred_v;


            % Compute mean credibility score and trustworthiness
            cred_mean = mean(cred);
            trust = cred_mean >= threshold;


            % Display credibility values
            % disp(['Cred_mean =', num2str(cred_mean)]);

            % save
            obj.cred.(vehicleID) = struct('t', obj.t(end), 'credibility_score', cred_mean, 'trustworthy', trust);
        end

        function store_cred(obj, trackedVehicle)
            vehicleID = sprintf('v%d', trackedVehicle.ID);
            if ~isfield(obj.cred, vehicleID)
                obj.cred.(vehicleID) = struct('t', 0, 'credibility_score', 1, 'trustworthy', 1);
                obj.str_cred.(vehicleID) = struct('t', 0, 'credibility_score', 1, 'trustworthy', 1);
            end

            if ~isfield(obj.str_cred, vehicleID)
                obj.str_cred.(vehicleID) = struct('t', 0, 'credibility_score', 1, 'trustworthy', 1);
            end

            cred_data = obj.cred.(vehicleID);

            % extract data
            cred = cred_data.credibility_score;
            trust = cred_data.trustworthy;

            % Append to historical data
            obj.str_cred.(vehicleID).t = [obj.str_cred.(vehicleID).t; obj.t(end) + obj.dt];
            obj.str_cred.(vehicleID).credibility_score = [obj.str_cred.(vehicleID).credibility_score; cred];
            obj.str_cred.(vehicleID).trustworthy = [obj.str_cred.(vehicleID).trustworthy; trust];
        end
    end
end
