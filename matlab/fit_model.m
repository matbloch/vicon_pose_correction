function [ fit ] = fitCircle( in )

    % in.points: sample points: matrix of column vectors
    % in.tau: threshold
    % in.ex_search: exhausting search 0 or 1
    
    ex_search = 0;
    
    % select method
    if isfield(in,'ex_search') && in.ex_search
        ex_search = 1;
    end
    
    fit = search_circle(in);
    
    %% RANSAC: random search
    function [ out ] = search_circle(in)

        points = in.points;
        tau = in.tau;
        
        out.x = 0;
        out.y = 0;
        out.r = 0;
        out.nr_inliers = 0;
        out.inliers = [];
        out.fitting_points = zeros(1,3); % 3 points to fit circle
        N = size(points,2); % # sample points
        
        out.points = in.points; % return sample points

        % calc number of iterations
        if ex_search
            % disp('applying exhaustive search.');
            % generate set of all possible point combinations
            indices_list = nchoosek(1:N,3);  % select column/point indices
            max_iter = size(indices_list, 2);
        else
            if isfield(in,'max_iter')
                max_iter = in.max_iter;
            else
                if isfield(in,'r')   % outlier ratio of the distribution
                    out = random_search(tau, points);
                    p = 0.99;
                    s = 3; % circle fitting
                    max_iter = ceil(log(1-p)/log(1-(1-r)^s));    % r: outlier ratio
                else
                    disp('Please specify the number of iterations.');
                    return;
                end
            end
        end

        for it=1:max_iter  % it: iteration number

            % STEP 1: select 3 points
            if ex_search
                extr_indices = indices_list(it,:);
            else
                extr_indices = randi(N, 1, 3);  % select column/point indices
            end
            
            x = points(1,extr_indices);
            y = points(2,extr_indices);

            % STEP 2: fit the circle
            mr = (y(2)-y(1))/(x(2)-x(1));
            mt = (y(3)-y(2))/(x(3)-x(2));
            x_m = (mr*mt*(y(3)-y(1))+mr*(x(2)+x(3))-mt*(x(1)+x(2)))/(2*mr-mt);
            y_m = -1/mr*(x_m-(x(1)+x(2))/2)+(y(1)+y(2))/2;
            rad = sqrt((x(1)-x_m)^2+(y(1)-y_m)^2);

            % STEP 3: count the number of inliers
            nr_inliers = 0;
            inliers = [];

                % iterate over all points
                for i=1:size(points, 2)
                    
                    % skip if equals extracted point
                    if any(extr_indices==i)
                        continue;
                    end
                    
                    % calculate dist from circle center
                    dr = abs(norm(points(:,i)-[x_m;y_m])-rad);

                    if dr <= tau
                        % count as inlier
                        nr_inliers = nr_inliers + 1;
                        inliers = [inliers, i];   % store inlier index
                    end
                end


            % STEP 4: compare to previous solution & update
            if nr_inliers > out.nr_inliers
                out.nr_inliers = nr_inliers;
                out.inliers = [inliers,extr_indices];   % add the  fitting points to the inlieres
                out.fitting_points = extr_indices;
                out.x = x_m;
                out.y = y_m;
                out.r = rad;
            end
        end
        
        % get outliers from final inliers
        out.outliers = 1:size(points,2);
        out.outliers(out.inliers) = [];   % subtract inliers
        
    end

end

