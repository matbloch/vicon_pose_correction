%% main file

clc;
clear all;

%% settings
nr_fits = 1000;  % number of fitting attempts for each outlier ratio
f_scale = 3;    % outlier ratios



% settings of the circle fitting
fitting_params.tau = 0.1;  % inlier distance treshold
fitting_params.ex_search = 0;





%% start 
for k = 1:3 % for all outlier ratios
    
    % specify outlier ratio
    dist_in.r = r(k);
    
    % collect # inliers in vector
    nr_inliers = zeros(1,nr_fits);
    max_nr_inliers = 0;

    for i=1:nr_fits

        %%% 1.1 generate point distribution
        circle_dist = load_data(dist_in);
		
		
		
		
        max_iter = ceil(log(1-0.99)/log(1-(1-dist_in.r)^3));    % r: outlier ratio

        fitting_params.points = circle_dist.points;
        fitting_params.max_iter = max_iter;

        %%% 1.2 do fitting
        est = fitCircle(fitting_params);
        nr_inliers(i) = est.nr_inliers;
        if est.nr_inliers > max_nr_inliers
            best_est = est;
            best_est_dist = circle_dist;
            max_nr_inliers = est.nr_inliers;
        end

    end

end
