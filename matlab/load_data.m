function [ out ] = load_data( in )


    out.inliers = [x;y];
    
    % combine in and outliers
    out.points = [out.inliers, out.outliers];

end

