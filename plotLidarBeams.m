function plotLidarBeams(thetaDeg, phiDeg, figTitle)
% plotLidarBeams plots lidar beam directions in 3D.
%
% Convention:
% v_LOS = u*cos(theta)*sin(phi) ...
%       + v*cos(theta)*cos(phi) ...
%       + w*sin(theta)
%
% Therefore the beam unit vector is:
% e = [cos(theta)*sin(phi),
%      cos(theta)*cos(phi),
%      sin(theta)]

    thetaDeg = thetaDeg(:);
    phiDeg   = phiDeg(:);

    nBeams = numel(thetaDeg);

    theta = deg2rad(thetaDeg);
    phi   = deg2rad(phiDeg);

    % Beam direction cosines
    x = cos(theta).*sin(phi);   % u direction
    y = cos(theta).*cos(phi);   % v direction
    z = sin(theta);             % w direction

    figure('Color','w');
    hold on; grid on; box on;

    % Ground projection circle
    t = linspace(0, 2*pi, 361);
    plot3(cos(t), sin(t), zeros(size(t)), 'k:', 'LineWidth', 1.0);

    % Vertical axis
    plot3([0 0], [0 0], [0 1.15], 'k-', 'LineWidth', 1.0);

    % Horizontal axes
    plot3([-1.15 1.15], [0 0], [0 0], 'k-', 'LineWidth', 0.8);
    plot3([0 0], [-1.15 1.15], [0 0], 'k-', 'LineWidth', 0.8);

    % Lidar position
    plot3(0, 0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);

    % Use one color per beam
    colors = lines(nBeams);

    for i = 1:nBeams

        quiver3( ...
            0, 0, 0, ...
            x(i), y(i), z(i), ...
            0, ...
            'LineWidth', 2.2, ...
            'MaxHeadSize', 0.18, ...
            'Color', colors(i,:) );

        plot3( ...
            x(i), y(i), z(i), ...
            'o', ...
            'MarkerSize', 7, ...
            'MarkerFaceColor', colors(i,:), ...
            'MarkerEdgeColor', colors(i,:) );

        % Projection of the beam onto horizontal plane
        plot3( ...
            [0 x(i)], [0 y(i)], [0 0], ...
            '--', ...
            'LineWidth', 0.9, ...
            'Color', colors(i,:) );

        % Beam label
        labelText = sprintf('B%d: \\theta=%g^\\circ, \\phi=%g^\\circ', ...
            i, thetaDeg(i), phiDeg(i));

        text( ...
            1.08*x(i), ...
            1.08*y(i), ...
            1.08*z(i), ...
            labelText, ...
            'FontSize', 9, ...
            'Color', colors(i,:) );
    end

    xlabel('u direction');
    ylabel('v direction');
    zlabel('w direction');

    title(figTitle, 'Interpreter', 'none');

    axis equal;
    xlim([-1.2 1.2]);
    ylim([-1.2 1.2]);
    zlim([0 1.2]);

    view(38, 24);

end