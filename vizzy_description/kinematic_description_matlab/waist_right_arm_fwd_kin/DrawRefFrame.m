function DrawRefFrame(G,num)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Draw reference frames attached to the centers of mass
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
length = 10 + 1.5^num;

Origin1 = G*[0 0 0 1]';

x_axis_1 = G*[length 0 0 1]';
h = quiver3(Origin1(1), Origin1(2), Origin1(3), x_axis_1(1) - Origin1(1), x_axis_1(2) - Origin1(2), x_axis_1(3) - Origin1(3), 0);
set(h, 'Color', 'r', 'LineWidth', 1/4*num + 1, 'MaxHeadSize', 4, 'ShowArrowHead', 'off')
set(h, 'ShowArrowHead', 'on')


y_axis_1 = G*[0 length 0 1]';
h = quiver3(Origin1(1), Origin1(2), Origin1(3), y_axis_1(1) - Origin1(1), y_axis_1(2) - Origin1(2), y_axis_1(3) - Origin1(3), 0);
set(h, 'Color', 'g', 'LineWidth', 1/4*num+ 1, 'MaxHeadSize', 4, 'ShowArrowHead', 'off')
set(h, 'ShowArrowHead', 'on')


z_axis_1 = G*[0 0 length 1]';
h = quiver3(Origin1(1), Origin1(2), Origin1(3), z_axis_1(1) - Origin1(1), z_axis_1(2) - Origin1(2), z_axis_1(3) - Origin1(3), 0);
set(h, 'Color', 'b', 'LineWidth', 1/4*num + 1, 'MaxHeadSize', 4, 'ShowArrowHead', 'off')
set(h, 'ShowArrowHead', 'on')

%h = text(x_axis_1(1), x_axis_1(2), x_axis_1(3), strcat('x_{', num2str(num), '}'));
%set(h, 'FontSize', 16)
%h = text(y_axis_1(1), y_axis_1(2), y_axis_1(3), strcat('y_{', num2str(num), '}'));
%set(h, 'FontSize', 16)
h = text(z_axis_1(1), z_axis_1(2), z_axis_1(3), strcat('z_{', num2str(num), '}'));
set(h, 'FontSize', num + 10)
