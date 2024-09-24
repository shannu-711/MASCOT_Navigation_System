data = readmatrix('coordinates.txt');
angle = readmatrix("angle.txt");

for i = 1:size(data)
    if(rem(i,2) == 0)
        figure(1)
        title('Marker 1')
        subplot(2, 2, 1)
        plot(round(i/2),data(i,1), 'o-')
        xlabel('Index')
        ylabel('X Coordinate')
        hold on
        subplot(2, 2, 2)
        plot(round(i/2),data(i,2), 'o-')
        xlabel('Index')
        ylabel('Y Coordinate')
        hold on
        subplot(2, 2, 3)
        plot(data(i,1), data(i,2), 'o-')
        xlabel('Index')
        ylabel('Y Coordinate')
        hold on
        subplot(2, 2, 4)
        plot(round(i/2), angle(i), 'o-')
        xlabel('Index')
        ylabel('Angle')
        hold on
    else
        figure(2)
        title('Marker 2')
        subplot(2, 2, 1)
        plot(round(i/2),data(i,1), 'o-')
        xlabel('Index')
        ylabel('X Coordinate')
        hold on
        subplot(2, 2, 2)
        plot(round(i/2),data(i,2), 'o-')
        xlabel('Index')
        ylabel('Y Coordinate')
        hold on
        subplot(2, 2, 3)
        plot(data(i,1), data(i,2), 'o-')
        xlabel('Index')
        ylabel('Y Coordinate')
        hold on
        subplot(2, 2, 4)
        plot(round(i/2), angle(i), 'o-')
        xlabel('Index')
        ylabel('Angle')
        hold on
    end
end

