[X,Y,Z] = sphere;
x = linspace(-1, 1, 75);
y = -x.^2;
figure(1)


for k2 = 1:2
    for k1 = 1:length(x)
        hold on
        surf(X+y(k1), Y+sin(2*pi*x(k1)), Z+cos(2*pi*x(k1)))
        axis([-5  5    -5  5    -5  5])
        axis square
        view([-20  20])

        refreshdata
        drawnow
    end
end