lipmz_it = load('../tmp/com');
lipmz_d = load('../tmp/zmp_d');
% lipmz_it = load('../tmp/lipmz_it');
% lipmz_d = load('../tmp/lipmz_d');

figure
hold on
plot(lipmz_it(:,3), lipmz_it(:,4), 'r');
plot(lipmz_it(:,12), lipmz_it(:,13), 'c');
plot(lipmz_d(:,3), lipmz_d(:,4), 'b');
axis equal;

figure
subplot(3,1,1)
hold on
plot(lipmz_d(:,3), 'b');
plot(lipmz_it(:,12), 'c');
plot(lipmz_it(:,3), 'r');

subplot(3,1,2)
hold on
plot(lipmz_d(:,4), 'b');
plot(lipmz_it(:,13), 'c');
plot(lipmz_it(:,4), 'r');

subplot(3,1,3)
hold on
plot(lipmz_d(:,5), 'b');
plot(lipmz_it(:,5), 'r');
