clc
clear 
close all
file='control_test_camr_log_10032020__11_07_17.dat';
st = ParseData(file);
% t = (st.hour * 3600) + (st.minute * 60) + st.second;
t=st.time
plot(t(2:end), t(2:end)-t(1:end-1));
