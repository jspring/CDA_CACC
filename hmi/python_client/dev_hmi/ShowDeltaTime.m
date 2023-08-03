clc
clear 
close all
file='control_test_taur_log_10032020__11_07_17.dat';
st = ParseData(file);
t=st.time;
plot(st.i_fuel_rate(2:end));
