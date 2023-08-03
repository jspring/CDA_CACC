function [st] = ParseData(file)
    tab = readtable(file,'Delimiter',' ');
    table = table2array(tab);
    st.hour = table(:,1);
    st.minute = table(:,2);
    st.second = table(:,3);
end
