#!/bin/bash
# This script creates tables files for preceding/following cars 2-5 from cacc_b-table.c.  
# That file serves as a template for creating all the other files, so it is the only
# one of them that is kept in the subversion repository. Edit cacc_b-table.c and then
# run this script before making everything.


cat cacc3_b-table.c | sed '{s/m56_m43/m56_m44/g}' >cacc3_c-table.c
cat cacc3_c-table.c | sed '{s/m56_m48/m56_m49/g}' >cacc3_c-table.c.mod
cat cacc3_c-table.c.mod | sed '{s/comm_specb/comm_specc/g}' >cacc3_c-table.c
cat cacc3_c-table.c | sed '{s/num_bfile_columns/num_cfile_columns/g}' >cacc3_c-table.c.mod
cat cacc3_c-table.c.mod | sed '{s/NUM_BFILE_COLUMNS/NUM_CFILE_COLUMNS/g}' >cacc3_c-table.c
rm cacc3_c-table.c.mod

cat cacc3_b-table.c | sed '{s/m56_m43/m56_m45/g}' >cacc3_e-table.c
cat cacc3_e-table.c | sed '{s/m56_m48/m56_m4A/g}' >cacc3_e-table.c.mod
cat cacc3_e-table.c.mod | sed '{s/comm_specb/comm_spece/g}' >cacc3_e-table.c
cat cacc3_e-table.c | sed '{s/num_bfile_columns/num_efile_columns/g}' >cacc3_e-table.c.mod
cat cacc3_e-table.c.mod | sed '{s/NUM_BFILE_COLUMNS/NUM_EFILE_COLUMNS/g}' >cacc3_e-table.c
rm cacc3_e-table.c.mod

cat cacc3_b-table.c | sed '{s/m56_m43/m56_m46/g}' >cacc3_f-table.c
cat cacc3_f-table.c | sed '{s/m56_m48/m56_m4B/g}' >cacc3_f-table.c.mod
cat cacc3_f-table.c.mod | sed '{s/comm_specb/comm_specf/g}' >cacc3_f-table.c
cat cacc3_f-table.c | sed '{s/num_bfile_columns/num_ffile_columns/g}' >cacc3_f-table.c.mod
cat cacc3_f-table.c.mod | sed '{s/NUM_BFILE_COLUMNS/NUM_FFILE_COLUMNS/g}' >cacc3_f-table.c
rm cacc3_f-table.c.mod

cat cacc3_b-table.c | sed '{s/m56_m43/m56_m47/g}' >cacc3_g-table.c
cat cacc3_g-table.c | sed '{s/m56_m48/m56_m4C/g}' >cacc3_g-table.c.mod
cat cacc3_g-table.c.mod | sed '{s/comm_specb/comm_specg/g}' >cacc3_g-table.c
cat cacc3_g-table.c | sed '{s/num_bfile_columns/num_gfile_columns/g}' >cacc3_g-table.c.mod
cat cacc3_g-table.c.mod | sed '{s/NUM_BFILE_COLUMNS/NUM_GFILE_COLUMNS/g}' >cacc3_g-table.c
rm cacc3_g-table.c.mod

