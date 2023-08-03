/* FILE: settime2gps.c
 *  Process to read the GPRMC GPS message, and use its date and time to
 *  set the system clock.
 */

#include <db_include.h>
#include "path_gps_lib.h"
#include "long_comm.h"

jmp_buf env;

int main( int argc, char *argv[] )
{
	int opt;
	int sync_truck_number = 1;
	veh_comm_packet_t comm_pkt_truck[3] = {{0}};
	int truck_ms[3] = {-1000000};
	struct timespec tspec;
	struct timespec new_tspec;
	struct tm tm_struct;
	int sec_at_midnight;
	db_clt_typ *pclt;               /// data bucket pointer
	char *domain=DEFAULT_SERVICE;
	char hostname[MAXHOSTNAMELEN+1];
	int xport = COMM_OS_XPORT;

        /** Get command line options */
        while ((opt = getopt(argc, argv, "t:")) != -1) {
                switch (opt) {
                case 't':
                        sync_truck_number = atoi(optarg);
                        break;
		default:
			printf("Usage: %s -t <sync_truck_number, def. 1>\n", argv[0]);
			break;
		}
	}

	sync_truck_number--; //Change from truck number to index

	get_local_name(hostname, MAXHOSTNAMELEN);
	pclt = db_list_init(argv[0], hostname, domain, xport, NULL, 0, NULL, 0);

	db_clt_read(pclt, DB_COMM_LEAD_TRK_VAR, sizeof(veh_comm_packet_t), &comm_pkt_truck[0]);
	db_clt_read(pclt, DB_COMM_SECOND_TRK_VAR, sizeof(veh_comm_packet_t), &comm_pkt_truck[1]);
	db_clt_read(pclt, DB_COMM_THIRD_TRK_VAR, sizeof(veh_comm_packet_t), &comm_pkt_truck[2]);
	truck_ms[0] = TS_TO_MS(&comm_pkt_truck[0].ts);
	truck_ms[1] = TS_TO_MS(&comm_pkt_truck[1].ts);
	truck_ms[2] = TS_TO_MS(&comm_pkt_truck[2].ts);
	printf("Time difference (ms): truck1-truck2 %d truck1-truck3 %d truck2-truck3 %d\n",
		truck_ms[0] - truck_ms[1],
		truck_ms[0] - truck_ms[2],
		truck_ms[1] - truck_ms[2]
	);

	clock_gettime(CLOCK_REALTIME, &tspec);
	localtime_r(&tspec.tv_sec, &tm_struct);
	tm_struct.tm_sec = 0;
	tm_struct.tm_min = 0;
	tm_struct.tm_hour = 0;
	sec_at_midnight = mktime(&tm_struct);

	if( (truck_ms[sync_truck_number] > 0) && (truck_ms[sync_truck_number] < 86400000) ){ //was initialized to 0, unlikely to be precisely 0
		new_tspec.tv_sec = sec_at_midnight + (truck_ms[sync_truck_number] / 1000);
		new_tspec.tv_nsec = (truck_ms[sync_truck_number] % 1000) * 1000000;
		clock_settime(CLOCK_REALTIME, &new_tspec);
	}
	else {
		printf("Error in reading vehicle %d's timestamp; it's set to %d ms after midnight\n", sync_truck_number + 1, truck_ms[sync_truck_number]);
		exit(EXIT_FAILURE);
	}

	return 0;
}
