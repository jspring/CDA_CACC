#include <hmi_server.h>


void Init_database(int argc, char const *argv[]){
	char hostname[MAXHOSTNAMELEN];
	char *domain; /// on Linux sets DB q-file directory
	int xport = COMM_OS_XPORT;      /// value set correctly in sys_os.h

	get_local_name(hostname, MAXHOSTNAMELEN);

	int opt;
	time_sync_requested = 0;
	ref_vehicle_id = -1;
	while((opt = getopt(argc, argv, "e:r:t")) != -1){
		switch(opt) {
			case 'e':
				ego_id = atoi(optarg);
				if(ego_id < 0 || ego_id > 2){
					printf("Ego ID must be between 0 and 2 (0=Accord/Leaf, 1=Prius, 2=Taurus/Camry");
					exit(EXIT_FAILURE);
				}
				break;
			case 'r':
				ref_vehicle_id = atoi(optarg);
				if(ref_vehicle_id < 0 || ref_vehicle_id > 2){
					printf("Ref vehicle ID must be between 0 and 2");
					exit(EXIT_FAILURE);
				}
				break;
			case 't':
				time_sync_requested = 1;
				break;
			default:
				printf("Not recognized: %s %s\n", argv[0], opt);
				exit(EXIT_FAILURE);
				break;
		}
	}
	if(time_sync_requested && ref_vehicle_id == -1){
		printf("hmi_server: Reference vehicle for time sync must be specified, exiting. \n");
		exit(EXIT_FAILURE);
	}

	if ( (pclt_hmi = db_list_init(argv[0], hostname, domain, xport, db_vars_list_hmi, NUM_DB_VARS_HMI, NULL, 0)) == NULL) {
//	if ( (pclt_hmi = db_list_init(argv[0], hostname, domain, xport, NULL, 0, NULL, 0)) == NULL) {
		exit(EXIT_FAILURE);
	}
	if (setjmp(exit_env_hmi) != 0) {
		db_list_done(pclt_hmi, (db_id_t *)NULL, (int)0, (int *)NULL, (int)0);
		exit(EXIT_SUCCESS);
	} else
		sig_ign(sig_list_hmi, sig_hand_hmi);

	sync_flag = 0;
	sync_button_state = 0;
	start_button_state = 0;
	Write_sync_flag_to_database(&sync_flag);
	printf("Done database init\n");
}

int main(int argc, char const *argv[]){

	Init_database(argc, argv);

	int server_fd, new_socket, valread;
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);
	char buffer[1024] = {0};

	// Creating socket file descriptor
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)	{
		perror("socket failed");
		exit(EXIT_FAILURE);
	}
	//  | SO_REUSEPORT
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)))	{
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}

	address.sin_family = AF_INET;
	switch(ego_id){
		case(LEAF):
			address.sin_addr.s_addr = inet_addr("172.16.0.121"); // LEAF
			break;
		case(PRIUS):
			address.sin_addr.s_addr = inet_addr("172.16.0.127"); // PRIUS
			break;
		case(CAMRY):
			address.sin_addr.s_addr = inet_addr("172.16.0.128"); // CAMRY
			break;
	}
	printf("ip_address %#X\n", address.sin_addr.s_addr);
	address.sin_port = htons( PORT );

	if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) {
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	if (listen(server_fd, 3) < 0) {
		perror("listen failed");
		exit(EXIT_FAILURE);
	}
	if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0) {
		perror("accept failed");
		exit(EXIT_FAILURE);
	}

	printf("Server connected \n");

	while(time_sync_requested){
		usleep(10000);
		(!Receive_sync_data_from_hmi(new_socket)) ? perror("Failed to get sync message form HMI\n") : printf("hmi_server: Received sync button: %d, Start button: %d \n", sync_button_state, start_button_state);
		if(ego_id == ref_vehicle_id){
			if(!sync_button_state){
				Send_sync_data_to_hmi(new_socket, sync_flag, start_button_state);
				continue;
			}
			time_sync_requested = !start_button_state;
			sync_flag = 1;
			Send_sync_data_to_hmi(new_socket, sync_flag, start_button_state);

		} else {
			if(!sync_button_state){
				Send_sync_data_to_hmi(new_socket, sync_flag, start_button_state);
				continue;
			}
			if(!Update_local_machine_time_with_ref_vehicle()){
				Send_sync_data_to_hmi(new_socket, sync_flag, start_button_state);
				continue;
			}
			time_sync_requested = !start_button_state;
			Send_sync_data_to_hmi(new_socket, sync_flag, start_button_state);
		}
	}


	sync_flag = 1;
	Write_sync_flag_to_database(&sync_flag);
	printf("hmi_server: Sync flag %d written \n", sync_flag);

	////////////////////////////////////////////
	db_data = (tx_data*)  malloc(sizeof(tx_data));
	hmi_data = (rx_data*) malloc(sizeof(rx_data));
	db_data->current_control_mode = 0;

	for(;;){
		if(Receive_data_from_hmi(new_socket) != 1)
			break;
		if(Send_data_to_hmi(new_socket) != 1)
			break;
		if(Read_control_variables_from_database() != 1)
			break;
		if(Write_user_variables_to_database() != 1)
			break;

		usleep(20000);
	}

	EndTasks(new_socket);
	return 1;
}

int EndTasks(int server_socket){
	close(server_socket);
	printf("\n Closing the connection with client . \n");
	return 1;
}

///////////////////////////////////////////////////////////////////////

int Receive_sync_data_from_hmi(int socket){
	char rx_char[BUFF_SIZE] = {0};
	ssize_t bytes = read( socket, rx_char, BUFF_SIZE);
	if(bytes <= 0)	{
		printf(" Recv error");
		return 0;
	}
	else {
		rx_char[bytes] = '\0';
	}
	char* token = strtok(rx_char, " ");
	int rx_integers[2];
	int position = 0;
	while (token != NULL) {
		if (position < 2){
			rx_integers[position] = atoi(token);
			position++;
		}
		token = strtok(NULL, " ");
	}
	sync_button_state = rx_integers[0];
	start_button_state = rx_integers[1];
	return 1;
}

int Send_sync_data_to_hmi(int socket, int sync, int start){
	char tx_char[BUFF_SIZE];
	sprintf(tx_char, "%d %d", sync, start);
	send(socket , tx_char , strlen(tx_char) , 0 );
	return 1;
}

int Write_sync_flag_to_database(int* flag){
	db_clt_write(pclt_hmi, CACC_CONTROL_ENABLE, sizeof(int), flag);
	return 1;
}

int Update_local_machine_time_with_ref_vehicle(){
	db_clt_read(pclt_hmi, DB_COMM_LEAD_TRK_VAR, sizeof(veh_comm_packet_t), &comm_pkt_1);
	db_clt_read(pclt_hmi, DB_COMM_SECOND_TRK_VAR, sizeof(veh_comm_packet_t), &comm_pkt_2);
	db_clt_read(pclt_hmi, DB_COMM_THIRD_TRK_VAR, sizeof(veh_comm_packet_t), &comm_pkt_3);
	int match = 0;
	if(!strcmp(VEHICLE_NAMES[ref_vehicle_id], comm_pkt_1.object_id)){
		ref_veh_pkt = comm_pkt_1;
		match = 1;
	}
	if(!strcmp(VEHICLE_NAMES[ref_vehicle_id], comm_pkt_2.object_id)){
		ref_veh_pkt = comm_pkt_2;
		match = 1;
	}
	if(!strcmp(VEHICLE_NAMES[ref_vehicle_id], comm_pkt_3.object_id)){
		ref_veh_pkt = comm_pkt_3;
		match = 1;
	}
	if(!match){
		printf("hmi_server:                                                      %s packet has not been detected \n", VEHICLE_NAMES[ref_vehicle_id]);
		return 0;
	}

	struct timespec oldtime, newtime;

	if( (clock_gettime(CLOCK_REALTIME, &oldtime)) == -1){
	    perror("hmi_server: clock_gettime \n");
	    return 0;
	}

	newtime = adjust_tm(oldtime, ref_veh_pkt.ts);

	if( (clock_settime(CLOCK_REALTIME, &newtime )) == 1){
	    perror("hmi_server: clock_settime \n");
	    return 0;
	}
	printf("hmi_server: Setting time = %d %d  pkt: hour %d:%d:%d.%d \n", newtime.tv_sec, newtime.tv_nsec, ref_veh_pkt.ts.hour, ref_veh_pkt.ts.min, ref_veh_pkt.ts.sec, ref_veh_pkt.ts.millisec);

	sync_flag = 1;
	return 1;
}

////////////////////////////////////////////////////////////////////////

int Receive_data_from_hmi(int socket){
	char rx_char[BUFF_SIZE] = {0};
	ssize_t bytes = read( socket, rx_char, BUFF_SIZE);
	if(bytes <= 0)	{
		printf(" Recv error");
	    return 0;
	}
	else {
		rx_char[bytes] = '\0';
	}
	char* token = strtok(rx_char, " ");
	int rx_integers[NB_DATA_FROM_HMI];
	int position = 0;
	while (token != NULL) {
		if (position < NB_DATA_FROM_HMI){
			rx_integers[position] = atoi(token);
			position++;
		}
		token = strtok(NULL, " ");
	}
	hmi_data->desired_control_mode = rx_integers[0];
	hmi_data->gap_choice = rx_integers[1];

	switch(hmi_data->gap_choice){
		case 1:
			hmi_data->desired_time_gap = ((db_data->current_control_mode == CACC) || (db_data->current_control_mode == ACC_2_CACC)) ? 0.3 : 0.9;
			break;
		case 2:
			hmi_data->desired_time_gap = ((db_data->current_control_mode == CACC) || (db_data->current_control_mode == ACC_2_CACC)) ? 0.5 : 1.1;
			break;
		case 3:
			hmi_data->desired_time_gap = ((db_data->current_control_mode == CACC) || (db_data->current_control_mode == ACC_2_CACC))? 0.7 : 1.3;
			break;
		case 4:
			hmi_data->desired_time_gap = ((db_data->current_control_mode == CACC) || (db_data->current_control_mode == ACC_2_CACC)) ? 0.9 : 1.5;
			break;
		case 5:
			hmi_data->desired_time_gap = ((db_data->current_control_mode == CACC) || (db_data->current_control_mode == ACC_2_CACC)) ? 1.1 : 1.7;
			break;
		default:
			hmi_data->desired_time_gap = (float) rx_integers[2]/100000;
			break;
	}

	hmi_data->desired_cruise_speed = (float) rx_integers[3]/100000;
	hmi_data->performance_factor = (float) rx_integers[4]/100000;

	return 1;
}

int Send_data_to_hmi(int socket){
	char tx_char[BUFF_SIZE];
	sprintf(tx_char, "%d %d %d %d %d %d %d %d %d %d %d %d %d",
											db_data->my_pip,
											db_data->target_vehicle_valid,
											db_data->current_control_mode,
											db_data->v2v_available,
											(int) (db_data->ego_speed*100000),
											(int) (db_data->measured_time_gap*100000),
											(int) (db_data->ACC_set_time_gap*100000),
											(int) (db_data->CACC_set_time_gap*100000),
											(int) (db_data->veh_1_speed*100000),
											(int) (db_data->veh_2_speed*100000),
											(int) (db_data->veh_3_speed*100000),
											(int) (db_data->timestamp*100000),
											(int) (db_data->sys_status*100000));
//	printf("Send_data_to_hmi: my_pip %d curr_ctl_mode %d v2v_avail %d ego_speed %d meas_time_gap %d\n",
//											db_data->my_pip,
//											db_data->current_control_mode,
//											db_data->v2v_available,
//											(int) (db_data->ego_speed*100000),
//											(int) (db_data->measured_time_gap*100000)
//											(int) (db_data->ACC_set_time_gap*100000),
//											(int) (db_data->CACC_set_time_gap*100000),
//											(int) (db_data->veh_1_speed*100000),
//											(int) (db_data->veh_2_speed*100000),
//											(int) (db_data->veh_3_speed*100000),
//											(int) (db_data->timestamp*100000),
//											(int) (db_data->sys_status*100000)
//											);
	send(socket , tx_char , strlen(tx_char) , 0 );
	return 1;
}

int Read_control_variables_from_database(){
	db_clt_read(pclt_hmi, DB_2_HMI_DATA, sizeof(tx_data), db_data);
	return 1;
}

int Write_user_variables_to_database(){
	db_clt_write(pclt_hmi, HMI_2_DB_DATA, sizeof(rx_data), hmi_data);
	return 1;
}

