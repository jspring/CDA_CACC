
#include "asn_application.h"
#define ASN_EMIT_DEBUG 1
#include "asn_internal.h"       /* for _ASN_DEFAULT_STACK_MAX */
#include "asn_SEQUENCE_OF.h"
#include "MessageFrame.h"
#include "long_comm.h"

//extern int vehcomm2BSM(MessageFrame_t *BSMCACC, veh_comm_packet_t *comm_pkt);
asn_enc_rval_t vehcomm2BSM(char * buffer, size_t buffer_size, veh_comm_packet_t *comm_pkt, int verbose);
extern int BSM2vehcomm(MessageFrame_t *BSMCACC, veh_comm_packet_t *comm_pkt);
extern int print_comm_packet(veh_comm_packet_t *comm_pkt);
