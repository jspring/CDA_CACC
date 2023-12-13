/*	Pixell configuration constants, prototypes, etc.
**
*/

#define ID_SERVER_PORT	48620
#define CONFIG_PORT		48630
#define DATA_PORT 		48630

//Identification server requests
#define IDT_REQUEST_IDENTIFY	0X0011

//Request header
typedef struct {
	unsigned char server_protocol_version[2];
	unsigned char request_code[2];
	unsigned char request_size[4];
} IS_PACKED request_header_t;

//Answer header
typedef struct{
	unsigned char server_protocol_version[2];
	unsigned char answer_code[2];
	unsigned char answer_size[4];
	unsigned char reserved_bytes[8];
} IS_PACKED answer_header_t;

//Element header
typedef struct{
	unsigned char element_code[2];
	unsigned char element_count[2];
	unsigned char element_size[4];
} IS_PACKED element_header_t;
	
//Answer codes
#define OK 			0x0000 //Request properly handled
#define	ERROR 			0x0001 //General error
#define	FLASH_ERROR		0x0002 //Error related to flash memory read or write
#define	HARDWARE_ERROR 		0x0003 //Error caused by abnormal hardware operation
#define	INVALID_DATA 		0x0004 //The request contained invalid data.
#define	INVALID_REQUEST 	0x0005 //The request is invalid (not defined/supported).
#define	PROTOCOL_ERROR 		0x0006 //Error in the protocol (invalid header or packet structure)
#define	UNSUPPORTED_PROTOCOL	0x0007 //Unsupported protocol version (as specified in the request header)
#define	LIMITED_MODE 		0x0008 //The device is in a limited mode of operation and cannot service this request.
#define	OUTPUT_SIZE_TOO_LONG 	0x0009 //The constructed response is larger than the maximum supported packet size.
#define	MISSING_ELEMENT		0x000A //The request is missing a required element.
//Device-specific ≥ 0x0100

