
#ifndef M_BUS_RECEIVER
#define M_BUS_RECEIVER

//#define REQ_UD2 100
#define C_Field 0x5B/7B  //C-field in REQ_UD2
#define A_field 0xAA     //Adresse field of the m-bus meater. We have to send this first in order to initialise it with the correct adress.
#define respone 0xE5     //Response from initialisation
#define start_short_frame 0x10
#define start_long_frame 0x68
#define stop_short_long_frame 0x16



#endif //M_BUS_RECEIVER
