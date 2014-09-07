/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class dynamixel_jni */

#ifndef _Included_dynamixel_jni
#define _Included_dynamixel_jni
#ifdef __cplusplus
extern "C" {
#endif
#undef dynamixel_jni_MAXNUM_TXPARAM
#define dynamixel_jni_MAXNUM_TXPARAM 150L
#undef dynamixel_jni_MAXNUM_RXPARAM
#define dynamixel_jni_MAXNUM_RXPARAM 60L
#undef dynamixel_jni_BROADCAST_ID
#define dynamixel_jni_BROADCAST_ID 254L
#undef dynamixel_jni_INST_PING
#define dynamixel_jni_INST_PING 1L
#undef dynamixel_jni_INST_READ
#define dynamixel_jni_INST_READ 2L
#undef dynamixel_jni_INST_WRITE
#define dynamixel_jni_INST_WRITE 3L
#undef dynamixel_jni_INST_REG_WRITE
#define dynamixel_jni_INST_REG_WRITE 4L
#undef dynamixel_jni_INST_ACTION
#define dynamixel_jni_INST_ACTION 5L
#undef dynamixel_jni_INST_RESET
#define dynamixel_jni_INST_RESET 6L
#undef dynamixel_jni_INST_SYNC_WRITE
#define dynamixel_jni_INST_SYNC_WRITE 131L
#undef dynamixel_jni_ERRBIT_VOLTAGE
#define dynamixel_jni_ERRBIT_VOLTAGE 1L
#undef dynamixel_jni_ERRBIT_ANGLE
#define dynamixel_jni_ERRBIT_ANGLE 2L
#undef dynamixel_jni_ERRBIT_OVERHEAT
#define dynamixel_jni_ERRBIT_OVERHEAT 4L
#undef dynamixel_jni_ERRBIT_RANGE
#define dynamixel_jni_ERRBIT_RANGE 8L
#undef dynamixel_jni_ERRBIT_CHECKSUM
#define dynamixel_jni_ERRBIT_CHECKSUM 16L
#undef dynamixel_jni_ERRBIT_OVERLOAD
#define dynamixel_jni_ERRBIT_OVERLOAD 32L
#undef dynamixel_jni_ERRBIT_INSTRUCTION
#define dynamixel_jni_ERRBIT_INSTRUCTION 64L
#undef dynamixel_jni_COMM_TXSUCCESS
#define dynamixel_jni_COMM_TXSUCCESS 0L
#undef dynamixel_jni_COMM_RXSUCCESS
#define dynamixel_jni_COMM_RXSUCCESS 1L
#undef dynamixel_jni_COMM_TXFAIL
#define dynamixel_jni_COMM_TXFAIL 2L
#undef dynamixel_jni_COMM_RXFAIL
#define dynamixel_jni_COMM_RXFAIL 3L
#undef dynamixel_jni_COMM_TXERROR
#define dynamixel_jni_COMM_TXERROR 4L
#undef dynamixel_jni_COMM_RXWAITING
#define dynamixel_jni_COMM_RXWAITING 5L
#undef dynamixel_jni_COMM_RXTIMEOUT
#define dynamixel_jni_COMM_RXTIMEOUT 6L
#undef dynamixel_jni_COMM_RXCORRUPT
#define dynamixel_jni_COMM_RXCORRUPT 7L
/*
 * Class:     dynamixel_jni
 * Method:    dxl_initialize
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1initialize
  (JNIEnv *, jclass);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_terminate
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1terminate
  (JNIEnv *, jclass);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_get_baud
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1get_1baud
  (JNIEnv *, jclass);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_set_baud
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1set_1baud
  (JNIEnv *, jclass, jint);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_get_result
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1get_1result
  (JNIEnv *, jclass);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_tx_packet
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1tx_1packet
  (JNIEnv *, jclass);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_rx_packet
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1rx_1packet
  (JNIEnv *, jclass);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_txrx_packet
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1txrx_1packet
  (JNIEnv *, jclass);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_set_txpacket_id
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1set_1txpacket_1id
  (JNIEnv *, jclass, jint);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_set_txpacket_instruction
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1set_1txpacket_1instruction
  (JNIEnv *, jclass, jint);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_set_txpacket_parameter
 * Signature: (II)V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1set_1txpacket_1parameter
  (JNIEnv *, jclass, jint, jint);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_set_txpacket_length
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1set_1txpacket_1length
  (JNIEnv *, jclass, jint);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_get_rxpacket_error
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1get_1rxpacket_1error
  (JNIEnv *, jclass, jint);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_get_rxpacket_length
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1get_1rxpacket_1length
  (JNIEnv *, jclass);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_get_rxpacket_parameter
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1get_1rxpacket_1parameter
  (JNIEnv *, jclass, jint);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_makeword
 * Signature: (II)I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1makeword
  (JNIEnv *, jclass, jint, jint);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_get_lowbyte
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1get_1lowbyte
  (JNIEnv *, jclass, jint);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_get_highbyte
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1get_1highbyte
  (JNIEnv *, jclass, jint);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_ping
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1ping
  (JNIEnv *, jclass, jint);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_read_byte
 * Signature: (II)I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1read_1byte
  (JNIEnv *, jclass, jint, jint);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_write_byte
 * Signature: (III)V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1write_1byte
  (JNIEnv *, jclass, jint, jint, jint);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_read_word
 * Signature: (II)I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1read_1word
  (JNIEnv *, jclass, jint, jint);

/*
 * Class:     dynamixel_jni
 * Method:    dxl_write_word
 * Signature: (III)V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1write_1word
  (JNIEnv *, jclass, jint, jint, jint);

#ifdef __cplusplus
}
#endif
#endif