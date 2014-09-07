#include <windows.h>
#include "dynamixel.h"
#include "dynamixel_jni.h"

#pragma comment(lib, "dynamixel.lib")


/*
 * Class:     dynamixel_jni
 * Method:    dxl_initialize
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1initialize(JNIEnv *env, jclass jcls)
{
	return (jint)dxl_initialize();
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_terminate
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1terminate(JNIEnv *env, jclass jcls)
{
	dxl_terminate();
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_get_baud
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1get_1baud(JNIEnv *env, jclass jcls)
{
	return (jint)dxl_get_baud();
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_set_baud
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1set_1baud(JNIEnv *env, jclass jcls, jint baudnum)
{
	dxl_set_baud( (int)baudnum );
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_get_result
 * Signature: ()Ldynamixel_jni/STATUS;
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1get_1result(JNIEnv *env, jclass jcls)
{
	return (jint)dxl_get_result();
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_tx_packet
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1tx_1packet(JNIEnv *env, jclass jcls)
{
	dxl_tx_packet();
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_rx_packet
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1rx_1packet(JNIEnv *env, jclass jcls)
{
	dxl_rx_packet();
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_txrx_packet
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1txrx_1packet(JNIEnv *env, jclass jcls)
{
	dxl_txrx_packet();
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_set_txpacket_id
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1set_1txpacket_1id(JNIEnv *env, jclass jcls, jint id)
{
	dxl_set_txpacket_id( (int)id );
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_set_txpacket_instruction
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1set_1txpacket_1instruction(JNIEnv *env, jclass jcls, jint instruction)
{
	dxl_set_txpacket_instruction( (int)instruction );
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_set_txpacket_parameter
 * Signature: (II)V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1set_1txpacket_1parameter(JNIEnv *env, jclass jcls, jint index, jint value)
{
	dxl_set_txpacket_parameter( (int)index, (int)value );
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_set_txpacket_length
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1set_1txpacket_1length(JNIEnv *env, jclass jcls, jint length)
{
	dxl_set_txpacket_length( (int)length );
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_get_rxpacket_error
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1get_1rxpacket_1error(JNIEnv *env, jclass jcls, jint errbit)
{
	return (jint)dxl_get_rxpacket_error( (int)errbit );
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_get_rxpacket_length
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1get_1rxpacket_1length(JNIEnv *env, jclass jcls)
{
	return (jint)dxl_get_rxpacket_length();
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_get_rxpacket_parameter
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1get_1rxpacket_1parameter(JNIEnv *env, jclass jcls, jint index)
{
	return (jint)dxl_get_rxpacket_parameter( (int)index );
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_makeword
 * Signature: (II)I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1makeword(JNIEnv *env, jclass jcls, jint lowbyte, jint highbyte)
{
	return (jint)dxl_makeword( (int)lowbyte, (int)highbyte );
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_get_lowbyte
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1get_1lowbyte(JNIEnv *env, jclass jcls, jint word)
{
	return (jint)dxl_get_lowbyte( (int)word );
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_get_highbyte
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1get_1highbyte(JNIEnv *env, jclass jcls, jint word)
{
	return (jint)dxl_get_highbyte( (int)word );
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_ping
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1ping(JNIEnv *env, jclass jcls, jint id)
{
	dxl_ping( (int)id );
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_read_byte
 * Signature: (II)I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1read_1byte(JNIEnv *env, jclass jcls, jint id, jint address)
{
	return (jint)dxl_read_byte( (int)id, (int)address );
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_write_byte
 * Signature: (III)V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1write_1byte(JNIEnv *env, jclass jcls, jint id, jint address, jint value)
{
	dxl_write_byte( (int)id, (int)address, (int)value );
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_read_word
 * Signature: (II)I
 */
JNIEXPORT jint JNICALL Java_dynamixel_1jni_dxl_1read_1word(JNIEnv *env, jclass jcls, jint id, jint address)
{
	return (jint)dxl_read_word( (int)id, (int)address );
}

/*
 * Class:     dynamixel_jni
 * Method:    dxl_write_word
 * Signature: (III)V
 */
JNIEXPORT void JNICALL Java_dynamixel_1jni_dxl_1write_1word(JNIEnv *env, jclass jcls, jint id, jint address, jint value)
{
	dxl_write_word( (int)id, (int)address, (int)value );
}
