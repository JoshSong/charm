public class dynamixel_jni
{
	public static final int MAXNUM_TXPARAM     = 150;
    public static final int MAXNUM_RXPARAM     = 60;

    public static final int BROADCAST_ID       = 254;
	
	public static final int INST_PING          = 1;
    public static final int INST_READ          = 2;
    public static final int INST_WRITE         = 3;
    public static final int INST_REG_WRITE     = 4;
    public static final int INST_ACTION        = 5;
    public static final int INST_RESET         = 6;
    public static final int INST_SYNC_WRITE    = 131;

    public static final int ERRBIT_VOLTAGE     = 1;
    public static final int ERRBIT_ANGLE       = 2;
    public static final int ERRBIT_OVERHEAT    = 4;
    public static final int ERRBIT_RANGE       = 8;
    public static final int ERRBIT_CHECKSUM    = 16;
    public static final int ERRBIT_OVERLOAD    = 32;
    public static final int ERRBIT_INSTRUCTION = 64;

    public static final int COMM_TXSUCCESS     = 0;
    public static final int COMM_RXSUCCESS     = 1;
    public static final int COMM_TXFAIL        = 2;
    public static final int COMM_RXFAIL        = 3;
    public static final int COMM_TXERROR       = 4;
    public static final int COMM_RXWAITING     = 5;
    public static final int COMM_RXTIMEOUT     = 6;
	public static final int COMM_RXCORRUPT		= 7;


	// device control methods
	public static native int dxl_initialize();
	public static native void dxl_terminate();
	public static native int dxl_get_baud();
	public static native void dxl_set_baud( int baudnum );
	public static native int dxl_get_result();

	// device control methods
	public static native void dxl_tx_packet();
	public static native void dxl_rx_packet();
	public static native void dxl_txrx_packet();

	// set/get packet methods
	public static native void dxl_set_txpacket_id( int id );
	public static native void dxl_set_txpacket_instruction( int instruction );
	public static native void dxl_set_txpacket_parameter( int index, int value );
	public static native void dxl_set_txpacket_length( int length );
	public static native int dxl_get_rxpacket_error( int errbit );
	public static native int dxl_get_rxpacket_length();
	public static native int dxl_get_rxpacket_parameter( int index );

	// utility for value
	public static native int dxl_makeword( int lowbyte, int highbyte );
	public static native int dxl_get_lowbyte( int word );
	public static native int dxl_get_highbyte( int word );

	// high communication methods
	public static native void dxl_ping( int id );
	public static native int dxl_read_byte( int id, int address );
	public static native void dxl_write_byte( int id, int address, int value );
	public static native int dxl_read_word( int id, int address );
	public static native void dxl_write_word( int id, int address, int value );
	
	static
	{
		System.loadLibrary("dynamixel_jni");
	}
}
