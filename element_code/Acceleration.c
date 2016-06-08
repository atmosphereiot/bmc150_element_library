	BMC150_accel_t accel = {0};
	BMC150_read_accel(&accel);
	
	static char valueBuffer[32];
	
	sprintf(valueBuffer, "[%d,%d,%d]", accel.x, accel.y, accel.z);
	
	return valueBuffer;
