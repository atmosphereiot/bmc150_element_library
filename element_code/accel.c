	bmc150_accel_t accel = {0};
	bmc150_read_accel(&accel);
	
	char valueBuffer[32];
	
	sprintf(valueBuffer, "[%d,%d,%d]", accel.x, accel.y, accel.z);
	
	return valueBuffer;