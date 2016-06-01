	BMC150_mag_t mag = {0};
	BMC150_read_mag(&mag);
	
	char valueBuffer[32];
	
	sprintf(valueBuffer, "[%d,%d,%d]", mag.x, mag.y, mag.z);
	
	return valueBuffer;
