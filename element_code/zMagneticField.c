
	bmc150_mag_t mag = {0};
	bmc150_read_mag(&mag);
	
	return mag.z;