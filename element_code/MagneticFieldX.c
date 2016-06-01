
	BMC150_mag_t mag = {0};
	BMC150_read_mag(&mag);
	
	return mag.x;
