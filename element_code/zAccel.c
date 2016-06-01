	bmc150_accel_t accel = {0};
	bmc150_read_accel(&accel);
	
	return accel.z;