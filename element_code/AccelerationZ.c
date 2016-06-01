	BMC150_accel_t accel = {0};
	BMC150_read_accel(&accel);
	
	return accel.z;
