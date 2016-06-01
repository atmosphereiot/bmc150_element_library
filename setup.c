	//Init with default i2c address
	bmc150_init(BMC150_SDO_LOW);
	//Setup Magnometer
	bmc150_mag_set_power(BMC150_MAG_POWER_ACTIVE);
	bmc150_mag_set_preset(BMC150_MAG_PRESET_HIGH_ACCURACY);
	//Setup Accelerometer
	bmc150_set_accel_mode(BMC150_MODE_2G);
	bmc150_set_bandwidth(BMC150_BANDWIDTH_64MS);
