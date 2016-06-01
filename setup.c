	//Init with default i2c address
	BMC150_init(BMC150_SDO_LOW);
	//Setup Magnometer
	BMC150_mag_set_power(BMC150_MAG_POWER_ACTIVE);
	BMC150_mag_set_preset(BMC150_MAG_PRESET_HIGH_ACCURACY);
	//Setup Accelerometer
	BMC150_set_accel_mode(BMC150_MODE_2G);
	BMC150_set_bandwidth(BMC150_BANDWIDTH_64MS);
