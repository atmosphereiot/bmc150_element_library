import json
import os
import base64
import sys
import traceback
import glob

driver_path = '.'
driver_name = glob.glob("*.json")[0].replace('.json', '')
print("Building: %s" % driver_name)

if os.path.isdir(driver_path):
	
	driver_data = {}
	
	try:
		driver_json_file = open('%(driver_name)s.json' % {'driver_name':driver_name}, 'r')
		#print(driver_json_file)
		driver_json = driver_json_file.read()
		#print(driver_json)
		driver_data = json.loads(driver_json)
		print(driver_data)
		driver_json_file.close()
		
	except:
		print('No JSON descriptor for %s!' % driver_name)
		exit()
	
	try:
		setup_code_file = open('%(driver_path)s/setup.c' % {'driver_path':driver_path}, 'r')
		setup_code = unicode(setup_code_file.read(), errors='replace')
		setup_code_file.close()
		
		driver_data['code']['setup'] = setup_code
		
	except:
		pass
	
	try:
		global_code_file = open('%(driver_path)s/global.c' % {'driver_path':driver_path}, 'r')
		global_code = unicode(global_code_file.read(), errors='replace')
		global_code_file.close()
		
		driver_data['code']['global'] = global_code
		
	except:
		pass
	
	try:
		connected_code_file = open('%(driver_path)s/connected.c' % {'driver_path':driver_path}, 'r')
		connected_code = unicode(connected_code_file.read(), errors='replace')
		connected_code_file.close()
		
		driver_data['code']['connected'] = connected_code
		
	except:
		pass
	
	try:
		interval_code_file = open('%(driver_path)s/interval.c' % {'driver_path':driver_path}, 'r')
		interval_code = unicode(interval_code_file.read(), errors='replace')
		interval_code_file.close()
		
		driver_data['code']['interval'] = interval_code;
		
	except:
		pass
	
	for element_name in driver_data.get('elements', {}):
		
		try:
			try:
				element_code_file = open('%(driver_path)s/element_code/%(element_name)s.c' % {'driver_path':driver_path, 'element_name':element_name}, 'r')
				element_code = unicode(element_code_file.read(), errors='replace')
				
				driver_data['elements'][element_name]['code'] = element_code
				
				try:
					icon_file = open('%(driver_path)s/element_code/%(element_name)s.png' % {'driver_path':driver_path, 'element_name':element_name}, 'rb')
					driver_data['elements'][element_name]['icon'] = 'data:image/png;base64,' + icon_file.read().encode('base64')
					icon_file.close()
					
				except:
					print >> sys.stderr, "Unexpected error:", sys.exc_info()[0]
					traceback.print_exc(file=sys.stderr)

				element_code_file.close()
				
			except:
				pass
			
		except:
			print >> sys.stderr, "Unexpected error:", sys.exc_info()[0]
			traceback.print_exc(file=sys.stderr)
			
	driver_data['files'] = {}
	
	for platform_name in os.listdir('%(driver_path)s/driver' % {'driver_path':driver_path}):
		platform_path = '%(driver_path)s/driver/%(platform_name)s' % {'driver_path':driver_path, 'platform_name':platform_name}
		
		if os.path.isdir(platform_path):
			driver_data['files'][platform_name] = {}
			
			folder_types = ['objects', 'headers', 'other']
			
			for type_name in folder_types:
				driver_data['files'][platform_name][type_name] = {}
				
				try:
					for object_filename in os.listdir('%(platform_path)s/%(type_name)s' % {'platform_path':platform_path, 'type_name':type_name}):
						object_location = '%(platform_path)s/%(type_name)s/%(object_filename)s' % {'platform_path':platform_path, 'object_filename':object_filename, 'type_name':type_name}
					
						if os.path.isfile(object_location):
							object_file = open(object_location, 'r')
							driver_data['files'][platform_name][type_name][object_filename] = unicode(object_file.read(), errors='replace')
							object_file.close()

				except:
					pass

	output_eel_file = open('%s.eel' % driver_name, 'w')
	output_eel_file.write(json.dumps(driver_data, sort_keys=True, indent=4, separators=(',', ': ')))
	output_eel_file.close()

				