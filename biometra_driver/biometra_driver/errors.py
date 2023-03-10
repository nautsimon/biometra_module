
#init

# has_error
# get_error

# looping function to wait until plate is ready



"""
err, all_error_log_files = error_log_cmds.GetAllErrorLogFiles(device_desc)
self.data_list = all_error_log_files.DataList
***run protocol***
new_data_list = all_error_log_files.DataList
if new_data_list == self.data_list:
    no new errors occurred
else:
    TODO: need a way to translate error code
    self.data_list = new_data_list
"""