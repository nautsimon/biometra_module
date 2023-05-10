import clr
from pathlib import Path 
import time as time

dotnet_path = Path(__file__).resolve().parent / 'dotnet' / 'BiometraLibraryNet'
clr.AddReference(str(dotnet_path))
import BiometraLibrary 

# from errors import ErrorResponse
from biometra_driver.functions import Functions

#TODO: experiement with async error checking

class Biometra:

    def __init__(self):
        self.functions = Functions()
        self.protocol_status = 1
        self.robot_status = self.functions.robot_status
        self.status_msg = self.functions.status_msg
        
    def check_error_logs(self):
        err, all_error_log_files = self.functions.error_log_cmds.GetAllErrorLogFiles(self.functions.device_desc)
        data_list = all_error_log_files.DataList
        return data_list

    def get_error(self, new_data_list):
        """If the device has an error, return it. Else, return None."""
        if self.data_list == new_data_list:
            pass
        #TODO
    
    def run_program(self, prog) -> None:
        """
        Main function, runs given program, waits until its complete,
        then verifies that the plate is clear
        
        Parameters
        -----------
        prog: ID number of desired program to run
        """
        # pull error log files
        # self.data_list = self.check_error_logs()
        #close lid (put into run pcr program)
        self.functions.run_pcr_program(prog)
        time.sleep(10)
        self.functions.countdown()
        plate_status = self.functions.wait_until_ready()
        #examine error log files again, see if any new ones
        # new_data_list = self.has_error()
        # self.get_error(new_data_list)
        self.protocol_status = plate_status
        self.status_msg = self.functions.status_msg
        self.functions.run_status = self.functions.get_run_status()
        if self.status_msg == -1 or self.protocol_status == -1:
            pass
            # TODO: run get_error here, accesses error.py for list of possible error codes
            
if __name__ == "__main__":
    pass