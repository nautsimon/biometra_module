import clr
from pathlib import Path 
import time as time

dotnet_path = Path(__file__).resolve().parent / 'dotnet' / 'BiometraLibraryNet'
clr.AddReference(str(dotnet_path))
import BiometraLibrary 

from biometra_driver.errors import ErrorResponse
from biometra_driver.functions import Functions


class Biometra:
    functions: Functions
    
    
    def __init__(self, serial_port: str):
        self.functions = Functions()
        self.protocol_status = 1
        
        
    def ready(self) -> bool:
        """True if the device is not busy"""
        return self.__memory_map.bits[1915]
    
    def has_error(self) -> bool:
        """True if the device has an error"""
        return self.__memory_map.bits[1814]

    def get_error(self) -> Optional[ErrorResponse]:
        """If the device has an error, return it. Else, return None."""
        if self.has_error:
            return ErrorResponse.from_error_code(self.__memory_map.data[200])
        return None
    
    def main(self, prog) -> None:
        """
        Main function, runs given program, waits until its complete,
        then verifies that the plate is clear
        
        Parameters
        -----------
        prog: ID number of desired program to run
        """
        self.functions.run_pcr_program(prog)
        self.functions.countdown()
        plate_status = self.wait_until_ready()
        self.protocol_status = plate_status
        