import clr
from pathlib import Path 

dotnet_path = Path(__file__).resolve().parent / 'dotnet' / 'BiometraLibraryNet'
clr.AddReference(str(dotnet_path))
import BiometraLibrary 

from biometra_driver.errors import ErrorResponse
from biometra_driver.functions import Functions


class Biometra:
    functions: Functions
    
    
    __serial_port: SerialPort
    __memory_map: MemoryMap
    plate_handler: PlateHandler
    """
    Exposes functionality related to plate handling
    """
    shaker_controller: ShakerController
    """
    Exposes tower shaker functionality
    """
    climate_controller: ClimateController
    """
    Exposes functionality related to temperature and CO2
    """

    def __init__(self, serial_port: str):
        self.functions = Functions()
        
        
    @property
    def ready(self) -> bool:
        """True if the device is not busy"""
        return self.__memory_map.bits[1915]

    @property
    def has_error(self) -> bool:
        """True if the device has an error"""
        return self.__memory_map.bits[1814]

    def initialize(self) -> None:
        """Initialize the device"""
        self.__memory_map.bits[1900] = True
        time.sleep(0.5)  # else it does not work
        self.__memory_map.bits[1801] = True

    def open_connection(self) -> None:
        """Open the connection"""
        self.__serial_port.open()

    def close_connection(self) -> None:
        """Close the connection"""
        self.__serial_port.close()

    def get_error(self) -> Optional[ErrorResponse]:
        """If the device has an error, return it. Else, return None."""
        if self.has_error:
            return ErrorResponse.from_error_code(self.__memory_map.data[200])
        return None

    def wait_until_ready(self, timeout: float) -> None:
        """
        Block the current thread until the device is not busy anymore

        Parameters
        ----------
        timeout
            Timeout in seconds

        Raises
        ------
        TimeoutError
            If the device is still busy after the timeout
        ErrorResponse
            If the device has an error
        """
        end_time = datetime.now() + timedelta(seconds=timeout)
        while not (self.ready or self.has_error):
            if datetime.now() > end_time:
                raise TimeoutError(f"Device still busy after {timeout} seconds")

        error = self.get_error()
        if error is not None:
            raise error

    def __enter__(self) -> Stx:
        self.open_connection()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close_connection()
