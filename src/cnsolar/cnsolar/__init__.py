# Warnings
import warnings
warnings.filterwarnings(action='ignore')

import logging
logging.basicConfig(level=logging.INFO)
logging.getLogger('numexpr').setLevel(logging.WARNING)

# Scripts
from cnsolar import cell_temperature
from cnsolar import cen
from cnsolar import data
from cnsolar import def_pvsystem
from cnsolar import energia_minima
#from cnsolar import gui_config
#from cnsolar import gui_protocols
from cnsolar import irradiance_models
from cnsolar import location_data
from cnsolar import pipeline
from cnsolar import production
from cnsolar import pvstructure
from cnsolar import verification
from cnsolar import pvsyst

if __name__ == '__main__':
    print(f'Successfully executed from {__name__}.')