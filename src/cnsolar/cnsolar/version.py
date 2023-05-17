'''
Version file for cnosolar library.

WARNING
---
Do not include any code in this file other than the definition of the __version__ variable. Please use https://semver.org/. 

CHANGE LOG
---
0.1.0 : Initial version of the library @ October 20th, 2021.
0.1.1 : Download files update and AC '_multi' pvlib functions addition @ February 18th, 2022.

0.2.0 : @ September 12th, 2022.
    1. The following parameters are eliminated from the JSON file since they are not required.
        - surface_type.
        - inverters_database.
        - inverter_name.
        - modules_database.
        - module_name.
        - module_type.
        - racking_model.
    2. Changed 'asi' module technology to 'amorphous' for clarity.
    3. 'alpha_sc' unity changed from A/ºC to %/ºC.
    4. 'beta_oc' unity changed from V/ºC to %/ºC.
    5. Module parameters were revised to maintain those established in Acuerdo 1571.
    6. Inverter parameters (for sandia or pvwatts 'ac_model') were revised to maintain those established in Acuerdo 1571.
'''

__version__ = '0.2.0'